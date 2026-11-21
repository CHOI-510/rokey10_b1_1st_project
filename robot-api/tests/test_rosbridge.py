import json
import time
import unittest
from unittest.mock import patch

from fastapi import HTTPException
from fastapi.testclient import TestClient
from app import main
from app.rosbridge import RosbridgeClient, RosbridgeError


class FakeSocket:
    def __init__(self, values=None, result=True):
        self.values = values or {'success': True, 'message': 'accepted'}
        self.result = result
        self.sent = None
        self.closed = False
        self.received = 0

    def send(self, message):
        self.sent = json.loads(message)

    def settimeout(self, timeout):
        pass

    def recv(self):
        self.received += 1
        # An unrelated response must not be accepted as our command result.
        return json.dumps({'op': 'service_response', 'id': 'unrelated' if self.received == 1 else self.sent['id'],
                           'result': self.result, 'values': self.values})

    def close(self):
        self.closed = True


class RosbridgeProtocolTests(unittest.TestCase):
    def test_correlated_service_response(self):
        client = RosbridgeClient('ws://test')
        socket = FakeSocket()
        with patch('app.rosbridge.create_connection', return_value=socket):
            result = client.call_service('/job_command', {'command': 'start', 'source': 'ui'})
        self.assertTrue(result['success'])
        self.assertEqual(socket.sent['op'], 'call_service')
        self.assertEqual(socket.sent['service'], '/job_command')
        self.assertEqual(socket.sent['args']['source'], 'ui')
        self.assertEqual(socket.received, 2)
        self.assertTrue(socket.closed)

    def test_rosbridge_protocol_failure_is_error(self):
        with patch('app.rosbridge.create_connection', return_value=FakeSocket(result=False)):
            with self.assertRaises(RosbridgeError):
                RosbridgeClient('ws://test').call_service('/job_command', {'command': 'start'})

    def test_command_is_not_retried(self):
        with patch('app.rosbridge.create_connection', side_effect=OSError('offline')) as connect:
            with self.assertRaises(RosbridgeError):
                RosbridgeClient('ws://test').call_service('/emergency_stop', {'source': 'ui'})
        self.assertEqual(connect.call_count, 1)

    def test_freshness_and_disconnect(self):
        client = RosbridgeClient('ws://test')
        self.assertFalse(client.snapshot()['control_connected'])
        client._accept_status({'state': 'READY'})
        self.assertTrue(client.snapshot()['control_connected'])
        with patch('app.rosbridge.time.monotonic', return_value=time.monotonic() + 20):
            self.assertFalse(client.snapshot()['control_connected'])
        client._online = False
        self.assertFalse(client.snapshot()['control_connected'])

    def test_workflow_preserves_stage_and_freezes_elapsed_time(self):
        client = RosbridgeClient('ws://test')
        for stage in ['BEFORE', 'PREFLIGHT', 'STANDBY', 'RUNNING', 'SCANNING', 'SCAN_DONE',
                      'JUDGING', 'JUDGE_DONE', 'PLACING', 'PLACE_DONE', 'DONE', 'READY']:
            client._accept_status({'state': stage, 'job_id': 'job-1', 'standby_sec': 15, 'box_seq': 4})
            self.assertEqual(client.snapshot()['stage'], stage)
        final = client.snapshot()
        self.assertIsNotNone(final['started_at'])
        self.assertIsNotNone(final['ended_at'])
        client._accept_status({'state': 'READY', 'job_id': 'job-1'})
        self.assertEqual(client.snapshot()['ended_at'], final['ended_at'])
        client._accept_status({'state': 'BEFORE', 'job_id': 'job-2'})
        self.assertIsNone(client.snapshot()['started_at'])
        self.assertIsNone(client.snapshot()['ended_at'])

    def test_countdown_does_not_reset_on_periodic_messages(self):
        client = RosbridgeClient('ws://test')
        client._accept_status({'state': 'STANDBY', 'standby_sec': 15})
        deadline = client.snapshot()['start_at']
        client._accept_status({'state': 'STANDBY', 'standby_sec': 15})
        self.assertEqual(client.snapshot()['start_at'], deadline)

    def test_terminal_outcome_survives_ready(self):
        client = RosbridgeClient('ws://test')
        client._accept_status({'state': 'ERROR', 'reason': 'ERROR : test'})
        client._accept_status({'state': 'READY'})
        self.assertEqual(client.snapshot()['status'], 'ERROR')
        client._accept_status({'state': 'BEFORE'})
        self.assertEqual(client.snapshot()['status'], 'BEFORE')
        self.assertEqual(client.snapshot()['reason'], '')


class RosbridgeApiTests(unittest.TestCase):
    def setUp(self):
        backend = patch.object(main, 'BACKEND', 'rosbridge')
        backend.start()
        self.addCleanup(backend.stop)
        self.client = TestClient(main.app)

    def test_commands_use_rosbridge_and_preserve_ui_source(self):
        for command, service, args in [
            ('start', '/job_command', {'command': 'start', 'source': 'ui'}),
            ('cancel', '/job_command', {'command': 'cancel', 'source': 'ui'}),
            ('start_now', '/job_command', {'command': 'start_now', 'source': 'ui'}),
            ('stop', '/emergency_stop', {'source': 'ui'}),
        ]:
            with self.subTest(command=command), patch.object(main.bridge, 'call_service', return_value={'success': True}) as call, patch.object(main, 'box_request') as http:
                response = self.client.post(f'/robot/{command}')
                self.assertEqual(response.status_code, 200)
                self.assertEqual(response.json()['mode'], 'rosbridge')
                self.assertEqual(response.json()['state'], 'unknown')
                call.assert_called_once_with(service, args)
                http.assert_not_called()

    def test_refusal_and_disconnect_do_not_report_success(self):
        with patch.object(main.bridge, 'call_service', return_value={'success': False, 'message': 'not READY'}):
            self.assertEqual(self.client.post('/robot/start').status_code, 409)
        with patch.object(main.bridge, 'call_service', side_effect=RosbridgeError('offline')):
            self.assertEqual(self.client.post('/robot/stop').status_code, 503)

    def test_http_state_cannot_hide_missing_rosbridge(self):
        with patch.object(main.bridge, 'snapshot', return_value={'stage': 'READY', 'control_connected': False}), patch.object(main, 'box_request', return_value={'control_connected': True, 'stage': 'READY'}):
            response = self.client.get('/robot/status').json()
        self.assertFalse(response['connected'])
        self.assertFalse(response['can_start'])
        self.assertEqual(response['state'], 'unknown')

    def test_ros_topic_is_used_when_http_is_unavailable(self):
        with patch.object(main.bridge, 'snapshot', return_value={'stage': 'JUDGING', 'control_connected': True, 'box_seq': 2}), patch.object(main, 'box_request', side_effect=HTTPException(503, 'offline')) as http:
            response = self.client.get('/robot/status').json()
        self.assertEqual(response['state'], 'running')
        self.assertEqual(response['stage'], 'JUDGING')
        self.assertEqual(response['box_seq'], 2)
        http.assert_not_called()


if __name__ == '__main__':
    unittest.main()
