import json
import time
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient
from app import main
from app.rosbridge import RosbridgeClient
from app.telemetry import TelemetryReader, finite_six


class ReadSocket:
    def __init__(self):
        self.sent = []

    def send(self, payload):
        self.sent.append(json.loads(payload))

    def settimeout(self, _timeout):
        pass

    def recv(self):
        request = self.sent[-1]
        values = ({'success': True, 'pos': [1, 2, 3, 4, 5, 6]}
                  if request['service'].endswith('get_current_posj') else
                  {'success': True, 'task_pos_info': [{'data': [300, -160, 90, 0, 180, 0, 2]}]})
        return json.dumps({'op': 'service_response', 'id': request['id'], 'result': True, 'values': values})


class TelemetryTests(unittest.TestCase):
    def test_base_sample_and_read_only_services(self):
        reader = TelemetryReader('ws://test', lambda: True)
        socket = ReadSocket()
        reader._sample_once(socket)
        data = reader.snapshot()
        self.assertTrue(data['connected'])
        self.assertEqual(data['joints_deg'], [1, 2, 3, 4, 5, 6])
        self.assertEqual(data['tcp_mm'], [300, -160, 90])
        self.assertEqual(data['orientation_deg'], [0, 180, 0])
        self.assertEqual(socket.sent[1]['args'], {'ref': 0})
        self.assertEqual([r['service'] for r in socket.sent], [
            '/dsr01/dsr_controller2/aux_control/get_current_posj',
            '/dsr01/dsr_controller2/aux_control/get_current_posx'])

    def test_invalid_numbers_are_not_published(self):
        for values in ([1]*5, [1]*7, [True]*6, [float('nan')]*6, [float('inf')]*6, ['1']*6):
            with self.subTest(values=values), self.assertRaises(ValueError):
                finite_six(values)

    def test_failure_does_not_merge_new_joints_with_old_tcp(self):
        reader = TelemetryReader('ws://test', lambda: True)
        reader._sample_once(ReadSocket())
        original = reader.snapshot()
        with patch.object(reader, '_read', side_effect=[{'pos': [9]*6}, {'task_pos_info': []}]):
            with self.assertRaises(ValueError):
                reader._sample_once(ReadSocket())
        self.assertEqual(reader.snapshot()['joints_deg'], original['joints_deg'])
        self.assertEqual(reader.snapshot()['received_at'], original['received_at'])

    def test_stale_and_disconnected_keep_last_pose(self):
        reader = TelemetryReader('ws://test', lambda: True)
        reader._sample_once(ReadSocket())
        reader._seen = time.monotonic() - 3
        self.assertFalse(reader.snapshot()['connected'])
        self.assertEqual(reader.snapshot()['tcp_mm'], [300, -160, 90])
        reader.real_robot_available = lambda: False
        self.assertFalse(reader.snapshot()['connected'])

    def test_only_real_fresh_control_allows_samples(self):
        bridge = RosbridgeClient('ws://test')
        for mode, monitor, connected, expected in [('real', 'dsr', True, True),
                ('virtual', 'dsr', True, False), ('real', 'fake', True, False), ('real', 'dsr', False, False)]:
            bridge._accept_status({'state': 'READY', 'robot_monitor': monitor, 'robot_hardware_mode': mode,
                                   'robot_connected': connected, 'robot_state': 1})
            self.assertEqual(bridge.real_robot_available(), expected)
        bridge._seen = time.monotonic()-20
        self.assertFalse(bridge.real_robot_available())

    def test_disconnect_during_sample_rejects_sample(self):
        reader = TelemetryReader('ws://test', lambda: False)
        with self.assertRaises(ValueError):
            reader._sample_once(ReadSocket())
        self.assertIsNone(reader.snapshot()['age_ms'])

    def test_api_returns_cached_sample_without_robot_command(self):
        reader = TelemetryReader('ws://test', lambda: True)
        reader._sample_once(ReadSocket())
        with patch.object(main, 'BACKEND', 'rosbridge'), patch.object(main.bridge, 'telemetry', reader), patch.object(main.bridge, 'call_service') as command:
            response = TestClient(main.app).get('/robot/telemetry')
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()['connected'])
            self.assertEqual(response.json()['tcp_mm'], [300, -160, 90])
            command.assert_not_called()

    def test_mock_does_not_claim_real_telemetry(self):
        with patch.object(main, 'BACKEND', 'mock'):
            response = TestClient(main.app).get('/robot/telemetry').json()
            self.assertFalse(response['connected'])
            self.assertIsNone(response['joints_deg'])
