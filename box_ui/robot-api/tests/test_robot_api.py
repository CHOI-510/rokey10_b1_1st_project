"""Run from robot-api: python -m unittest discover -s tests -v."""
import unittest
from unittest.mock import patch

from fastapi import HTTPException
from fastapi.testclient import TestClient
from app import main


class RobotApiTests(unittest.TestCase):
    def setUp(self):
        self.backend = patch.object(main, 'BACKEND', 'box')
        self.backend.start()
        self.addCleanup(self.backend.stop)
        self.client = TestClient(main.app)

    def test_start_and_stop_use_existing_ros_gateway(self):
        for command, path in [('start', '/start'), ('stop', '/estop'), ('cancel', '/cancel')]:
            with self.subTest(command=command), patch.object(main, 'box_request', return_value={'ok': True}) as request:
                response = self.client.post(f'/robot/{command}')
                self.assertEqual(response.status_code, 200)
                self.assertEqual(response.json()['state'], 'unknown')
                request.assert_called_once_with(path, 'POST')

    def test_refused_command_is_not_success(self):
        with patch.object(main, 'box_request', return_value={'ok': False, 'message': 'not READY'}):
            response = self.client.post('/robot/start')
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json()['detail'], 'not READY')

    def test_unavailable_controller_does_not_fall_back_to_mock(self):
        with patch.object(main, 'box_request', side_effect=HTTPException(503, 'offline')):
            self.assertEqual(self.client.post('/robot/start').status_code, 503)

    def test_actual_status_mapping(self):
        for stage, job, connected, expected in [
            ('READY', 'DONE', True, 'idle'), ('PREFLIGHT', 'BEFORE', True, 'countdown'),
            ('STANDBY', 'BEFORE', True, 'countdown'), ('JUDGING', 'RUNNING', True, 'running'),
            ('READY', 'STOPPED', True, 'stopped'), ('READY', 'ERROR', True, 'error'),
            ('READY', 'BEFORE', False, 'unknown'),
        ]:
            with self.subTest(stage=stage, job=job, connected=connected):
                with patch.object(main, 'box_request', return_value={
                    'stage': stage, 'status': job, 'control_connected': connected,
                    'start_at': '2026-11-20T01:10:00+00:00', 'box_seq': 3,
                }):
                    status = self.client.get('/robot/status').json()
                self.assertEqual(status['state'], expected)
                self.assertEqual(status['can_start'], connected and stage == 'READY')
                self.assertEqual(status['box_seq'], 3)

    def test_inventory_rejection_and_validation(self):
        with patch.object(main, 'box_request', return_value={'ok': False, 'message': 'not READY'}):
            self.assertEqual(self.client.post('/robot/inventory', json=dict(a=0,b=0,c=0,d=0,u=0)).status_code, 409)
        self.assertEqual(self.client.post('/robot/inventory', json=dict(a=7,b=0,c=0,d=0,u=0)).status_code, 422)

    def test_stop_transport_failure_is_not_retried(self):
        from urllib.error import URLError
        with patch.object(main, 'urlopen', side_effect=URLError('offline')) as request:
            response = self.client.post('/robot/stop')
        self.assertEqual(response.status_code, 503)
        self.assertIn('명령 처리 여부', response.json()['detail'])
        self.assertEqual(request.call_count, 1)


if __name__ == '__main__':
    unittest.main()

class RobotConnectionTests(unittest.TestCase):
    def test_only_live_real_robot_is_connected(self):
        for monitor, mode, link, state, live, expected in [
            ('dsr', 'real', True, 1, True, True),
            ('dsr', 'real', False, 1, True, False),
            ('dsr', 'real', True, None, True, False),
            ('dsr', 'virtual', True, 1, True, False),
            ('fake', 'real', True, 1, True, False),
            ('dsr', 'real', True, 1, False, False),
            ('dsr', '', True, 1, True, False),
        ]:
            with self.subTest(monitor=monitor, mode=mode, link=link, state=state, live=live):
                data = dict(stage='READY', control_connected=live, robot_monitor=monitor,
                            robot_hardware_mode=mode, robot_connected=link, robot_state=state)
                with patch.object(main, 'BACKEND', 'box'), patch.object(main, 'box_request', return_value=data):
                    response = TestClient(main.app).get('/robot/status')
                    self.assertEqual(response.status_code, 200)
                    self.assertEqual(response.json()['robot_connected'], expected)

    def test_mock_never_reports_hardware_connected(self):
        with patch.object(main, 'BACKEND', 'mock'):
            self.assertFalse(TestClient(main.app).get('/robot/status').json()['robot_connected'])
