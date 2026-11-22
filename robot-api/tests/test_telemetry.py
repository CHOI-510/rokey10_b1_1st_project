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

class CurrentControllerContractTests(unittest.TestCase):
    def test_current_controller_mode_enables_real_telemetry(self):
        bridge = RosbridgeClient('ws://test')
        bridge._accept_status({'state': 'INVENTORY_CHECK', 'job_id': 'current-job',
                               'robot_hardware_mode': 'real', 'robot_monitor': 'dsr',
                               'robot_connected': True, 'robot_state': 2})
        self.assertTrue(bridge.real_robot_available())
        with patch.object(main, 'BACKEND', 'rosbridge'), patch.object(main, 'bridge', bridge):
            body = TestClient(main.app).get('/robot/status').json()
        self.assertTrue(body['robot_connected'])
        self.assertEqual(body['robot_hardware_mode'], 'real')
        self.assertEqual(body['state'], 'running')

    def test_missing_mode_is_reported_not_assumed_real(self):
        bridge = RosbridgeClient('ws://test')
        bridge._accept_status({'state': 'READY', 'robot_monitor': 'dsr',
                               'robot_connected': True, 'robot_state': 1})
        with patch.object(main, 'BACKEND', 'rosbridge'), patch.object(main, 'bridge', bridge):
            body = TestClient(main.app).get('/robot/status').json()
        self.assertFalse(body['robot_connected'])
        self.assertIn('robot_hardware_mode', body['robot_hint'])

class JointTopicTests(unittest.TestCase):
    def message(self):
        stamp=time.time()
        return {'header':{'stamp':{'sec':int(stamp),'nanosec':int((stamp%1)*1e9)}},
                'name':['rg2_finger_joint','joint_6','joint_5','joint_4','joint_3','joint_2','joint_1'],
                'position':[0,0,0,0,0,0,1.5707963267948966]}

    def test_reordered_measured_radians_become_degrees(self):
        reader=TelemetryReader('ws://test',lambda:True)
        reader._accept_joints(self.message())
        sample=reader.snapshot()
        self.assertTrue(sample['connected'])
        self.assertEqual(sample['source'],'dsr_joint_states')
        self.assertAlmostEqual(sample['joints_deg'][0],90)
        self.assertFalse(sample['tcp_connected'])

    def test_tcp_timeout_cannot_freeze_live_joint_angles(self):
        reader=TelemetryReader('ws://test',lambda:True)
        reader._sample_once(ReadSocket())
        reader._seen=time.monotonic()-100
        reader._online=False
        reader._accept_joints(self.message())
        sample=reader.snapshot()
        self.assertTrue(sample['connected'])
        self.assertFalse(sample['tcp_connected'])
        self.assertGreater(sample['tcp_age_ms'],99000)
        self.assertAlmostEqual(sample['joints_deg'][0],90)

    def test_old_or_duplicate_headers_do_not_refresh_age(self):
        reader=TelemetryReader('ws://test',lambda:True)
        message=self.message();reader._accept_joints(message)
        original=reader._joint_seen
        reader._accept_joints(message)
        self.assertEqual(reader._joint_seen,original)
        message['header']['stamp']['sec']-=20
        reader._accept_joints(message)
        self.assertEqual(reader._joint_seen,original)
        reader._joint_seen-=5
        self.assertFalse(reader.snapshot()['connected'])

    def test_fake_and_missing_joint_values_are_not_measurements(self):
        reader=TelemetryReader('ws://test',lambda:False)
        reader._accept_joints(self.message())
        self.assertFalse(reader.snapshot()['connected'])
        reader.real_robot_available=lambda:True
        message=self.message();message['name'][1]='joint_1'
        with self.assertRaises(ValueError):reader._accept_joints(message)
