from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import Mock
from box_comm.safe_off_recovery import recover_safe_off


class FakeIo:
    def __init__(self, states=(3,1)):
        self._stop_lock=threading.Lock()
        self._stop_revision=1
        self._recovering=False
        self._stop_hold_until=0
        self._armed=False
        self._active=None
        self._stop=('EMERGENCY_STOP','ui')
        self._last_alarm='old'
        self._stop_event=threading.Event();self._stop_event.set()
        self._monitor='dsr'
        self._dsr=SimpleNamespace(GetRobotState=SimpleNamespace(Request=SimpleNamespace),SetRobotControl=SimpleNamespace(Request=SimpleNamespace))
        self.states=deque(states)
        self._cli_robot_state=Mock();self._cli_robot_state.service_is_ready.return_value=True
        self._cli_robot_control=Mock();self._cli_robot_control.service_is_ready.return_value=True
        def read(_request):
            future=Future();future.set_result(SimpleNamespace(success=True,robot_state=self.states.popleft()));return future
        def reset(_request):
            future=Future();future.set_result(SimpleNamespace(success=True));return future
        self._cli_robot_state.call_async.side_effect=read
        self._cli_robot_control.call_async.side_effect=reset
        self._robot_move_stop_hold=Mock()


class SafeOffRecoveryTests(unittest.TestCase):
    def test_safe_off_reset_then_actual_standby_clears_latch(self):
        io=FakeIo();ok,message=recover_safe_off(io)
        self.assertTrue(ok);self.assertIn('STANDBY',message)
        self.assertIsNone(io._stop);self.assertFalse(io._stop_event.is_set())
        self.assertEqual(io._cli_robot_control.call_async.call_args.args[0].robot_control,3)
        self.assertEqual(io._cli_robot_control.call_async.call_count,1)
        io._robot_move_stop_hold.assert_not_called()

    def test_already_standby_only_clears_software_stop(self):
        io=FakeIo((1,));self.assertTrue(recover_safe_off(io)[0])
        io._cli_robot_control.call_async.assert_not_called()

    def test_hardware_emergency_and_other_states_are_not_reset(self):
        for state in (0,2,5,6,7,8,9,10,15):
            with self.subTest(state=state):
                io=FakeIo((state,));self.assertFalse(recover_safe_off(io)[0])
                io._cli_robot_control.call_async.assert_not_called()
                self.assertTrue(io._stop_event.is_set())

    def test_active_job_and_stop_hold_block_recovery(self):
        for field,value in [('_armed',True),('_active',object()),('_stop_hold_until',time.monotonic()+10),('_recovering',True)]:
            io=FakeIo();setattr(io,field,value)
            self.assertFalse(recover_safe_off(io)[0]);io._cli_robot_state.call_async.assert_not_called()

    def test_new_stop_during_reset_is_not_cleared(self):
        io=FakeIo()
        def reset(_request):
            io._stop_revision+=1
            future=Future();future.set_result(SimpleNamespace(success=True));return future
        io._cli_robot_control.call_async.side_effect=reset
        self.assertFalse(recover_safe_off(io)[0])
        self.assertTrue(io._stop_event.is_set());io._robot_move_stop_hold.assert_called_once()

    def test_reset_refusal_keeps_emergency_and_reissues_stop(self):
        io=FakeIo()
        future=Future();future.set_result(SimpleNamespace(success=False))
        io._cli_robot_control.call_async.side_effect=None
        io._cli_robot_control.call_async.return_value=future
        self.assertFalse(recover_safe_off(io)[0]);self.assertTrue(io._stop_event.is_set())
        io._robot_move_stop_hold.assert_called_once()

    def test_read_service_missing_never_sends_reset(self):
        io=FakeIo();io._cli_robot_state.service_is_ready.return_value=False
        self.assertFalse(recover_safe_off(io)[0]);io._cli_robot_control.call_async.assert_not_called()

    def test_repeat_stop_during_recovery_is_reissued(self):
        from box_comm.control_interface import ControlInterface
        io=FakeIo();io._recovering=True
        io.get_logger=Mock(return_value=Mock());io.log_event=Mock()
        self.assertTrue(ControlInterface.request_stop(io,'EMERGENCY_STOP','second click'))
        self.assertEqual(io._stop_revision,2)
        self.assertEqual(io._stop,('EMERGENCY_STOP','second click'))
        io._robot_move_stop_hold.assert_called_once()
