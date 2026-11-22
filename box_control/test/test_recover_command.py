import threading
import unittest
from unittest.mock import Mock
from box_control.control import Control,State


class RecoverCommandTests(unittest.TestCase):
    def control(self,state=State.READY):
        control=Control.__new__(Control);control.state=state
        control._cmd_lock=threading.Lock();control._start_req=threading.Event()
        control.io=Mock();control.io.recover_safe_off.return_value=(True,'복구 완료')
        control.publish_status=Mock();control.recovery_id=''
        return control

    def test_recovery_does_not_start_a_job(self):
        control=self.control();ok,message=control.on_job_command('recover','ui')
        self.assertTrue(ok);self.assertFalse(control._start_req.is_set())
        self.assertEqual(control.state,State.READY);self.assertTrue(control.recovery_id)
        control.io.set_job_status.assert_called_once_with('BEFORE','')

    def test_failure_does_not_publish_recovery_or_clear_status(self):
        control=self.control();control.io.recover_safe_off.return_value=(False,'hardware emergency')
        self.assertFalse(control.on_job_command('recover','ui')[0])
        self.assertEqual(control.recovery_id,'');control.publish_status.assert_not_called()
        control.io.set_job_status.assert_not_called()

    def test_running_and_pending_start_are_rejected(self):
        for state in (State.RUNNING,State.SCANNING,State.STANDBY):
            control=self.control(state);self.assertFalse(control.on_job_command('recover','ui')[0])
            control.io.recover_safe_off.assert_not_called()
        control=self.control();control._start_req.set()
        self.assertFalse(control.on_job_command('recover','ui')[0])
