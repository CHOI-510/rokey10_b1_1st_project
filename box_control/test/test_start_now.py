import threading
import unittest
from box_control.control import Control, State


class StartNowTests(unittest.TestCase):
    def make_control(self, state):
        control = Control.__new__(Control)
        control.state = state
        control._cmd_lock = threading.Lock()
        control._now_req = threading.Event()
        control._now_source = ''
        return control

    def test_ui_may_skip_countdown_without_starting_motion_in_callback(self):
        control = self.make_control(State.STANDBY)
        accepted, message = control.on_job_command('start_now', 'ui')
        self.assertTrue(accepted)
        self.assertTrue(control._now_req.is_set())
        self.assertEqual(control._now_source, 'ui')
        self.assertEqual(control.state, State.STANDBY)

    def test_other_stages_do_not_accept_immediate_start(self):
        for state in State:
            if state == State.STANDBY:
                continue
            with self.subTest(state=state):
                control = self.make_control(state)
                accepted, message = control.on_job_command('start_now', 'ui')
                self.assertFalse(accepted)
                self.assertFalse(control._now_req.is_set())
                self.assertIn('카운트다운 중이 아님', message)
