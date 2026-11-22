import unittest
from unittest.mock import Mock
from box_control.control import Control, State


class StatusContractTests(unittest.TestCase):
    def test_hardware_mode_is_published_with_every_controller_status(self):
        for mode in ('real', 'virtual', 'unknown'):
            with self.subTest(mode=mode):
                control = Control.__new__(Control)
                control.io = Mock()
                control.state = State.READY
                control.robot_hardware_mode = mode
                control.job_id = 'job'
                control.box_seq = 0
                control._current_box = control._current_class = ''
                control.inventory = dict.fromkeys('ABCDU', 0)
                control.count_mismatches = []
                control.recovery_retry_total = 0
                control._inventory_needs_recheck = False
                control.publish_status()
                fields = control.io.publish_status.call_args.kwargs
                self.assertEqual(fields['robot_hardware_mode'], mode)
                self.assertEqual(fields['state'], 'READY')
