import unittest
from unittest.mock import Mock

from box_control.control import Control


class ControlRecoveryTests(unittest.TestCase):
    def make_control(self):
        control = Control.__new__(Control)
        control.io = Mock()
        control.io.stop_reason.return_value = None
        control.io.progress.return_value = ('measuring_length1', 0.0)
        control.log = Mock()
        control.recovery_retries = 0
        control.recovery_retry_total = 0
        control._current_class = 'A'
        control.clear_measure_zone = Mock(return_value=None)
        return control

    def test_judge_not_found_clears_zone_and_restarts_scan(self):
        control = self.make_control()

        result = control.on_part_error('NOT_FOUND', 'judge')

        self.assertIsNone(result)
        control.clear_measure_zone.assert_called_once_with('judge', 'NOT_FOUND')
        control.io.log_event.assert_called_once()
        self.assertEqual(control.recovery_retries, 1)
        self.assertEqual(control.recovery_retry_total, 1)
        self.assertEqual(control._current_class, '')

    def test_not_found_from_other_part_does_not_trigger_recovery(self):
        control = self.make_control()

        result = control.on_part_error('NOT_FOUND', 'scan')

        self.assertEqual(result, 'NOT_FOUND')
        control.clear_measure_zone.assert_not_called()
        self.assertEqual(control.recovery_retries, 0)

    def test_not_found_shares_recovery_retry_limit(self):
        control = self.make_control()
        control.recovery_retries = 3

        result = control.on_part_error('NOT_FOUND', 'judge')

        self.assertEqual(result, 'RETRY_LIMIT')
        control.clear_measure_zone.assert_not_called()
