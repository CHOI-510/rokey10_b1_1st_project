import unittest
from types import SimpleNamespace

from box_control.recovery import Recovery


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.warn_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warn(self, message):
        self.warn_messages.append(message)


class FakeControlIO:
    _monitor = 'dsr'

    def __init__(self):
        self.stop = None

    def declare_parameter(self, name, default):
        return SimpleNamespace(value='robot' if name == 'clear_zone' else default)

    def stop_reason(self):
        return self.stop


class FakeRobot:
    def __init__(self, fail_move_at=None, grabbed=True):
        self.current_pose = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
        self.fail_move_at = fail_move_at
        self.grabbed = grabbed
        self.report = None
        self.moves = []
        self.grip_opens = 0
        self.grip_closes = []
        self.waits = []
        self.stops = []
        self.moved = False
        self.unbound = False

    def bind(self, report):
        self.report = report

    def unbind(self):
        self.report = None
        self.unbound = True

    def _check(self):
        if self.report:
            self.report.check()

    def pose(self):
        return list(self.current_pose)

    def movel(self, pose, vel, acc):
        self._check()
        self.moved = True
        self.moves.append((list(pose), vel, acc))
        if self.fail_move_at == len(self.moves):
            raise RuntimeError('simulated motion failure')
        self.current_pose = list(pose)

    def grip_open(self):
        self._check()
        self.moved = True
        self.grip_opens += 1

    def grip_close(self, force_n):
        self._check()
        self.moved = True
        self.grip_closes.append(force_n)
        return SimpleNamespace(grabbed=self.grabbed, width_mm=42.0)

    def wait(self, seconds):
        self._check()
        self.waits.append(seconds)

    def needs_stop(self):
        return self.moved

    def stop(self, emergency=False):
        self.stops.append(emergency)


class RecoverySweepTests(unittest.TestCase):
    def make_recovery(self, robot):
        io = FakeControlIO()
        log = FakeLogger()
        recovery = Recovery(io, log)
        recovery._connect = lambda: robot
        return recovery, log

    def test_robot_mode_picks_tool_sweeps_and_returns_it(self):
        robot = FakeRobot()
        recovery, _ = self.make_recovery(robot)

        result = recovery.clear_measure_zone({})

        self.assertIsNone(result)
        self.assertEqual(len(robot.moves), 10)
        self.assertEqual(robot.moves[0][0], [-178.0, -239.0, 160.0, 70.0, 180.0, 70.0])
        self.assertEqual(robot.moves[4][0], [300.0, -280.0, 101.0, 26.6, -180.0, -62.0])
        self.assertEqual(robot.moves[5][0], [300.0, -44.0, 103.0, 26.6, -180.0, -62.0])
        self.assertEqual(robot.moves[-1][0], [-178.0, -239.0, 160.0, 70.0, 180.0, 70.0])
        self.assertEqual(robot.grip_closes, [10.0])
        self.assertEqual(robot.grip_opens, 2)
        self.assertEqual(robot.waits, [0.5, 0.5])
        self.assertFalse(recovery.tool_held)
        self.assertEqual(recovery.tool_phase, 'complete')
        self.assertTrue(robot.unbound)

    def test_motion_failure_while_holding_tool_stops_without_opening(self):
        robot = FakeRobot(fail_move_at=6)
        recovery, log = self.make_recovery(robot)

        result = recovery.clear_measure_zone({})

        self.assertEqual(result, 'RECOVERY_FAILED')
        self.assertTrue(recovery.tool_held)
        self.assertEqual(recovery.tool_phase, 'sweeping')
        self.assertEqual(robot.grip_opens, 1)
        self.assertEqual(robot.stops, [True])
        self.assertTrue(any('도구 파지 상태에서 중단' in message for message in log.warn_messages))
        self.assertTrue(robot.unbound)

    def test_failed_tool_grip_returns_recovery_failed(self):
        robot = FakeRobot(grabbed=False)
        recovery, _ = self.make_recovery(robot)

        result = recovery.clear_measure_zone({})

        self.assertEqual(result, 'RECOVERY_FAILED')
        self.assertFalse(recovery.tool_held)
        self.assertEqual(recovery.tool_phase, 'gripping_tool')
        self.assertTrue(robot.unbound)


if __name__ == '__main__':
    unittest.main()
