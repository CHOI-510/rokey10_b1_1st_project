"""판별 집기 재시도·GRASP_FAILED 시험 (11/18, 관제). ROS 없이 — 측정 로직은 건드리지 않고 집기 부분만 본다.
실행: python3 box_judge/test/test_judge_grip.py   (저장소 루트에서)"""
import sys
import types
import unittest
from pathlib import Path

_root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_root / 'box_comm'), str(_root / 'box_judge')]
from box_comm import common as _common  # noqa: E402
_ji = types.ModuleType('box_comm.judge_interface')     # judge_interface는 ROS(rclpy·메시지)가 필요 → PartError만 흉내
_ji.PartError = _common.PartError
sys.modules['box_comm.judge_interface'] = _ji
import box_judge.judge_algorithm as J  # noqa: E402
from box_comm.common import PartError  # noqa: E402
from box_comm.sync_grip import GripMissed  # noqa: E402


class Report:
    def __init__(self):
        self.logs = []

    def __call__(self, s):
        pass

    def log(self, m, level='INFO'):
        self.logs.append((level, m))


class SG:
    def __init__(self, fails):
        self.fails, self.calls = fails, 0

    def close_with_lift(self, est):
        self.calls += 1
        if self.calls <= self.fails:
            raise GripMissed('물체가 잡히지 않음 (가짜)')
        return {'width_mm': 40.0}


def make(fails, descent='clear'):
    a = J.JudgeAlgorithm.__new__(J.JudgeAlgorithm)
    a.sg, a._report = SG(fails), Report()
    a.p = {k: v for k, (v, _d) in J.PARAMS.items()}
    a.tip_clearance = 2.6
    a.downs = 0
    g = types.SimpleNamespace(open=lambda f, w: 109.5)
    a.io = types.SimpleNamespace(g=g, cur_z=lambda: 200.0, move_z_to=lambda z, v: None)
    a._tcp_open_tip = lambda raw, tip: 0.0

    def pos_descent(*args, **kw):
        a.downs += 1
        return descent, None
    a._pos_descent = pos_descent
    return a


class JudgeGrip(unittest.TestCase):
    def test_retry_then_success(self):
        a = make(fails=2)
        w, _tip = a._measure_width(20.0)
        self.assertEqual(w, 40.0)
        self.assertEqual(a.sg.calls, 3)
        self.assertEqual(a.downs, 3)                     # 처음 + 다시 내려가기 2번 (같은 자리)
        self.assertEqual(sum(1 for lv, _ in a._report.logs if lv == 'WARN'), 2)

    def test_three_misses_grasp_failed(self):
        a = make(fails=5)
        with self.assertRaises(PartError) as ctx:
            a._measure_width(20.0)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        self.assertEqual(a.sg.calls, 3)

    def test_finger_landed_is_grasp_failed_without_retry(self):
        a = make(fails=0, descent='contact')
        with self.assertRaises(PartError) as ctx:
            a._measure_width(20.0)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        self.assertEqual(a.sg.calls, 0)

    def test_common_sync_defaults(self):
        self.assertAlmostEqual(J.PARAMS['grasp_height'][0], 0.5, places=2)
        self.assertEqual(J.PARAMS['ref_width'][0], 25.0)
        self.assertEqual(J.PARAMS['grip_force'][0], 10.0)


if __name__ == '__main__':
    unittest.main()
