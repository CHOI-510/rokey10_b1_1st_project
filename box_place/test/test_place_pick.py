"""적재 측정존 sync 집기 시험 (11/18). 가짜 로봇·가짜 RG2(핑거 끝 높이 모델 포함)·가짜 시계, ROS 없이.
실행: python3 box_place/test/test_place_pick.py   (저장소 루트에서)
실제 힘 제어·운동학 검증은 아님. 핑거 끝이 박스 아래쪽을 무는지, 층 판단 보정이 맞는지만 본다.
주의: 가짜 RG2는 일정한 속도로 닫힌다고 가정해서, 100mm에서 닫으면 핑거 끝이 목표(2.6)보다 몇 mm 낮게 나온다
(바닥 아래로 계산되면 바닥에서 멈춘 것으로 침). 판별이 같은 방식으로 실물에서 문제없이 돌았으므로 모델이 보수적인 것으로
보고, 실물에서 긁히면 common.SYNC_GRIP tip_above_floor_mm를 올린다."""
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

_root = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_root / 'box_comm'), str(_root / 'box_place')]
from box_comm.common import TCP, PartError  # noqa: E402
import box_comm.sync_grip as SG  # noqa: E402
import box_place.place_algorithm as P  # noqa: E402

ZONE = [300.0, -160.0, 30.0]
POSES = {'home_j': [0, 0, 90, 0, 90, 0], 'measure_zone': ZONE,
         'place_zones': {'A': [480.0, -160.0], 'B': [650.0, -160.0], 'C': [580.0, 300.0]}}
PAD = 10.3


class Clock:
    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t

    def monotonic(self):
        self.t += 0.001
        return self.t

    def sleep(self, s):
        self.t += max(s, 0.0)


class FakeRG2:
    """원시 폭·잡힘 비트만 흉내. 닫으면 박스 폭에서 멈추되, 그때 핑거 끝이 박스 옆면 높이에 있어야 잡힘."""

    def __init__(self, robot):
        self.r, self.pad_total = robot, PAD
        self.raw, self.grip = 109.5, 0

    def status(self):
        return {'width_raw_mm': self.raw, 'width_mm': self.raw - PAD, 'busy': 0, 'grip': self.grip}

    def open(self, force, width):
        self.raw, self.grip = min(width + PAD, 109.5), 0
        self.r.held = None
        return self.raw

    def grip_until_done(self, force, timeout=5.0):
        box = self.r.box
        if self.r.held is not None:                      # 이미 쥔 채 다시 조임
            self.grip = 1
            return True
        target = box['raw'] if box else PAD
        dt = SG.est_close_time(force, self.raw, target)
        self.r.clock.t += dt                            # 닫는 동안 비동기 상승이 진행됨
        tip = self.r.z() + TCP['pos'][2] - SG.tip_h_mm(target)   # 닿는 순간 핑거 끝 높이 (모델)
        self.r.contact_tips.append(tip)
        tip = max(tip, ZONE[2])                          # 바닥 아래로는 못 감
        if box and ZONE[2] - 0.5 <= tip <= ZONE[2] + box['h'] - 2.0:
            self.raw, self.grip = box['raw'], 1
            self.r.held = box
        else:
            self.raw, self.grip = PAD + 0.3, 1          # 빈손: 패드끼리 닿음 (잡힘 비트도 켜짐, 실물과 같음)
        return True

    def settled_width(self, n_samples=10, interval=0.05):
        return self.raw, 0.0

    def calibrated(self, raw):
        return max(0.0, raw - 0.30 - PAD)


class FakeRobot:
    def __init__(self, clock, box):
        self.clock, self.box = clock, box
        self.p = [370.0, 0.0, 420.0, 0.0, 180.0, 0.0]
        self.motion = None
        self.held = None
        self.contact_tips = []
        self.opened_widths, self.close_forces = [], []
        self.gripper = FakeRG2(self)

    def z(self):
        if self.motion:
            z0, z1, t0, dur = self.motion
            k = min(1.0, (self.clock.t - t0) / dur) if dur > 0 else 1.0
            self.p[2] = z0 + (z1 - z0) * k
            if k >= 1.0:
                self.motion = None
        return self.p[2]

    def pose(self):
        self.z()
        return self.p[:]

    def movel(self, pose, vel=None, acc=None):
        self.motion = None
        self.p = list(map(float, pose))

    def movej(self, j, vel=None, acc=None):
        self.motion = None

    def amovel_rel(self, d, vel=None, acc=None, time_s=None):
        z = self.z()
        self.motion = (z, z + d[2], self.clock.t, time_s or 1.0)

    def move_stop(self, mode=2):
        self.z()
        self.motion = None
        return True

    def wait(self, seconds):                      # robot.wait (집은 뒤 힘이 다 걸리기를 기다림, 11/18)
        self.clock.t += max(float(seconds), 0.0)
        self.z()

    def mwait(self):
        if self.motion:
            self.clock.t = self.motion[2] + self.motion[3]
            self.z()

    def grip_open(self, width_mm=None, force_n=None):
        self.opened_widths.append(width_mm or 100.0)
        return self.gripper.open(force_n or 20.0, width_mm or 100.0)

    def grip_close(self, force_n=None):
        self.close_forces.append(force_n)
        self.gripper.grip_until_done(force_n)
        w = self.gripper.calibrated(self.gripper.raw)
        return types.SimpleNamespace(grabbed=bool(self.gripper.grip) and w > 5.0, width_mm=w, raw_mm=self.gripper.raw)

    def force_z(self):
        return 0.0


class Node:
    def __init__(self, params):
        self.params = params

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)

    def declare_parameter(self, n, v):
        self.params.setdefault(n, v)

    def get_parameter(self, n):
        return types.SimpleNamespace(value=self.params[n])


class Report:
    def __init__(self):
        self.logs = []

    def __call__(self, s):
        pass

    def log(self, m, level='INFO'):
        self.logs.append((level, m))


class SyncPick(unittest.TestCase):
    def make(self, box, config_edit=None):
        self.clock = Clock()
        fake_time = types.SimpleNamespace(time=self.clock.time, monotonic=self.clock.monotonic, sleep=self.clock.sleep)
        P.time = SG.time = fake_time
        self.robot = FakeRobot(self.clock, box)
        tmp = Path(tempfile.mkdtemp())
        cfg = json.loads((_root / 'box_place' / 'config' / 'place_config.json').read_text())
        if config_edit:
            config_edit(cfg)
        (tmp / 'c.json').write_text(json.dumps(cfg))
        params = {'place_config': str(tmp / 'c.json'), 'place_state': str(tmp / 's.json'), 'sim_contact': True}
        return P.PlaceAlgorithm(Node(params), self.robot)

    def check_box(self, cls, h, side):
        algo = self.make({'h': h, 'raw': side + PAD + 0.3})
        rep = Report()
        count = algo.place(cls, POSES, rep)
        self.assertEqual(count, 1)
        tip = self.robot.contact_tips[0]
        self.assertLess(tip - ZONE[2], h / 2, f'{cls} 핑거 끝이 박스 아래쪽이 아님: 바닥 위 {tip - ZONE[2]:.1f}mm')
        self.assertGreater(tip - ZONE[2], -7.0)          # 모델 오차 범위 (위 주의 참고)
        msg = [m for _, m in rep.logs if '적재 →' in m][0]
        self.assertIn('1층', msg)
        return tip - ZONE[2], algo._hold_off

    def test_all_sides_grip_low(self):
        """A·B·C 어느 면으로 잡아도 핑거 끝이 바닥 위 0 ~ 박스 높이 절반 (11/18 실물: A 60mm 면을 위 3mm만 물었음)"""
        for cls, h, side in (('A', 20.0, 60.0), ('A', 20.0, 40.0), ('B', 25.0, 75.0), ('B', 25.0, 44.0),
                             ('C', 20.0, 50.0), ('C', 20.0, 40.0)):
            with self.subTest(cls=cls, side=side):
                tip, off = self.check_box(cls, h, side)
                print(f'\n  {cls} {side:.0f}mm 면: 핑거 끝(모델) 바닥 위 {tip:+.1f} mm, TCP−바닥 {off:+.1f} mm', end='')

    def test_old_fixed_z_would_bite_top(self):  # noqa
        """참고: 예전 방식(TCP 바닥+5에서 그냥 닫기)이면 A 60mm 면에서 핑거 끝이 바닥 위 약 17mm"""
        tip = ZONE[2] + 5.0 + TCP['pos'][2] - SG.tip_h_mm(60.0 + PAD + 0.3) - ZONE[2]
        self.assertGreater(tip, 15.0)

    def test_empty_zone_grasp_failed(self):
        algo = self.make(None)
        rep = Report()
        with self.assertRaises(PartError) as ctx:
            algo.place('A', POSES, rep)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        self.assertEqual(sum(1 for lv, m in rep.logs if lv == 'WARN' and '집기 실패' in m), 3)
        self.assertIsNone(algo.state['pending'])

    def test_old_config_keys_rejected(self):
        for key in ('pick_above_floor_mm', 'pick_tip_above_floor_mm', 'sync_grip_force_n'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.make(None, config_edit=lambda c: c.update({key: 5.0}))

    def test_uses_common_open_100_and_tightens_40(self):
        algo = self.make({'h': 20.0, 'raw': 70.6})
        rep = Report()
        algo.pick_from_measure_zone(ZONE, rep)
        self.assertGreater(self.robot.opened_widths[0], 95.0)
        self.assertEqual(self.robot.close_forces, [40.0])


if __name__ == '__main__':
    unittest.main()
