"""가짜 로봇(DsrIO와 같은 함수)·가짜 시계로 scan_algorithm 흐름 시험. 실제 운동학·힘 제어 검증은 아님.
실행: python3 box_scan/test/test_scan_flow.py   (저장소 루트에서, ROS 없이)"""
import math
import sys
import types
import unittest

_root = __import__('pathlib').Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_root / 'box_comm'), str(_root / 'box_scan')]
from box_comm.common import GRIPPER, PartError  # noqa: E402
import box_scan.scan_algorithm as A  # noqa: E402

HOME_POSE = [370.0, 0.0, 420.0, 0.0, 180.0, 0.0]
ZONE = [300.0, -160.0, 30.0]


class Clock:
    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        self.t += 0.001          # 호출마다 조금씩 흐름 (무한 루프 방지)
        return self.t

    def sleep(self, s):
        self.t += max(s, 0.0)


class Box:
    def __init__(self, x, y, h, half=20.0):
        self.x, self.y, self.h, self.half = x, y, h, half

    def under(self, x, y, pad=0.0):
        return abs(x-self.x) <= self.half+pad and abs(y-self.y) <= self.half+pad


class FakeGripper:
    def __init__(self, robot):
        self.r = robot

    def status(self):
        return {'grip': int(self.r.held is not None), 'width_raw_mm': 50.0 if self.r.held else 10.3}

    def calibrated(self, raw):
        return raw - 10.3


class FakeRobot:
    def __init__(self, clock, boxes, grip_ok=True, emulator=False):
        self.c, self.boxes, self.grip_ok = clock, list(boxes), grip_ok
        self.emulator = emulator      # 에뮬레이터 흉내: 힘 센서 0, 그리퍼는 닫으면 항상 잡힘(FakeRG2)
        self.p = HOME_POSE[:]
        self.motion = None        # (start, target, t0, duration)
        self.force_mode = False
        self.compliance = False
        self.opened = False
        self.held = None
        self.placed = []
        self.log = []
        self.gripper = FakeGripper(self)
        self.moved = False

    # --- 위치
    def _cur(self):
        if self.motion:
            s, t, t0, dur = self.motion
            k = min(1.0, (self.c.t - t0)/dur) if dur > 0 else 1.0
            self.p = [s[i] + (t[i]-s[i])*k for i in range(6)]
            if k >= 1.0:
                self.motion = None
        if self.force_mode:
            self._force_step()
        return self.p

    def pose(self):
        return self._cur()[:]

    def motion_state(self):
        self._cur()
        return 2 if self.motion else 0

    def _jump(self, pose):
        if self.held is None:
            for b in self.boxes:      # 탐색 높이 아래로 박스를 뚫고 가면 시험 실패
                if b.under(pose[0], pose[1]) and pose[2] < b.h - 1 and not self.opened:
                    raise AssertionError(f'박스 관통 이동 {pose[:3]}')
        self.p = list(map(float, pose))
        self.c.t += 0.5

    def movej(self, j, vel, acc):
        self._jump(HOME_POSE)

    def movel(self, pose, vel, acc, tol=2.0):
        self._jump(pose)

    def movejx(self, pose, vel, acc, tol=2.0):
        self._jump(pose)

    def amovel(self, pose, vel, acc):
        v = vel[0] if isinstance(vel, list) else vel
        dur = math.dist(self.p[:3], pose[:3]) / v
        self.motion = (self.p[:], list(map(float, pose)), self.c.t, dur)

    def amovel_rel(self, d, vel=None, acc=None, time_s=None):
        self.amovel([self.p[i]+d[i] for i in range(6)], vel, acc)

    def move_stop(self, mode=2):
        self._cur()
        self.motion = None
        return True

    def wait(self, s):
        self.c.t += s

    # --- 힘
    def compliance_on(self, stx):
        self.compliance = True

    def force_on(self, fd, d, time_s=0.5):
        self.force_mode = True
        self._fz_extra = 0.0

    def stop(self, emergency=False):
        self.force_mode = self.compliance = False
        self._fz_extra = 0.0

    def _force_step(self):
        # 힘 제어 하강 5 mm/s. 닫힌 핑거는 박스 윗면에서, 열린 핑거는 바닥(0)에서 막힘
        dt = self.c.t - getattr(self, '_last_t', self.c.t)
        self._last_t = self.c.t
        floor = 0.0
        if not self.opened:
            for b in self.boxes:
                if b.under(self.p[0], self.p[1]):
                    floor = max(floor, b.h)
        nz = max(floor, self.p[2] - 5.0*dt)
        self._fz_extra = 10.0 if nz <= floor + 1e-9 and self.p[2] - 5.0*dt < floor else 0.0
        self.p[2] = nz

    def force(self, ref='base'):
        p = self._cur()
        if self.emulator:
            return [0.0] * 6
        fz = getattr(self, '_fz_extra', 0.0) if self.force_mode else 0.0
        fy = 0.0
        # 탐색: 보정표 기준값 + 박스 접촉
        if abs(p[2] - A.SCAN_Z) < 1 and self.motion:
            lane = min(range(len(A.X_LANES)), key=lambda i: abs(A.X_LANES[i]-p[0])) + 1
            d = '-Y' if self.motion[1][1] < self.motion[0][1] else '+Y'
            if abs(self.motion[1][1]-self.motion[0][1]) > 1 and A.BASELINE_N[lane]['direction'] == d:
                fy = A.baseline_fy(p[1], d, lane) or 0.0
            if any(b.under(p[0], p[1], pad=5.0) for b in self.boxes):
                fy += 4.0
        # 내려놓기: 쥔 박스 바닥이 측정존 바닥에 닿음
        if self.held is not None and self.motion and p[2] <= ZONE[2] + self.held_grip_z:
            fz = 8.0
        return [0.0, fy, fz, 0.0, 0.0, 0.0]

    # --- 그리퍼
    def grip_open(self, w=None, f=None):
        if self.held is not None:
            self.placed.append((self.held, self.p[:3]))
            self.held = None
        self.opened = True

    def grip_close(self, f=None):
        self.opened = False
        if self.emulator:
            self.held, self.held_grip_z = 'fake', self.p[2]
            return types.SimpleNamespace(grabbed=True, width_mm=40.0, raw_mm=50.3)
        for b in self.boxes:
            if b.under(self.p[0], self.p[1], pad=5.0) and 0 < self.p[2] < b.h and self.grip_ok:
                self.held = b
                self.held_grip_z = self.p[2]
                self.boxes.remove(b)
                return types.SimpleNamespace(grabbed=True, width_mm=40.0, raw_mm=50.3)
        return types.SimpleNamespace(grabbed=False, width_mm=0.0, raw_mm=10.3)


class Report:
    def __init__(self):
        self.stages, self.logs = [], []

    def __call__(self, s):
        self.stages.append(s)

    def log(self, m, level='INFO'):
        self.logs.append((level, m))


class Node:
    def __init__(self, params=None):
        self.params = params or {}

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warn=lambda *_: None)

    def declare_parameter(self, *a):
        pass

    def get_parameter(self, n):
        return types.SimpleNamespace(value=self.params.get(n, ''))


POSES = {'home_j': [0, 0, 90, 0, 90, 0], 'measure_zone': ZONE}


class Flow(unittest.TestCase):
    def make(self, boxes, params=None, **kw):
        self.clock = Clock()
        A.time = types.SimpleNamespace(monotonic=self.clock.monotonic, sleep=self.clock.sleep)
        self.robot = FakeRobot(self.clock, boxes, **kw)
        return A.ScanAlgorithm(Node(params), self.robot)

    def test_sim_boxes_on_emulator(self):
        """에뮬레이터(힘 0)에서 sim_boxes로 2개 → True, True, False"""
        algo = self.make([], params={'sim_boxes': '420,100,50;510,-20,60'}, emulator=True)
        self.assertTrue(algo.scan(POSES, Report()))
        self.assertTrue(algo.scan(POSES, Report()))
        self.assertFalse(algo.scan(POSES, Report()))
        self.assertEqual(algo.sim_boxes, [])
        at_zone = [at for _, at in self.robot.placed if abs(at[0]-ZONE[0]) < 1 and abs(at[1]-ZONE[1]) < 1]
        self.assertEqual(len(at_zone), 2)       # 측정존에서 2번 놓음 (가짜 그리퍼는 빈손 닫기도 잡힘으로 침)

    def test_emulator_without_sim_gives_false_contact(self):
        """sim_boxes 없이 에뮬레이터에서 돌리면 보정표 때문에 가짜 접촉이 남 (그래서 SIM=1이 필요)"""
        algo = self.make([], emulator=True)
        rep = Report()
        try:
            algo.scan(POSES, rep)
        except Exception:
            pass
        self.assertTrue(any('물품 접촉' in m for _, m in rep.logs))

    def test_two_boxes_one_per_goal_then_empty(self):
        algo = self.make([Box(420, 100, 50), Box(510, -20, 60)])
        rep = Report()
        self.assertTrue(algo.scan(POSES, rep))
        self.assertEqual(rep.stages, ['space_scan', 'picking', 'moving', 'placing'])
        self.assertEqual(len(self.robot.placed), 1)
        first_lane = algo.lane
        self.assertIsNotNone(algo.resume)
        self.assertTrue(algo.scan(POSES, Report()))           # 두 번째: 이어서
        self.assertGreaterEqual(algo.lane, first_lane)
        self.assertFalse(algo.scan(POSES, Report()))          # 다 훑음
        self.assertEqual((algo.lane, algo.resume), (0, None)) # 다음 작업은 처음부터
        self.assertEqual(len(self.robot.placed), 2)
        for box, at in self.robot.placed:                     # 측정존 위에서 놓았는지
            self.assertAlmostEqual(at[0], ZONE[0])
            self.assertAlmostEqual(at[1], ZONE[1])
            self.assertLessEqual(abs(at[2] - (ZONE[2] + A.calculate_grip_z(box.h))), 1.0)

    def test_empty_workspace(self):
        algo = self.make([])
        self.assertFalse(algo.scan(POSES, Report()))

    def test_grasp_failed_keeps_cursor(self):
        algo = self.make([Box(420, 100, 50)], grip_ok=False)
        rep = Report()
        with self.assertRaises(PartError) as ctx:
            algo.scan(POSES, rep)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        self.assertEqual(sum(1 for lv, m in rep.logs if lv == 'WARN' and '집기 실패' in m), 3)
        self.assertIsNotNone(algo.resume)                     # 재스캔은 그 접촉점 -50mm부터
        self.assertTrue(self.robot.opened)

    def test_low_object_blocks_after_3_misses(self):
        algo = self.make([Box(420, 100, 10)])                 # 17mm 미만: 높이 탐지 불가 (윗면-15mm로 못 집음)
        rep = Report()
        with self.assertRaises(PartError) as ctx:
            algo.scan(POSES, rep)
        self.assertEqual(ctx.exception.code, 'SCAN_FAILED')
        self.assertIn('스캔공간 막힘', str(ctx.exception))
        self.assertEqual(sum(1 for _, m in rep.logs if '물품 접촉' in m), A.HEIGHT_MISS_LIMIT)
        self.assertEqual((algo.lane, algo.resume), (0, None))


if __name__ == '__main__':
    unittest.main()
