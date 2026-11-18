"""가짜 로봇(DsrIO와 같은 함수)·가짜 시계로 scan_algorithm 흐름 시험. 실제 운동학·힘 제어 검증은 아님.
실행: python3 box_scan/test/test_scan_flow.py   (저장소 루트에서, ROS 없이)"""
import math
import sys
import types
import unittest

_root = __import__('pathlib').Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_root / 'box_comm'), str(_root / 'box_scan')]
from box_comm.common import PartError  # noqa: E402
import box_scan.scan_algorithm as A  # noqa: E402
from box_comm.common import TCP  # noqa: E402
from box_comm.sync_grip import est_close_time, tip_h_mm  # noqa: E402

TCP_Z = TCP['pos'][2]

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
    """원시 폭·잡힘 비트 흉내. sync 집기(grip_until_done)는 닫히는 순간 핑거 끝 높이가 박스 옆면 안이어야 잡힘"""
    pad_total = 10.3

    def __init__(self, robot):
        self.r = robot

    def status(self):
        r = self.r
        if r.held is not None:
            return {'grip': 1, 'width_raw_mm': 50.3}
        if r.stall_active:
            return {'grip': 1, 'width_raw_mm': r.stall_width + 10.3}
        if r.closed_empty:
            return {'grip': 1, 'width_raw_mm': 10.6}
        return {'grip': 0, 'width_raw_mm': r.raw}

    def calibrated(self, raw):
        return raw - 10.3

    def settled_width(self, n_samples=10, interval=0.05):
        return self.status()['width_raw_mm'], 0.0

    def grip_until_done(self, force, timeout=5.0):
        r = self.r
        r.opened = False
        if r.emulator:
            r.held, r.held_grip_z = 'fake', r.p[2]
            return True
        if r.stall_width is not None:
            r.stall_active = True
            return True
        r.c.t += est_close_time(force, r.raw, 50.3)     # 닫히는 동안 sync 상승이 진행됨
        r._cur()
        tip = r.p[2] + TCP_Z - tip_h_mm(50.3)          # 닫혀 박스 폭(40)에 닿는 순간 핑거 끝 높이
        tip = max(tip, 0.1)                            # 바닥 아래로는 못 감 (가짜 모델은 몇 mm 깊게 계산됨)
        for b in r.boxes:
            if b.under(r.p[0], r.p[1], pad=5.0) and 0 < tip < b.h and r.grip_ok:
                r.held, r.held_grip_z = b, r.p[2]
                r.boxes.remove(b)
                r.tips.append(tip)
                return True
        r.closed_empty = True
        return True


class FakeRobot:
    def __init__(self, clock, boxes, grip_ok=True, emulator=False, stall_width=None):
        self.c, self.boxes, self.grip_ok = clock, list(boxes), grip_ok
        self.stall_width = stall_width  # 한쪽 핑거만 걸려 멈춤 흉내: 잡힘 비트 켜지고 이 폭에서 멈추지만 실제로는 안 잡힘
        self.emulator = emulator      # 에뮬레이터 흉내: 힘 센서 0, 그리퍼는 닫으면 항상 잡힘(FakeRG2)
        self.p = HOME_POSE[:]
        self.motion = None        # (start, target, t0, duration)
        self.force_mode = False
        self.compliance = False
        self.opened = False
        self.held = None
        self.placed = []
        self.grip_z_of = {}         # 놓은 박스별 집었을 때 TCP Z
        self.log = []
        self.gripper = FakeGripper(self)
        self.moved = False
        self.raw = 109.5            # 원시 폭 (열림)
        self.stall_active = False   # 한쪽 핑거가 걸려 멈춘 상태 (들어 올리면 풀림)
        self.closed_empty = False
        self.tips = []              # ② sync로 잡힌 순간 핑거 끝 높이

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
        if self.held is None and pose[2] < self.p[2] - 1e-6:   # 내려가는 이동만 검사
            for b in self.boxes:      # 탐색 높이 아래로 박스를 뚫고 가면 시험 실패
                if b.under(pose[0], pose[1]) and pose[2] < b.h - 1 and not self.opened:
                    raise AssertionError(f'박스 관통 이동 {pose[:3]}')
        self.p = list(map(float, pose))
        self.c.t += 0.5
        self.stall_active = False   # 움직이면 걸렸던 핑거가 풀림

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
        target = [self.p[i]+d[i] for i in range(6)]
        if time_s:
            self.motion = (self.p[:], target, self.c.t, float(time_s))
        else:
            self.amovel(target, vel, acc)

    def mwait(self):
        if self.motion:
            self.c.t = self.motion[2] + self.motion[3]
            self._cur()

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
        floor = 0.0 if not self.opened else -(TCP_Z - tip_h_mm(self.raw))   # 열린 핑거 끝이 바닥(0)에서 막힘
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
        # 열린 핑거로 위치 하강: 핑거 끝이 바닥(0)에 닿으면 막히고 Fz 증가
        if self.opened and self.held is None and self.motion:
            floor_tcp = -(TCP_Z - tip_h_mm(self.raw))
            if p[2] <= floor_tcp:
                self.p[2] = floor_tcp
                self.motion = None
                fz = 8.0
        # 내려놓기: 쥔 박스 바닥이 측정존 바닥에 닿음
        if self.held is not None and self.motion and p[2] <= ZONE[2] + self.held_grip_z:
            fz = 8.0
        return [0.0, fy, fz, 0.0, 0.0, 0.0]

    # --- 그리퍼
    def grip_open(self, w=None, f=None):
        if self.held is not None:
            self.placed.append((self.held, self.p[:3]))
            self.grip_z_of[id(self.held)] = self.held_grip_z
            self.held = None
        self.opened = True
        self.stall_active = self.closed_empty = False
        self.raw = min((w or 100.0) + 10.3, 109.5)
        return self.raw

    def grip_close(self, f=None):
        """sync로 잡은 뒤 40N으로 다시 조이기"""
        self.opened = False
        if self.held is not None:
            return types.SimpleNamespace(grabbed=True, width_mm=40.0, raw_mm=50.3)
        if self.stall_active:
            return types.SimpleNamespace(grabbed=True, width_mm=self.stall_width, raw_mm=self.stall_width + 10.3)
        self.closed_empty = True
        return types.SimpleNamespace(grabbed=False, width_mm=0.3, raw_mm=10.6)


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
        A.TABLE_Z = 0.0           # 가짜 작업공간은 바닥 0
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
        for box, at in self.robot.placed:                     # 측정존 위에서 놓았는지 (박스 바닥이 측정존 바닥에 닿는 TCP)
            self.assertAlmostEqual(at[0], ZONE[0])
            self.assertAlmostEqual(at[1], ZONE[1])
            self.assertLessEqual(abs(at[2] - (ZONE[2] + self.robot.grip_z_of[id(box)])), 1.0)

    def test_sync_grip_bites_15mm_below_top(self):
        """sync 집기: 100mm로 열고 핑거 끝이 윗면 − 15에 오게 내려가 sync로 닫음 (적재와 같은 방식)
        (예전 방식이면 40mm 면을 윗면 아래 약 9mm, 60mm 면은 약 3mm만 물었음)"""
        algo = self.make([Box(420, 100, 50)])
        rep = Report()
        self.assertTrue(algo.scan(POSES, rep))
        tip = self.robot.tips[0]
        print(f'\n  핑거 끝(모델) 윗면 아래 {50 - tip:.1f} mm', end='')
        self.assertLess(abs((50 - tip) - A.GRIP_DEPTH_FROM_TOP_MM), 6.0)   # 가짜 모델은 100mm에서 닫으면 몇 mm 깊게 나옴 (적재 시험과 같음)
        self.assertTrue(any('핑거 끝 윗면 − ' in m and '집기 성공' in m for _, m in rep.logs))

    def test_thin_box_grips_just_above_floor(self):
        """아주 얇은 물체(12mm)는 윗면 − 15가 바닥 + 2.6 아래 → 작업대 바닥(TABLE_Z) + 2.6에서 집음 (적재와 같은 기준)"""
        algo = self.make([Box(420, 100, 12)])
        algo.robot.p = [420.0, 100.0, 120.0, 0.0, 180.0, 0.0]
        rep = Report()
        info = algo._grip([420.0, 100.0, 12.0], [0.0, 180.0, 0.0], rep)
        self.assertIsNotNone(info)
        self.assertTrue(any('작업대 바닥 + 2.6' in m for _, m in rep.logs))
        self.assertGreater(self.robot.tips[0], 0.0)

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

    def test_finger_stall_out_of_range_is_not_a_grip(self):
        """한쪽 핑거만 걸려 12mm에서 멈춤(잡힘 비트 켜짐) → 박스 폭 범위 밖이라 못 잡은 것으로 보고 각도 재시도 (11/18)"""
        algo = self.make([Box(420, 100, 50)], stall_width=12.0)
        rep = Report()
        with self.assertRaises(PartError) as ctx:
            algo.scan(POSES, rep)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        fails = [m for lv, m in rep.logs if lv == 'WARN' and '집기 실패' in m]
        self.assertEqual(len(fails), 3)
        self.assertTrue(all('범위' in m for m in fails))
        self.assertEqual(self.robot.placed, [])               # 공기 운반 안 함

    def test_finger_stall_in_range_caught_after_lift(self):
        """범위 안 폭(30mm)에서 멈췄어도 들어 올리면 풀려 잡힘 비트가 꺼짐 → 놓침으로 보고 재시도 (공기 운반 안 함)"""
        algo = self.make([Box(420, 100, 50)], stall_width=30.0)
        rep = Report()
        with self.assertRaises(PartError) as ctx:
            algo.scan(POSES, rep)
        self.assertEqual(ctx.exception.code, 'GRASP_FAILED')
        self.assertTrue(all('놓침' in m for lv, m in rep.logs if lv == 'WARN' and '집기 실패' in m))
        self.assertFalse(any(abs(at[0]-ZONE[0]) < 1 for _, at in self.robot.placed))

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
