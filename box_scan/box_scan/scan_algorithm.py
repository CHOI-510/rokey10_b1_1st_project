#!/usr/bin/env python3
"""
scan_algorithm.py — 탐지 알고리즘 (담당: 김창균)
================================================================
원본 m0609_scan_final.py(v17, 실로봇 확인)의 탐색·높이 탐지·집기 로직을 팀 약속에 맞춘 통합본 (11/17).
관제 관련 코드는 없다. 로봇은 생성자로 받은 robot(box_comm.dsr_io.DsrIO)으로만 움직인다
→ 명령 전 관제 정지 확인·진행 알림이 자동. 실패하면 예외를 던지면 box_comm이 로봇을 멈추고 SCAN_FAILED로 보낸다.

약속 (box_comm/scan_interface.py start_algorithm)
  ScanAlgorithm(node, robot)
  scan(poses, report) -> bool
    True  : 물품 1개를 측정존에 놓고 HOME까지 옴
    False : 작업공간 전체를 훑었는데 물품 없음 (HOME 복귀)
    PartError('GRASP_FAILED') : 높이까지 찾았는데 3번 다 못 집음 → 관제가 공간 스캔부터 다시

Goal 한 번 = 물품 1개 (원본은 한 번 실행에 전체 영역을 돌며 여러 개를 놓았음)
  노드는 작업 내내 켜져 있고 이 객체도 하나다 → 어디까지 훑었는지(탐색 커서)를 self에 들고 있다가
  다음 Goal에서 마지막 정지점에서 진행 방향 뒤쪽으로 이동해 같은 선을 이어서 훑는다.
  커서는 False를 반환할 때(다 훑음)와 예상 못 한 오류 때 처음으로 돌아간다. 노드를 다시 켜도 처음부터.

한 Goal의 흐름
  space_scan : HOME → 그리퍼 닫기(핑거가 탐침) → 탐색 시작점(또는 재개점) → 지그재그 Y 탐색
               Y 양 끝 40mm를 제외한 구간에서 경로별 Fy 보정표와 비교해 |Fy - 기준| >= 2.0 N이면 빠른 정지 (접촉)
  picking    : 접촉점 앞쪽 부채꼴 최대 4점을 차례로 Force 하강해 윗면 높이 탐지
               (쓰러진 물품 대비) → 처음 찾은 점에서 윗면 - 25mm 높이를 0 / 45 / 90도로 최대 3번 집기
               4점 모두 높이 없음 → 진행 방향 뒤쪽에서 재개해 탐색 계속. 같은 자리에서 3번이면 '스캔공간 막힘'(_on_scan_blocked)
               3번 다 실패 → GRASP_FAILED (커서는 남겨 둠: 다음 Goal에서 그 정지점의 진행 방향 뒤쪽부터)
  moving     : 들어 올려 측정존 위로 (관절 보간)
  placing    : 천천히 내려가다 바닥에 닿으면 멈추고 놓음 (판별 _pos_descent 방식) → 들어 올림 → HOME

원본에서 바뀐 것
  - 로봇 연결·TCP 등록·모드 전환·인자·JSON 기록 삭제 (box_comm 담당). 기록은 report.log → 이벤트 로그
  - 그리퍼 DO/DI → Modbus. 잡힘은 닫은 직후 grabbed + 들어 올린 뒤 한 번 더 확인
    (원본의 '이송 중 50ms마다 DI 확인'은 없음 → 측정존 내려놓기 직전에 한 번 확인)
  - 원본은 한 점에서 3번 실패하면 다음 높이 점에서 다시 3번 시도했음 → 첫 점 기준 3번으로 줄임
  - 높이 탐지와 집기 접근의 Force 하강을 함수 하나(_force_down)로 합침
  - 높이 없음 → 진행 방향 뒤쪽에서 재개가 같은 자리에서 3번 반복되면 멈춤 (원본은 40mm 미만 물체에서 끝없이 반복)
    멈춘 뒤 처리는 팀 결정 전이라 임시로 SCAN_FAILED (_on_scan_blocked 한 곳만 바꾸면 됨)
  - 배치 높이: PLACE_Z_OFFSET(+35) 대신 측정존 z(바닥) 기준 접촉 하강
  - 집는 높이: 윗면 높이의 50% → 윗면 - 25mm. 높이 탐지 하한은 BASE TCP Z 40mm
  - CSV 기록은 파라미터 csv_dir을 줄 때만 (보정표를 다시 만들 때)

파라미터
  csv_dir   : 빈 값이면 CSV 안 남김 (기본). 예) -p csv_dir:=~/ROkEY_협동1/04_실험데이터
  sim_boxes : 에뮬레이터 전용 가상 박스 "x,y,높이;x,y,높이" (기본 빈 값 = 실제 로봇).
              에뮬레이터는 힘 센서가 없어서 그대로 돌리면 보정표 때문에 바로 가짜 접촉이 난다.
              주면: 탐색 Fy = 보정표 값 (+ 가상 박스 근처면 접촉 힘), 높이 탐지·집기 접근·내려놓기는
              힘 제어 대신 가상 높이까지 movel (적재 sim_contact와 같은 방식). 집은 박스는 목록에서 빠진다.
              그리퍼는 BOX_GRIPPER=fake와 같이 쓴다. 실제 로봇에서 켜지 말 것.
              예) -p sim_boxes:="420,100,50;510,-20,60"
================================================================
"""
import csv
import math
import time
from bisect import bisect_right
from datetime import datetime
from pathlib import Path

from box_comm.common import GRIP_RETRY, GRIPPER, PartError
from box_comm.gripper import check_grip, holding

from box_scan.scan_config import *  # noqa: F401,F403  (설정값 이름을 원본 그대로 씀)


# ----------------------------------------------------------------------
# 계산 (원본 그대로)
# ----------------------------------------------------------------------
def zyz_rotation(angles):
    a, b, c = map(math.radians, angles)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb],
            [sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb],
            [-sb*cc, sb*sc, cb]]


def rotation_error(a, b):
    ra, rb = zyz_rotation(a), zyz_rotation(b)
    return max(abs(ra[i][j]-rb[i][j]) for i in range(3) for j in range(3))


def lane_direction(index):
    """index 0부터: 짝수 선은 -Y, 홀수 선은 +Y"""
    return '-Y' if index % 2 == 0 else '+Y'


def validate_config():
    if set(BASELINE_N) != set(range(1, len(X_LANES)+1)):
        raise ValueError('보정표가 모든 선을 덮지 않음')
    for lane, x in enumerate(X_LANES, 1):
        entry = BASELINE_N[lane]
        if entry['x_mm'] != x or entry['direction'] != lane_direction(lane-1):
            raise ValueError(f'{lane}번 선 보정표 X/방향 불일치')
        values = entry['values']
        if len(values) != len(Y_MM) or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError(f'{lane}번 선 보정값 {len(Y_MM)}개 필요')
    if not 0 < GRIP_DEPTH_FROM_TOP_MM < MAX_OBJECT_HEIGHT_MM:
        raise ValueError('0 < GRIP_DEPTH_FROM_TOP_MM < MAX_OBJECT_HEIGHT_MM')
    if not 0 < GRIP_APPROACH_CONTACT_N < GRIP_APPROACH_FORCE_TARGET_N < GRIP_APPROACH_SAFETY_N:
        raise ValueError('집기 접근: 0 < 접촉 < 목표 < 안전 힘')
    if not 0 < PROBE_CONTACT_N < PROBE_FORCE_TARGET_N < PROBE_SAFETY_N:
        raise ValueError('높이 탐지: 0 < 접촉 < 목표 < 안전 힘')
    if PROBE_START_Z < Z0 + MAX_OBJECT_HEIGHT_MM + PROBE_CLEARANCE_MM:
        raise ValueError('PROBE_START_Z가 최대 물품 높이 + 여유보다 낮음')
    if PROBE_START_Z <= SCAN_Z:
        raise ValueError('PROBE_START_Z는 SCAN_Z보다 높아야 함')
    if not PROBE_POINTS or any(r <= 0 or abs(a) >= 90 for r, a in PROBE_POINTS):
        raise ValueError('높이 탐지점: 반지름 > 0, -90 < 각도 < 90')
    if not GRIP_RETRY_ANGLES:
        raise ValueError('집기 각도가 비어 있음')


def baseline_fy(y_mm, direction, lane):
    entry = BASELINE_N.get(lane)
    if entry is None or entry['direction'] != direction:
        raise ValueError(f'보정표 없음: lane={lane}, direction={direction}')
    y = float(y_mm)
    if not Y_MM[0] <= y <= Y_MM[-1]:
        return None                        # 보정 범위 밖은 판정 안 함
    values = entry['values']
    i = min(bisect_right(Y_MM, y)-1, len(Y_MM)-2)
    fraction = (y-Y_MM[i])/(Y_MM[i+1]-Y_MM[i])
    return values[i] + fraction*(values[i+1]-values[i])


def backward_orientation(reference, direction):
    """진행 방향 반대로 BACKWARD_TILT_DEG만큼 기울인 ZYZ 자세"""
    if BACKWARD_TILT_DEG == 0:
        return list(reference)
    r = zyz_rotation(reference)
    if r[2][2] > -0.98:
        raise RuntimeError('기울기 계산에는 툴 Z가 거의 아래를 향한 HOME 자세가 필요')
    angle = math.radians(BACKWARD_TILT_DEG if direction == '-Y' else -BACKWARD_TILT_DEG)
    c, t = math.cos(angle), math.sin(angle)
    rx = [[1, 0, 0], [0, c, -t], [0, t, c]]
    q = [[sum(rx[i][k]*r[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    b = math.acos(max(-1.0, min(1.0, q[2][2])))
    if abs(math.sin(b)) < 1e-8:
        a = math.atan2(-q[0][1], -q[0][0]) if q[2][2] < 0 else math.atan2(q[1][0], q[0][0])
        return [math.degrees(a), math.degrees(b), 0.0]
    return [math.degrees(math.atan2(q[1][2], q[0][2])), math.degrees(b),
            math.degrees(math.atan2(q[2][1], -q[2][0]))]


def sector_probe_points(contact):
    """접촉점에서 진행 방향 앞쪽 부채꼴 높이 탐지점 [x, y, PROBE_START_Z]"""
    side = contact['stopped_pose']
    sign = 1.0 if contact['direction'] == '+Y' else -1.0
    return [[side[0] + r*math.sin(math.radians(a)), side[1] + sign*r*math.cos(math.radians(a)), PROBE_START_Z]
            for r, a in PROBE_POINTS]


def probe_floor_z():
    """Force 높이 탐지의 하강 하한 (BASE TCP Z, mm)."""
    return PROBE_MIN_Z


def calculate_grip_z(top_z):
    """집는 TCP 높이 = 윗면 - GRIP_DEPTH_FROM_TOP_MM (11/18)"""
    z = top_z - GRIP_DEPTH_FROM_TOP_MM
    if z < Z0 + MIN_GRIP_ABOVE_Z0_MM:
        raise RuntimeError('계산한 집기 높이가 너무 낮음')
    return z


def parse_sim_boxes(text):
    """'x,y,h;x,y,h' → [[x, y, h], ...]. 빈 값이면 None (실제 로봇)"""
    text = str(text or '').strip()
    if not text:
        return None
    boxes = []
    for part in text.split(';'):
        if part.strip():
            v = [float(t) for t in part.split(',')]
            if len(v) != 3 or v[2] <= 0:
                raise ValueError(f'sim_boxes 형식 오류: "{part}" (x,y,높이)')
            boxes.append(v)
    return boxes


SIM_HALF_MM = 20.0          # 가상 박스 반 폭
SIM_TOUCH_MM = 25.0         # 탐색 중 이 거리 안이면 접촉 (핑거 두께 포함)
SIM_CONTACT_FY = 4.0        # 가상 접촉 힘 [N]


class ProbeMiss(Exception):
    """정상적으로 내려갔지만 유효한 윗면 높이가 없음"""


class Recorder:
    """CSV 기록 (원본 그대로, 보정표 재측정용). csv_dir 파라미터를 줄 때만 만든다."""
    FIELDS = ('elapsed_s', 'segment_elapsed_s', 'actual_interval_s', 'lane', 'phase', 'direction',
              'x_mm', 'y_mm', 'z_mm', 'fy_raw_n', 'fy_baseline_n', 'fy_corrected_n', 'motion_state',
              'detection_enabled', 'contact', 'speed_mm_s', 'acc_mm_s2', 'baseline_loaded')

    def __init__(self, root):
        self.directory = Path(root).expanduser() / (
            f'raster_v{SCAN_VEL:g}_a{SCAN_ACC:g}_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.rows, self.index = [], 0
        self.origin = self.segment_origin = self.previous = None

    def add(self, stamp, lane, phase, direction, pose, fy, baseline, residual, status, enabled, contact):
        if self.origin is None:
            self.origin = stamp
        if self.segment_origin is None:
            self.segment_origin = stamp
        interval = '' if self.previous is None else stamp-self.previous
        self.rows.append((stamp-self.origin, stamp-self.segment_origin, interval, lane, phase, direction,
                          *pose[:3], fy, '' if baseline is None else baseline,
                          '' if residual is None else residual, status, int(enabled), int(contact),
                          SCAN_VEL, SCAN_ACC, 1))
        self.previous = stamp

    def flush(self):
        if not self.rows:
            return
        target = self.directory / f'{self.index+1:02d}_lane_{self.rows[0][3]:02d}_{self.rows[0][4]}.csv'
        temporary = target.with_suffix('.csv.partial')
        with temporary.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(self.FIELDS)
            writer.writerows(self.rows)
        temporary.rename(target)
        self.index += 1
        self.rows.clear()
        self.segment_origin = self.previous = None


# ----------------------------------------------------------------------
# 알고리즘
# ----------------------------------------------------------------------
class ScanAlgorithm:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot
        self._grip_width = None             # 닫았을 때 폭 (들어 올린 뒤·이송 중 놓침 판정 기준)
        self._hold_why = ''
        validate_config()
        node.declare_parameter('csv_dir', '')
        node.declare_parameter('sim_boxes', '')
        self.sim_boxes = parse_sim_boxes(node.get_parameter('sim_boxes').value)
        if self.sim_boxes is not None:
            self.log.warn(f'[scan] sim_boxes 켜짐 — 힘 센서 대신 가상 박스 {self.sim_boxes}. '
                          f'에뮬레이터 전용, 실제 로봇에서 쓰지 말 것')
        csv_dir = str(node.get_parameter('csv_dir').value or '').strip()
        self.recorder = Recorder(csv_dir) if csv_dir else None
        if self.recorder:
            self.log.info(f'[scan] CSV 기록: {self.recorder.directory}')
        self._reset_cursor()
        self.log.info(f'[scan] 탐색 X={X_LANES[0]}..{X_LANES[-1]} ({len(X_LANES)}선), Y={Y_START}..{Y_END}, '
                      f'Z={SCAN_Z}, 접촉 |dFy|>={FORCE_LIMIT} N, 집기 각도 {GRIP_RETRY_ANGLES}')

    # ---- 탐색 커서 (Goal 사이에 유지) ----
    def _reset_cursor(self):
        self.lane = 0            # 지금 훑는 선 index
        self.resume = None       # 다음 Goal에서 이어 훑을 접촉 정보 (None이면 선 시작점부터)

    # ==================================================================
    # 약속된 함수
    # ==================================================================
    def scan(self, poses, report) -> bool:
        if self.robot is None:
            raise RuntimeError('로봇 연결 없음 (두산 패키지 확인)')
        home, zone = self._goal_poses(poses)
        try:
            return self._scan(home, zone, report)
        except PartError as e:
            if e.code != 'GRASP_FAILED':       # 정지(CANCELED) 등: 다음 작업은 처음부터
                self._reset_cursor()
            raise
        except BaseException:
            self._reset_cursor()
            raise
        finally:
            if self.recorder:
                self.recorder.flush()

    def _scan(self, home, zone, report):
        r = self.robot
        report('space_scan')
        self._last_miss = None                           # 높이 없음 반복 감지 (Goal 안에서만)
        r.movej(home, vel=HOME_VEL, acc=HOME_ACC)
        here = r.pose()
        reference, home_z = here[3:6], here[2]           # HOME 자세의 ABC가 기준 자세
        orient = {d: backward_orientation(reference, d) for d in ('-Y', '+Y')}
        self._close()                                    # 탐색·높이 탐지는 닫힌 핑거로

        if self.resume is not None:
            self._go_resume(self.resume, orient)
            report.log(f'{self.lane+1}번 선 이어서 탐색 (재개 Y={r.pose()[1]:.1f}mm)')
            ignore = RESUME_IGNORE_MM
        else:
            x = X_LANES[self.lane]
            d = lane_direction(self.lane)
            start_y = Y_START if d == '-Y' else Y_END
            r.movejx([x, start_y, SCAN_Z, *orient[d]], vel=APPROACH_VEL, acc=APPROACH_ACC)
            r.wait(SETTLE_SECONDS)
            ignore = 0.0

        while self.lane < len(X_LANES):
            lane = self.lane + 1
            x = X_LANES[self.lane]
            d = lane_direction(self.lane)
            end_y = Y_END if d == '-Y' else Y_START
            o = orient[d]
            self._orient(o, TILT_VEL, TILT_ACC)
            r.wait(BETWEEN_LINES_WAIT)
            while abs(r.pose()[1] - end_y) > 1.0:            # 재개점이 선 끝이면 바로 다음 선
                contact = self._scan_line([x, end_y, SCAN_Z, *o], d, lane, ignore)
                if contact is None:
                    break
                sp = contact['stopped_pose']
                report.log(f'물품 접촉: {lane}번 선 {d}, x={sp[0]:.0f}, y={sp[1]:.0f}, '
                           f'dFy={contact["residual_n"]:+.2f} N')
                self.resume = contact                       # 이 뒤로 무슨 일이 있어도 이 정지점 기준으로 재개
                report('picking')
                grip = self._find_and_grip(contact, reference, report)
                if grip is not None:
                    self._carry_and_place(grip, zone, home_z, report)
                    r.movej(home, vel=HOME_VEL, acc=HOME_ACC)
                    report.log('물품 1개 측정존 배치 완료, HOME 복귀')
                    return True
                # 원본은 여기서 진행 방향 뒤쪽에서 재개 → 낮은 물체(40mm 미만)면 같은 물체에 다시 닿아 끝없이 반복함
                if (self._last_miss and self._last_miss[0] == lane
                        and abs(self._last_miss[1] - sp[1]) < HEIGHT_MISS_SAME_SPOT_MM):
                    misses = self._last_miss[2] + 1
                else:
                    misses = 1
                self._last_miss = (lane, sp[1], misses)
                if misses >= HEIGHT_MISS_LIMIT:
                    self._on_scan_blocked(contact, misses, report)
                report.log(f'높이 탐지 4점 모두 유효 높이 없음 ({misses}/{HEIGHT_MISS_LIMIT}) → 탐색 계속', 'WARN')
                report('space_scan')
                self._go_resume(contact, orient)
                ignore = RESUME_IGNORE_MM
            self.lane += 1
            self.resume = None
            ignore = 0.0
            if self.lane < len(X_LANES):                    # 다음 선으로 +X 이동 (판정 없이 기록만)
                r.wait(BETWEEN_LINES_WAIT)
                self._scan_line([X_LANES[self.lane], end_y, SCAN_Z, *o], '+X', lane, phase='transfer')
                r.wait(BETWEEN_LINES_WAIT)

        r.movej(home, vel=HOME_VEL, acc=HOME_ACC)
        self._reset_cursor()
        report.log('작업공간 전체 탐색 완료: 물품 없음')
        return False

    def _on_scan_blocked(self, contact, misses, report):
        """같은 자리에서 높이 탐지가 HEIGHT_MISS_LIMIT번 실패 = 스캔공간 막힘 (낮거나 집을 수 없는 물체 추정).
        TODO(팀 결정): 지금은 임시로 높은 곳으로 올라가 멈춤(SCAN_FAILED → 관제 ERROR, 사람이 치움).
          후보: 그 물체를 건너뛰고 다음 선부터 계속 / 관제에 별도 코드(예: SCAN_BLOCKED)로 알림"""
        sp = contact['stopped_pose']
        msg = (f'스캔공간 막힘: {contact["lane"]}번 선 x={sp[0]:.0f}, y={sp[1]:.0f}에서 '
               f'높이 탐지 {misses}회 실패 (낮거나 집을 수 없는 물체) → 사람이 확인')
        report.log(msg, 'ERROR')
        self._move_z(max(PROBE_START_Z, self.robot.pose()[2]))   # 물체 위에서 벗어나 멈춤
        raise PartError('SCAN_FAILED', msg)

    # ==================================================================
    # 이동 단위
    # ==================================================================
    def _move(self, pose):
        self.robot.movel(pose, vel=[LINEAR_VEL, 20.0], acc=[LINEAR_ACC, 40.0], tol=1.0)

    def _move_z(self, z):
        p = self.robot.pose()
        if abs(p[2] - z) > 0.1:
            p[2] = float(z)
            self._move(p)

    def _orient(self, abc, vel, acc):
        """제자리에서 자세만 바꾸기 (원본 set_scan_orientation)"""
        p = self.robot.pose()
        if rotation_error(p[3:6], abc) <= 1e-6:
            return
        self.robot.movel([*p[:3], *abc], vel=[5.0, vel], acc=[10.0, acc], tol=1.0)
        if rotation_error(self.robot.pose()[3:6], abc) > 0.005:
            raise RuntimeError('자세 회전 완료 확인 실패')

    def _wait_still(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while self.robot.motion_state() != 0:
            if time.monotonic() >= deadline:
                raise TimeoutError('정지 확인 시간 초과')
            time.sleep(POLL)

    def _close(self):
        self.robot.grip_close(GRIP_FORCE_N or GRIPPER['grip_force_n'])

    def _open(self):
        self.robot.grip_open(OPEN_WIDTH_MM, None)
        self._grip_width = None

    def _holding(self) -> bool:
        """쥐고 있는지: 잡힘 비트 + 폭이 박스 범위 안 + 닫았을 때 폭에서 안 변함 (box_comm.gripper.holding, 11/18)"""
        ok, self._hold_why, _w = holding(self.robot.gripper, getattr(self, '_grip_width', None))
        return ok

    def _go_resume(self, contact, orient):
        """정지점에서 진행 방향 뒤로 이동할 재개점을 구하고 탐색 높이로 하강"""
        lane, d = contact['lane'], contact['direction']
        distance = abs(SCAN_RESUME_Y_OFFSET_MM)
        offset = distance if d == '-Y' else -distance
        y = contact['stopped_pose'][1] + offset
        y = max(min(Y_START, Y_END), min(max(Y_START, Y_END), y))
        x = X_LANES[lane-1]
        self._move_z(max(PROBE_START_Z, self.robot.pose()[2]))
        self.robot.movejx([x, y, PROBE_START_Z, *orient[d]], vel=TRANSFER_JOINT_VEL, acc=TRANSFER_JOINT_ACC)
        self._move_z(SCAN_Z)
        self.robot.wait(SETTLE_SECONDS)

    # ==================================================================
    # 탐색 한 줄 (원본 Robot.scan_line)
    # ==================================================================
    def _scan_line(self, target, direction, lane, ignore_mm=0.0, phase='scan'):
        """target까지 비동기 직선 이동하며 50ms마다 Fy를 보정표와 비교. 접촉하면 빠른 정지 후 접촉 정보, 끝까지 가면 None"""
        r = self.robot
        start_y = r.pose()[1]
        sign = 1.0 if direction == '+Y' else -1.0
        r.amovel(target, vel=[SCAN_VEL, 10.0], acc=[SCAN_ACC, 20.0])
        started = next_sample = time.monotonic()
        try:
            while True:
                status = r.motion_state()
                now = time.monotonic()
                if now >= next_sample:
                    fy = r.force('base')[1]                  # 정지 확인 + 1초마다 진행 알림 포함
                    pose = r.pose()
                    offset = baseline_fy(pose[1], direction, lane) if phase == 'scan' else None
                    if self.sim_boxes is not None:
                        fy = self._sim_fy(pose, offset)
                    residual = None if offset is None else fy - offset
                    enabled = status != 0 and residual is not None and sign*(pose[1]-start_y) >= ignore_mm
                    contact = enabled and abs(residual) >= FORCE_LIMIT
                    if self.recorder:
                        self.recorder.add(now, lane, phase, direction, pose, fy, offset, residual,
                                          status, enabled, contact)
                    if contact:
                        if not r.move_stop(1):
                            raise RuntimeError('접촉 후 빠른 정지 요청 실패')
                        self._wait_still()
                        return {'lane': lane, 'direction': direction, 'sample_pose': pose,
                                'stopped_pose': r.pose(), 'fy_n': fy, 'baseline_n': offset,
                                'residual_n': residual}
                    next_sample = now + SAMPLE_PERIOD
                if status == 0:
                    actual = r.pose()
                    if math.dist(actual[:3], target[:3]) <= 1.0:
                        return None
                    if now - started > 2.0:              # 시작 직후 idle은 무시
                        raise RuntimeError(f'탐색 이동이 중간에 멈춤: {actual[:3]} → {target[:3]}')
                if now - started >= 90.0:
                    raise TimeoutError('탐색 이동 시간 초과')
                time.sleep(POLL)
        finally:
            if self.recorder:
                self.recorder.flush()

    # ==================================================================
    # Force 하강 (원본 probe_contact + grip_force_approach 공통 부분)
    # ==================================================================
    def _force_down(self, stop_z, target_n, contact_n, safety_n, check_xyz):
        """순응·힘 제어로 내려가다 (1) Fz가 기준보다 contact_n 이상 늘면 'contact', (2) stop_z에 닿으면 'reached'.
        어느 쪽이든 힘·순응 제어를 풀고 멈춘 뒤 (결과, 정지 후 pose)를 돌려준다.
        check_xyz: True면 기준 안정성·안전 힘을 XYZ 합성으로 (집기 접근), False면 Fz만 (높이 탐지)"""
        r = self.robot
        time.sleep(0.3)
        samples = []
        for _ in range(PROBE_BASELINE_SAMPLES):
            samples.append(r.force('base')[:3])
            time.sleep(0.03)
        axes = range(3) if check_xyz else (2,)
        if any(max(s[i] for s in samples) - min(s[i] for s in samples) > PROBE_NOISE_MAX_N for i in axes):
            raise RuntimeError('정지 상태 외력이 불안정해 Force 하강 취소')
        base = [sum(s[i] for s in samples)/len(samples) for i in range(3)]
        initial = r.pose()
        if initial[2] <= stop_z:
            raise RuntimeError(f'Force 하강 시작 높이({initial[2]:.1f})가 목표({stop_z:.1f})보다 낮음')
        deadline = time.monotonic() + (initial[2] - stop_z)/PROBE_TIMEOUT_REFERENCE_MM_S + 10.0
        result = None
        try:
            r.compliance_on(PROBE_STIFFNESS[:])
            time.sleep(PROBE_MODE_WAIT_SECONDS)
            r.force_on([0.0, 0.0, -target_n, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], PROBE_FORCE_RAMP_SECONDS)
            while time.monotonic() < deadline:
                f = r.force('base')[:3]
                dfz = f[2] - base[2]
                change = math.dist(f, base) if check_xyz else abs(dfz)
                if change >= safety_n:
                    raise RuntimeError(f'Force 하강 안전 한계: {change:.1f} N')
                if dfz >= contact_n:
                    result = 'contact'
                    break
                p = r.pose()
                if math.dist(p[:2], initial[:2]) > PROBE_XY_DRIFT_LIMIT_MM:
                    raise RuntimeError('Force 하강 중 XY 밀림 과다')
                if p[2] <= stop_z:
                    result = 'reached'
                    break
                time.sleep(0.02)
            else:
                raise TimeoutError('Force 하강 시간 초과')
        finally:
            r.stop()                                   # 힘·순응 제어 해제 + 멈춤 확인 (실패·취소 때도)
        return result, r.pose()

    def _probe_height(self):
        """지금 위치에서 내려가 윗면 높이 탐지 → 윗면 접촉 pose. 유효 높이가 없으면 ProbeMiss"""
        floor_z = probe_floor_z()
        if self.sim_boxes is not None:
            return self._sim_probe(floor_z)
        result, pose = self._force_down(floor_z, PROBE_FORCE_TARGET_N, PROBE_CONTACT_N, PROBE_SAFETY_N,
                                        check_xyz=False)
        if result != 'contact':
            raise ProbeMiss('하한까지 내려갔지만 윗면 없음')
        if pose[2] < floor_z:
            raise ProbeMiss(f'접촉 높이가 너무 낮음 (Z={pose[2]:.1f})')
        return pose

    def _grip_approach(self, grip_z):
        """열린 그리퍼로 집기 높이까지 Force 하강. 도중 접촉·초과 하강이면 False (다음 각도로)"""
        if self.sim_boxes is not None:
            self._move_z(grip_z)
            return True, ''
        result, pose = self._force_down(grip_z, GRIP_APPROACH_FORCE_TARGET_N, GRIP_APPROACH_CONTACT_N,
                                        GRIP_APPROACH_SAFETY_N, check_xyz=True)
        if result == 'contact':
            return False, f'내려가다 걸림 (Z={pose[2]:.1f})'
        if abs(pose[2] - grip_z) > GRIP_APPROACH_Z_TOL_MM:
            return False, f'목표 높이 초과 (Z={pose[2]:.1f}, 목표 {grip_z:.1f})'
        return True, ''

    # ==================================================================
    # 높이 탐지 + 집기 (원본 pick_and_place의 앞부분 + try_pick_and_place의 집기)
    # ==================================================================
    def _find_and_grip(self, contact, reference, report):
        """부채꼴 4점 높이 탐지 → 처음 찾은 점에서 최대 3번 집기.
        성공: dict(grip_z, top_z, orientation) / 4점 모두 높이 없음: None / 집기 실패: PartError('GRASP_FAILED')"""
        r = self.robot
        if contact['stopped_pose'][2] >= PROBE_START_Z:
            raise RuntimeError('접촉 Z가 높이 탐지 시작 높이보다 높음')
        self._move_z(PROBE_START_Z)
        self._orient(reference, TILT_VEL, TILT_ACC)
        points = sector_probe_points(contact)
        for i, probe in enumerate(points, 1):
            r.movejx([*probe, *r.pose()[3:6]], vel=TRANSFER_JOINT_VEL, acc=TRANSFER_JOINT_ACC)
            try:
                top = self._probe_height()
            except ProbeMiss as miss:
                self.log.info(f'[scan] 높이 {i}/{len(points)} 없음: {miss}')
                self._move_z(PROBE_START_Z)
                continue
            if math.dist(top[:2], probe[:2]) > PROBE_XY_DRIFT_LIMIT_MM:
                raise RuntimeError('윗면 접촉점이 계획한 탐지점에서 벗어남')
            report.log(f'윗면 높이 {top[2]-Z0:.1f} mm (탐지점 {i}/{len(points)})')
            self._move_z(PROBE_START_Z)
            return self._grip(top, reference, report)
        self._orient(reference, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)
        return None

    def _grip(self, top, reference, report):
        r = self.robot
        grip_z = calculate_grip_z(top[2])
        retry_z = max(PROBE_START_Z, top[2] + GRIP_RETRY_LIFT_MM, grip_z + GRIP_RETRY_LIFT_MM)
        self._move([top[0], top[1], retry_z, *r.pose()[3:6]])
        initial = r.pose()[3:6]
        angles = GRIP_RETRY_ANGLES[:GRIP_RETRY]
        why = ''
        for n, angle in enumerate(angles, 1):
            o = initial[:]
            o[2] += angle                                  # ZYZ 마지막 회전 = 툴 Z축 기준
            self._open()
            self._orient(o, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)
            ok, why = self._grip_approach(grip_z)
            if ok:
                res = r.grip_close(GRIP_FORCE_N or GRIPPER['grip_force_n'])
                # 11/18: 잡힘 비트만 보지 않고 폭이 박스 범위 안인지 확인 (한쪽 핑거만 걸려 멈춘 경우 거름,
                #        원본은 WebLogic 폭 조건 → DI로 같은 확인). 들어 올린 뒤 폭이 변했으면 놓친 것
                ok, why = check_grip(res.width_mm, 1) if res.grabbed else (False, f'빈손으로 닫힘 (폭 {res.width_mm:.1f} mm)')
                if ok:
                    self._grip_width = res.width_mm
                    # 11/18 (적재와 같음): 40N이 다 걸리기 전에 들면 박스가 핑거 사이로 빠짐
                    #   → common.GRIPPER settle_after_grip_s 기다린 뒤, 처음 lift_slow_mm는 lift_slow_vel로 천천히
                    r.wait(GRIPPER['settle_after_grip_s'])
                    p = r.pose()
                    p[2] = grip_z + min(GRIPPER['lift_slow_mm'], PICK_LIFT_MM)
                    slow = GRIPPER['lift_slow_vel']
                    r.movel(p, vel=[slow, 20.0], acc=[slow * 3, 40.0], tol=1.0)
                    self._move_z(grip_z + PICK_LIFT_MM)
                    time.sleep(PICK_HOLD_SECONDS)
                    if self._holding():
                        self._sim_take()
                        report.log(f'집기 성공 ({n}/{len(angles)}번째, {angle:+g}도, 폭 {res.width_mm:.1f} mm)')
                        if angle == 45.0:
                            # 물품을 들어 올린 뒤 집기 전 0도 자세로 복귀
                            self._move_z(retry_z)
                            self._orient(initial, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)
                            o = initial
                        return {'grip_z': grip_z, 'top_z': top[2], 'orientation': o}
                    why = f'들어 올리다 놓침: {self._hold_why}'
                self._open()
            nxt = f'다음 {angles[n]:+g}도' if n < len(angles) else '포기'
            report.log(f'집기 실패 {n}/{len(angles)} ({angle:+g}도): {why} → {nxt}', 'WARN')
            self._move_z(retry_z)
        self._move_z(retry_z)
        self._orient(reference, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)   # 누적 회전 방지
        raise PartError('GRASP_FAILED', f'{len(angles)}회 모두 실패 ({why}). 그리퍼 연 채 높이 {retry_z:.0f}mm')

    # ==================================================================
    # 측정존 이동 + 내려놓기
    # ==================================================================
    def _carry_and_place(self, grip, zone, home_z, report):
        r = self.robot
        contact_z = zone[2] + (grip['grip_z'] - Z0)        # 예상: 박스 바닥이 측정존 바닥(z)에 닿는 TCP Z
        transfer_z = max(home_z, PROBE_START_Z, grip['top_z'] + PROBE_CLEARANCE_MM,
                         contact_z + PLACE_SLOW_FROM_MM + PICK_LIFT_MM)
        report('moving')
        r.movejx([zone[0], zone[1], transfer_z, *grip['orientation']],
                 vel=TRANSFER_JOINT_VEL, acc=TRANSFER_JOINT_ACC)
        report('placing')
        if not self._holding():
            raise RuntimeError(f'측정존 도착 전 박스를 놓침 ({self._hold_why}) — 떨어진 위치 확인 필요')
        self._move_z(contact_z + PLACE_SLOW_FROM_MM)
        status, z = self._descend_until_contact(zone[2] + PLACE_MIN_ABOVE_FLOOR_MM, contact_z)
        if status == 'clear':
            report.log(f'측정존 바닥 접촉 없이 하한 도달 (Z={z:.1f}) → 그 자리에서 놓음', 'WARN')
        self._open()
        self._move_z(transfer_z)

    def _descend_until_contact(self, z_target, expected_z=None):
        """판별 _pos_descent 방식: 비동기로 천천히 내려가며 Fz 증가를 보다 닿으면 빠른 정지.
        z_target: 하강 하한(측정존 바닥 바로 위), expected_z: 예상 접촉 높이(sim에서만 사용)"""
        r = self.robot
        if self.sim_boxes is not None:              # 가상: 예상 접촉 높이까지 가서 닿은 셈
            self._move_z(max(z_target, expected_z if expected_z is not None else z_target))
            return 'contact', r.pose()[2]
        time.sleep(0.3)
        samples = [r.force('base')[2] for _ in range(10)]
        if max(samples) - min(samples) > PROBE_NOISE_MAX_N:
            raise RuntimeError('내려놓기 전 Fz 불안정')
        fz0 = sum(samples) / len(samples)
        z0 = r.pose()[2]
        dist = z0 - z_target
        if dist <= 0.2:
            return 'clear', z0
        r.amovel_rel([0.0, 0.0, -dist, 0.0, 0.0, 0.0], vel=PLACE_DESCENT_VEL, acc=PLACE_DESCENT_VEL*4)
        t0, status = time.monotonic(), 'timeout'
        while time.monotonic() - t0 < dist/PLACE_DESCENT_VEL + 3.0:
            d = r.force('base')[2] - fz0
            z = r.pose()[2]
            if d >= PLACE_SAFETY_N:
                status = 'safety'
                break
            if d >= PLACE_CONTACT_N and time.monotonic() - t0 >= PLACE_IGNORE_S:
                status = 'contact'
                break
            if z <= z_target + 0.2:
                status = 'clear'
                break
            time.sleep(0.02)
        if status != 'clear':
            r.move_stop(1)
        self._wait_still()
        z = r.pose()[2]
        if status == 'safety':
            raise RuntimeError(f'내려놓기 중 과도한 힘 (Z={z:.1f})')
        if status == 'timeout':
            raise TimeoutError('내려놓기 하강 시간 초과')
        return status, z

    # ==================================================================
    # 에뮬레이터 가상 박스 (sim_boxes)
    # ==================================================================
    def _sim_box_at(self, x, y, reach):
        for b in self.sim_boxes:
            if abs(x - b[0]) <= reach and abs(y - b[1]) <= reach:
                return b
        return None

    def _sim_fy(self, pose, offset):
        fy = offset or 0.0
        if abs(pose[2] - SCAN_Z) < 5.0 and self._sim_box_at(pose[0], pose[1], SIM_TOUCH_MM):
            fy += SIM_CONTACT_FY
        return fy

    def _sim_probe(self, floor_z):
        p = self.robot.pose()
        b = self._sim_box_at(p[0], p[1], SIM_HALF_MM)
        if b is None or Z0 + b[2] < floor_z:
            self._move_z(floor_z)
            raise ProbeMiss('(sim) 하한까지 내려갔지만 윗면 없음')
        self._move_z(Z0 + b[2])
        return self.robot.pose()

    def _sim_take(self):
        if self.sim_boxes is None:
            return
        p = self.robot.pose()
        b = self._sim_box_at(p[0], p[1], SIM_HALF_MM + 5.0)
        if b is not None:
            self.sim_boxes.remove(b)
            self.log.info(f'[scan] (sim) 가상 박스 {b} 집음 → 남은 {self.sim_boxes}')

    # ==================================================================
    # 좌표
    # ==================================================================
    @staticmethod
    def _goal_poses(poses):
        home = poses.get('home_j')
        zone = poses.get('measure_zone')
        if home is None or len(home) != 6:
            raise ValueError('Goal에 HOME 관절 좌표가 없음')
        if zone is None or len(zone) != 3 or all(abs(v) < 1e-6 for v in zone):
            raise ValueError('Goal에 측정존 좌표가 없음 (0,0,0은 미설정)')
        return list(home), list(zone)
