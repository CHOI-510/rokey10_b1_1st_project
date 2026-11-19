#!/usr/bin/env python3
"""
scan_algorithm.py — box_scan 탐색·집기·이송 (담당: 김창균)

scan(poses, report)는 Goal마다 물품 한 개를 측정존에 배치하고 HOME으로 복귀한다.
성공 시 True, 남은 경로를 모두 훑고 물품을 발견하지 못하면 False를 반환한다.
GRASP_FAILED는 탐색 커서를 유지해 관제 재요청 시 같은 경로에서 이어서 탐색한다.
다른 오류가 발생하거나 남은 경로를 모두 훑으면 커서를 초기화한다.

- space_scan: 툴 Z축 기준 90도 회전한 닫힌 RG2로 Y 방향 지그재그 탐색. 경로별 Fy 기준값을 뺀
  변화량의 절댓값이 FORCE_LIMIT 이상이면 정지한다. 보정표 범위 밖에서는
  판정하지 않는다. 재개 시 멈춘 줄의 원래 시작점으로 돌아가고,
  재출발 후 40 mm 구간은 감지에서 제외한다.
- picking: 측면 정지점을 기준으로 진행 방향 앞쪽 반원에서 윗면을 탐침한다.
  연속 검출 각도의 양 끝 평균 방향으로 직선 탐침해 먼 경계를 좁힌다.
  측면 정지점과 경계의 중간에서 첫 윗면 높이를 사용하고, 0°, +90°, +45° 순으로 집기한다.
  동기 닫기, 재조임 및 파지 판별은 기존 통합 방식에 따른다.
- moving: 수직 상승 후 파지를 확인하고 측정존까지 수평 직선 이동한다.
  +45°에서 집은 경우는 이송 높이에서 최초 집기 자세로 회전하고 파지를 다시 확인한다.
- placing: 예상 접촉점 위 20 mm에서 순응·힘 제어로 하강한다.
  허용 높이에서 Fz 접촉과 정지를 확인한 뒤에만 놓는다.

ROS 파라미터
- csv_dir: 지정한 경우에만 탐색 중 힘과 보정값을 CSV로 기록한다.
- sim_boxes: 에뮬레이터 전용 "x,y,h;x,y,h". h는 Z0 기준 윗면 높이(mm).
  탐색 외력과 높이 탐지를 모의하며 집은 박스는 목록에서 제거한다.
  BOX_GRIPPER=fake와 함께 사용한다.
"""
import csv
import math
import time
from bisect import bisect_right
from datetime import datetime
from pathlib import Path

from box_comm.common import GRIPPER, PartError
from box_comm.sync_grip import GripMissed, make_sync_gripper, tip_h_mm
from box_comm.common import SYNC_GRIP, TCP

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
    if not 0 < PLACE_CONTACT_N < PLACE_FORCE_TARGET_N < PLACE_SAFETY_N:
        raise ValueError('배치: 0 < 접촉 < 목표 < 안전 힘')
    if not 0 < PLACE_CONTACT_Z_TOL_MM < PLACE_SLOW_FROM_MM:
        raise ValueError('배치 접촉 Z 허용차는 시작 여유보다 작아야 함')
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
    if not PROBE_FAN_ANGLES_DEG or any(not -90 <= a <= 90 for a in PROBE_FAN_ANGLES_DEG):
        raise ValueError('반원 탐침 각도는 -90~90도여야 함')
    if not (0 < PROBE_FAN_RADIUS_MM < PROBE_RAY_FIRST_MM
            and PROBE_RAY_BISECTIONS > 0 and PROBE_TOP_TOL_MM > 0):
        raise ValueError('반원·직선 탐침 거리 또는 허용차 설정 오류')
    if not math.isfinite(GRIP_WIDTH_RANGE_MARGIN_MM) or GRIP_WIDTH_RANGE_MARGIN_MM < 0:
        raise ValueError('쥠 폭 범위 여유는 0 이상의 유한한 값이어야 함')
    if not math.isfinite(GRIP_HOLD_WIDTH_TOL_MM) or GRIP_HOLD_WIDTH_TOL_MM <= 0:
        raise ValueError('쥠 폭 변화 허용은 양의 유한한 값이어야 함')
    if not math.isfinite(SCAN_GRIPPER_ROTATE_DEG):
        raise ValueError('탐색 그리퍼 회전각은 유한한 값이어야 함')
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


def scan_orientation(reference, direction):
    """Y 탐색 자세: 기존 후방 기울기 유지 + 툴 로컬 Z축 회전.

    ZYZ 마지막 각도에 더하면 툴 Z 방향(기울기)은 그대로이고 핑거 방향만 바뀐다.
    높이 탐지/집기에서 쓰는 HOME 기준 reference는 변경하지 않는다.
    """
    angles = backward_orientation(reference, direction)
    angles[2] += SCAN_GRIPPER_ROTATE_DEG
    return angles


def probe_direction(scan_direction, angle):
    """탐색 방향과 반원 각도를 BASE XY 단위벡터로 변환한다."""
    sign = -1 if scan_direction == '-Y' else 1
    radians = math.radians(angle)
    return -sign * math.sin(radians), sign * math.cos(radians)


def locate_probe_edge(distance_limit, sample_top):
    """85 mm에서 같은 윗면이면 그 거리를 끝점으로 사용한다."""
    inside = PROBE_FAN_RADIUS_MM
    outside = min(distance_limit, PROBE_RAY_FIRST_MM)
    if outside <= inside:
        raise ProbeMiss('작업 범위 안에서 직선 탐침 경계를 확인하지 못함')
    try:
        distance = outside
        if sample_top(distance):
            if distance == PROBE_RAY_FIRST_MM:
                return distance
            raise ProbeMiss('85 mm 전에 작업 범위 한계에 도달해 경계를 확인하지 못함')
        for _ in range(PROBE_RAY_BISECTIONS):
            distance = (inside + outside) / 2.0
            if sample_top(distance):
                inside = distance
            else:
                outside = distance
    except DifferentProbeObject:
        # 다른 물체를 찍은 지점은 바깥쪽 경계 후보로만 사용한다.
        outside = distance
    return (inside + outside) / 2.0


def calculate_grip_z(top_z):
    """윗면보다 지정 깊이만큼 아래인 핑거 끝 목표 Z를 계산한다."""
    return top_z - GRIP_DEPTH_FROM_TOP_MM


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


class DifferentProbeObject(Exception):
    """유효한 접촉이지만 첫 윗면과 높이가 다름."""


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

    def add(self, stamp, lane, phase, direction, pose, fy, baseline, residual, status, enabled, contact, speed, acceleration):
        if self.origin is None:
            self.origin = stamp
        if self.segment_origin is None:
            self.segment_origin = stamp
        interval = '' if self.previous is None else stamp-self.previous
        self.rows.append((stamp-self.origin, stamp-self.segment_origin, interval, lane, phase, direction,
                          *pose[:3], fy, '' if baseline is None else baseline,
                          '' if residual is None else residual, status, int(enabled), int(contact),
                          speed, acceleration, 1))
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
        r.movej(home, vel=ANGULAR_VEL, acc=ANGULAR_ACC)
        here = r.pose()
        reference, home_z = here[3:6], here[2]           # HOME 자세의 ABC가 기준 자세
        orient = {d: scan_orientation(reference, d) for d in ('-Y', '+Y')}
        self._close()                                    # 탐색·높이 탐지는 닫힌 핑거로

        if self.resume is not None:
            self._go_resume(self.resume, orient)
            report.log(f'{self.lane+1}번 선 시작점부터 다시 탐색')
            ignore = RESUME_IGNORE_MM
        else:
            x = X_LANES[self.lane]
            d = lane_direction(self.lane)
            start_y = Y_START if d == '-Y' else Y_END
            r.movejx([x, start_y, SCAN_Z, *orient[d]], vel=ANGULAR_VEL, acc=ANGULAR_ACC)
            r.wait(SETTLE_SECONDS)
            ignore = 0.0

        while self.lane < len(X_LANES):
            lane = self.lane + 1
            x = X_LANES[self.lane]
            d = lane_direction(self.lane)
            end_y = Y_END if d == '-Y' else Y_START
            o = orient[d]
            self._orient(o)
            r.wait(BETWEEN_LINES_WAIT)
            while abs(r.pose()[1] - end_y) > 1.0:            # 재개점이 선 끝이면 바로 다음 선
                contact = self._scan_line([x, end_y, SCAN_Z, *o], d, lane, ignore)
                if contact is None:
                    break
                sp = contact['stopped_pose']
                report.log(f'물품 접촉: {lane}번 선 {d}, x={sp[0]:.0f}, y={sp[1]:.0f}, '
                           f'dFy={contact["residual_n"]:+.2f} N')
                self.resume = contact                       # 현재 경로의 재개 기준점 저장
                report('picking')
                grip = self._find_and_grip(contact, reference, report)
                if grip is not None and self._carry_and_place(grip, zone, home_z, report):
                    r.movej(home, vel=ANGULAR_VEL, acc=ANGULAR_ACC)
                    report.log('물품 1개 측정존 배치 완료, HOME 복귀')
                    return True
                failure = '반원·직선 중심 탐지 실패' if grip is None else '이송 높이에서 파지 확인 실패'
                # 같은 위치에서 재탐지와 실패가 반복되는 횟수를 제한한다.
                if (self._last_miss and self._last_miss[0] == lane
                        and abs(self._last_miss[1] - sp[1]) < HEIGHT_MISS_SAME_SPOT_MM):
                    misses = self._last_miss[2] + 1
                else:
                    misses = 1
                self._last_miss = (lane, sp[1], misses)
                if misses >= HEIGHT_MISS_LIMIT:
                    self._on_scan_blocked(contact, misses, report, failure)
                report.log(f'{failure} ({misses}/{HEIGHT_MISS_LIMIT}) → 탐색 계속', 'WARN')
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

        r.movej(home, vel=ANGULAR_VEL, acc=ANGULAR_ACC)
        self._reset_cursor()
        report.log('작업공간 전체 탐색 완료: 물품 없음')
        return False

    def _on_scan_blocked(self, contact, misses, report, failure):
        """같은 자리에서 높이 탐지 또는 상승 후 파지 확인이 HEIGHT_MISS_LIMIT번 실패 = 스캔공간 막힘 (낮거나 집을 수 없는 물체 추정).
        TODO(팀 결정): 지금은 임시로 높은 곳으로 올라가 멈춤(SCAN_FAILED → 관제 ERROR, 사람이 치움).
          후보: 그 물체를 건너뛰고 다음 선부터 계속 / 관제에 별도 코드(예: SCAN_BLOCKED)로 알림"""
        sp = contact['stopped_pose']
        msg = (f'스캔공간 막힘: {contact["lane"]}번 선 x={sp[0]:.0f}, y={sp[1]:.0f}에서 '
               f'{failure} {misses}회 → 사람이 확인')
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

    def _orient(self, abc):
        """제자리에서 자세만 바꾸기 (원본 set_scan_orientation)"""
        p = self.robot.pose()
        if rotation_error(p[3:6], abc) <= 1e-6:
            return
        self.robot.movel([*p[:3], *abc], vel=[5.0, ANGULAR_VEL], acc=[10.0, ANGULAR_ACC], tol=1.0)
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
        """열고 실제 열린 원시 폭을 돌려줌 (sync 집기 높이 계산용)"""
        raw = self.robot.grip_open(OPEN_WIDTH_MM, None)
        self._grip_width = None
        return raw

    def _check_grip(self, width_mm, grip_bit, ref_width_mm=None):
        """탐색 전용: 잡힘/빈손은 검사하고, 폭 범위·변화량만 여유를 둔다."""
        if not math.isfinite(width_mm):
            return False, '유효하지 않은 그리퍼 폭'
        if not grip_bit:
            return False, f'잡힘 신호 없음 (폭 {width_mm:.1f} mm)'
        if width_mm <= GRIPPER['empty_width_mm']:
            return False, f'빈손으로 닫힘 (폭 {width_mm:.1f} mm)'
        lo, hi = GRIPPER['box_width_range_mm']
        margin = GRIP_WIDTH_RANGE_MARGIN_MM
        if not lo - margin <= width_mm <= hi + margin:
            return False, f'폭 {width_mm:.1f} mm가 탐색 허용 범위 {lo-margin:g}~{hi+margin:g} mm 밖'
        tolerance = max(GRIP_HOLD_WIDTH_TOL_MM, GRIPPER['hold_width_tol_mm'])
        if ref_width_mm is not None:
            if not math.isfinite(ref_width_mm):
                return False, '유효하지 않은 기준 파지 폭'
            if abs(width_mm - ref_width_mm) > tolerance:
                return False, f'폭이 {ref_width_mm:.1f} → {width_mm:.1f} mm로 변함 (허용 {tolerance:g} mm 초과)'
        return True, ''

    def _holding(self) -> bool:
        """닫기/재조임과 같은 탐색 전용 기준으로 이송 중 파지 확인."""
        st = self.robot.gripper.status()
        width = self.robot.gripper.calibrated(st['width_raw_mm'])
        ok, self._hold_why = self._check_grip(
            width, st['grip'], getattr(self, '_grip_width', None))
        return ok

    def _go_resume(self, contact, orient):
        """멈춘 선의 원래 시작점에서 같은 방향으로 그 줄 전체를 다시 훑는다."""
        lane, d = contact['lane'], contact['direction']
        if not 1 <= lane <= len(X_LANES) or d != lane_direction(lane-1):
            raise ValueError('재개할 탐색 선 또는 방향이 잘못됨')
        y = Y_START if d == '-Y' else Y_END
        x = X_LANES[lane-1]
        self._move_z(max(PROBE_START_Z, self.robot.pose()[2]))
        self.robot.movejx([x, y, PROBE_START_Z, *orient[d]], vel=ANGULAR_VEL, acc=ANGULAR_ACC)
        self._move_z(SCAN_Z)
        self.robot.wait(SETTLE_SECONDS)

    # ==================================================================
    # 탐색 한 줄 (원본 Robot.scan_line)
    # ==================================================================
    def _scan_line(self, target, direction, lane, ignore_mm=0.0, phase='scan'):
        """Y 탐색에서는 접촉을 판정하고, X 경로 전환에서는 일반 직선 속도로 이동하며 기록한다."""
        r = self.robot
        start_y = r.pose()[1]
        sign = 1.0 if direction == '+Y' else -1.0
        scanning = phase == 'scan'
        speed, acceleration = (SCAN_VEL, SCAN_ACC) if scanning else (LINEAR_VEL, LINEAR_ACC)
        r.amovel(target, vel=[speed, 10.0], acc=[acceleration, 20.0])
        started = next_sample = time.monotonic()
        try:
            while True:
                status = r.motion_state()
                now = time.monotonic()
                if now >= next_sample:
                    fy = r.force('base')[1]                  # 정지 확인 + 1초마다 진행 알림 포함
                    pose = r.pose()
                    offset = baseline_fy(pose[1], direction, lane) if scanning else None
                    if self.sim_boxes is not None:
                        fy = self._sim_fy(pose, offset)
                    residual = None if offset is None else fy - offset
                    enabled = status != 0 and residual is not None and sign*(pose[1]-start_y) >= ignore_mm
                    contact = enabled and abs(residual) >= FORCE_LIMIT
                    if self.recorder:
                        self.recorder.add(now, lane, phase, direction, pose, fy, offset, residual,
                                          status, enabled, contact, speed, acceleration)
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
    def _force_down(self, stop_z, target_n, contact_n, safety_n, check_xyz, *, contact_details=None):
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
                    if contact_details is not None:
                        contact_details.update(z=r.pose()[2], dfz=dfz)
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
        floor_z = PROBE_MIN_Z
        if self.sim_boxes is not None:
            return self._sim_probe(floor_z)
        result, pose = self._force_down(floor_z, PROBE_FORCE_TARGET_N, PROBE_CONTACT_N, PROBE_SAFETY_N,
                                        check_xyz=False)
        if result != 'contact':
            raise ProbeMiss('하한까지 내려갔지만 윗면 없음')
        if pose[2] < floor_z:
            raise ProbeMiss(f'접촉 높이가 너무 낮음 (Z={pose[2]:.1f})')
        return pose

    def _sync_approach(self, gz):
        """열린 핑거를 순응·힘 제어로 내리고, 재시도할 경우 이유를 반환한다."""
        if self.sim_boxes is not None:
            self._move_z(gz)
            return None
        if not math.isfinite(gz):
            raise ValueError('집기 접근 목표 Z가 유효하지 않음')

        result, stopped_pose = self._force_down(
            gz, GRIP_APPROACH_FORCE_TARGET_N, GRIP_APPROACH_CONTACT_N,
            GRIP_APPROACH_SAFETY_N, check_xyz=True)
        if result == 'contact':
            return f'열린 핑거 하강 중 접촉 (ΔFz ≥ {GRIP_APPROACH_CONTACT_N:g} N)'
        actual_z = stopped_pose[2]
        if abs(actual_z - gz) > GRIP_APPROACH_Z_TOL_MM:
            return f'힘 제어 해제 후 집기 Z 오차 (현재 {actual_z:.1f}, 목표 {gz:.1f} mm)'
        return None

    # ==================================================================
    # 높이 탐지 + 집기 (원본 pick_and_place의 앞부분 + try_pick_and_place의 집기)
    # ==================================================================
    def _find_and_grip(self, contact, reference, report):
        """측면 정지점에서 반원·직선 탐침으로 집기 중심을 추정한다."""
        origin = contact['stopped_pose']
        direction = contact['direction']
        if origin[2] >= PROBE_START_Z:
            raise RuntimeError('접촉 Z가 높이 탐지 시작 높이보다 높음')
        if direction not in ('-Y', '+Y'):
            raise ValueError(f'높이 탐침 방향 오류: {direction}')
        self._move_z(PROBE_START_Z)
        probe_orientation = list(reference)
        probe_orientation[2] += PROBE_TOOL_Z_ROTATION_DEG
        self._orient(probe_orientation)
        x_min, x_max = min(X_LANES), max(X_LANES)
        y_min, y_max = sorted((Y_START, Y_END))

        def xy(ux, uy, distance):
            return origin[0] + ux * distance, origin[1] + uy * distance

        def sample(x, y, reference_top=None):
            self._move_z(PROBE_START_Z)
            self._move([x, y, PROBE_START_Z, *probe_orientation])
            try:
                top = self._probe_height()
            except ProbeMiss:
                self.log.info(f'[scan] 탐침 X={x:.1f}, Y={y:.1f}: 미접촉')
                return None
            if math.dist(top[:2], (x, y)) > PROBE_XY_DRIFT_LIMIT_MM:
                raise RuntimeError('윗면 접촉점이 계획한 탐지점에서 벗어남')
            self.log.info(f'[scan] 탐침 X={x:.1f}, Y={y:.1f}, Z={top[2]:.1f}')
            if reference_top is not None and abs(top[2] - reference_top) > PROBE_TOP_TOL_MM:
                report.log(f'다른 물체 높이 감지: 기준 Z={reference_top:.1f}, 접촉 Z={top[2]:.1f}', 'WARN')
                raise DifferentProbeObject()
            return top

        angles = []
        first_z = None
        try:
            fan_angles = (PROBE_FAN_ANGLES_DEG if direction == '-Y'
                          else reversed(PROBE_FAN_ANGLES_DEG))
            for angle in fan_angles:
                x, y = xy(*probe_direction(direction, angle), PROBE_FAN_RADIUS_MM)
                if not (x_min <= x <= x_max and y_min <= y <= y_max):
                    if angles:
                        break
                    continue
                try:
                    top = sample(x, y, first_z)
                except DifferentProbeObject:
                    break
                if top is None:
                    if angles:
                        break
                    continue
                if first_z is None:
                    first_z = top[2]
                angles.append(angle)
            if not angles:
                raise ProbeMiss('반원에 유효 윗면 없음')

            angle = (angles[0] + angles[-1]) / 2.0
            ux, uy = probe_direction(direction, angle)
            limits = [PROBE_RAY_FIRST_MM]
            for position, component, lower, upper in (
                    (origin[0], ux, x_min, x_max), (origin[1], uy, y_min, y_max)):
                if abs(component) > 1e-12:
                    limits.append(((upper if component > 0 else lower) - position) / component)
            limit = min(limits)

            def sample_ray(distance):
                x, y = xy(ux, uy, distance)
                return sample(min(x_max, max(x_min, x)), min(y_max, max(y_min, y)), first_z)

            edge = locate_probe_edge(limit, sample_ray)
            if edge == PROBE_RAY_FIRST_MM:
                report.log(f'{edge:g} mm에서 같은 윗면 확인 → 추가 탐침 없이 집기')
        except ProbeMiss as miss:
            self._move_z(PROBE_START_Z)
            report.log(f'중심 탐지 실패: {miss}', 'WARN')
            return None

        center_x, center_y = xy(ux, uy, edge / 2.0)
        # 직선 탐침 기울기로 정렬한다. 평행 핑거의 180도 대칭을 이용한다.
        ray_angle = math.degrees(math.atan2(ux, -uy))
        ray_angle = (ray_angle + 90.0) % 180.0 - 90.0
        grip_orientation = probe_orientation[:]
        grip_orientation[2] -= ray_angle
        report.log(f'반원 성공 {angles}, 방향 {angle:+.1f}도, 먼 경계 {edge:.1f} mm, '
                   f'집기 중심 X={center_x:.1f}, Y={center_y:.1f}, 윗면 Z={first_z:.1f}')
        self._move_z(PROBE_START_Z)
        return self._grip([center_x, center_y, first_z], reference, report, grip_orientation)

    def _grip(self, top, reference, report, grip_orientation):
        """sync 집기 (11/18 정쓰, 적재와 같은 방식). 높이 탐지한 윗면 기준:
        윗면 위로 올라감 → 100mm로 열기 → 핑거 끝이 윗면 − GRIP_DEPTH_FROM_TOP_MM에 오는 TCP 높이까지 하강
        (순응·힘 제어로 하강하고 조기 접촉 감시) → sync로 닫기(핑거 끝 높이 유지) → 40N 조임 → 0.5초 → 천천히 들기.
        못 잡으면 그리퍼 열고 올라가서 툴 각도를 바꿔(GRIP_RETRY_ANGLES) 다시.
        얇은 박스라 윗면 − GRIP_DEPTH_FROM_TOP_MM 값이 작업대 바닥(TABLE_Z) + 2.6보다 낮으면 TABLE_Z + 2.6에서 집음 (적재의 '바닥 + 2.6'과 같은 기준)."""
        r = self.robot
        tip_z = calculate_grip_z(top[2])                   # 핑거 끝이 물 높이 = 윗면 − GRIP_DEPTH_FROM_TOP_MM
        floor_tip = TABLE_Z + SYNC_GRIP['tip_above_floor_mm']   # 핑거 끝 높이 하한
        on_floor = tip_z < floor_tip
        tip_z = max(tip_z, floor_tip)
        sg = make_sync_gripper(r, tip_z - SYNC_GRIP['tip_above_floor_mm'],
                               log=lambda m: self.log.info(f'[scan] {m}'))
        retry_z = max(PROBE_START_Z, top[2] + GRIP_RETRY_LIFT_MM)
        self._move([top[0], top[1], retry_z, *r.pose()[3:6]])   # 높이 탐지 뒤 위로
        initial = list(grip_orientation)
        angles = GRIP_RETRY_ANGLES
        force = GRIP_FORCE_N or GRIPPER['grip_force_n']
        why = ''
        for n, angle in enumerate(angles, 1):
            o = initial[:]
            o[2] += angle                                  # ZYZ 마지막 회전 = 툴 Z축 기준
            raw_open = self._open()                        # 최대로 열기 (OPEN_WIDTH_MM, 기본 100)
            self._orient(o)
            gz = sg.grasp_z_for_open(raw_open)             # 열린 폭에서 핑거 끝이 tip_z가 되는 TCP Z
            approach_failure = self._sync_approach(gz)
            if approach_failure is not None:
                why = approach_failure
                nxt = f'다음 {angles[n]:+g}도' if n < len(angles) else '포기'
                report.log(f'집기 실패 {n}/{len(angles)} ({angle:+g}도): {why} → {nxt}', 'WARN')
                self._move_z(retry_z)
                continue
            try:
                res = sg.close_with_lift(SYNC_GRIP['est_raw_mm'])
            except GripMissed as e:
                ok, why = False, str(e)
            else:
                width = res['width_mm']
                ok, why = self._check_grip(width, 1)             # 폭 범위 (한쪽 핑거만 걸린 경우 거름)
                if ok and force > SYNC_GRIP['close_force_n']:
                    tight = r.grip_close(force)            # 같은 자리에서 40N으로 다시 조임
                    ok, why = (self._check_grip(tight.width_mm, 1) if tight.grabbed
                               else (False, '다시 조이다 놓침'))
                    # 재조임 전후 폭 변화는 판정하지 않음. 재조임 후 폭을 이송 감시 기준으로 저장.
                    width = tight.width_mm if tight.grabbed else width
            if ok:
                self._grip_width = width
                hold = r.pose()
                raw_hold = r.gripper.status()['width_raw_mm']
                depth = top[2] - (hold[2] + TCP['pos'][2] - tip_h_mm(raw_hold))   # 실제 문 깊이 (윗면 아래)
                r.wait(GRIPPER['settle_after_grip_s'])     # 40N이 다 걸린 뒤 들기
                p = hold[:]
                p[2] = hold[2] + min(GRIPPER['lift_slow_mm'], PICK_LIFT_MM)
                slow = GRIPPER['lift_slow_vel']
                r.movel(p, vel=[slow, 20.0], acc=[slow * 3, 40.0], tol=1.0)
                note = ', 얇아서 작업대 바닥 + 2.6 기준' if on_floor else ''
                return {'grip_z': hold[2], 'top_z': top[2],
                        'transfer_alignment': initial if angle == 45.0 else None,
                        'tip_off': TCP['pos'][2] - tip_h_mm(raw_hold),
                        'grasp_summary': (f'{n}/{len(angles)}번째, {angle:+g}도, 폭 {width:.1f} mm, '
                                          f'핑거 끝 윗면 − {depth:.0f} mm{note}')}
            self._open()
            nxt = f'다음 {angles[n]:+g}도' if n < len(angles) else '포기'
            report.log(f'집기 실패 {n}/{len(angles)} ({angle:+g}도): {why} → {nxt}', 'WARN')
            self._move_z(retry_z)
        self._move_z(retry_z)
        self._orient(reference)   # 누적 회전 방지
        raise PartError('GRASP_FAILED', f'{len(angles)}회 모두 실패 ({why}). 그리퍼 연 채 높이 {retry_z:.0f}mm')

    # ==================================================================
    # 측정존 이동 + 내려놓기
    # ==================================================================
    def _carry_and_place(self, grip, zone, home_z, report):
        r = self.robot
        contact_z = zone[2] + (grip['grip_z'] - TABLE_Z)   # 예상: 박스 바닥이 측정존 바닥(z)에 닿는 TCP Z
        transfer_z = max(home_z, PROBE_START_Z, grip['top_z'] + PROBE_CLEARANCE_MM,
                         contact_z + PLACE_SLOW_FROM_MM + PICK_LIFT_MM)
        report('moving')
        # 파지 위치에서 수직으로 이탈한 뒤 도달한 높이와 자세로 수평 이송한다.
        self._move_z(max(transfer_z, r.pose()[2]))
        time.sleep(PICK_HOLD_SECONDS)
        if not self._holding():
            report.log(f'이송 높이에서 파지 실패: {self._hold_why}', 'WARN')
            self._open()
            self._close()
            return False
        self._sim_take()
        report.log(f"집기 성공 ({grip['grasp_summary']})")
        alignment = grip.get('transfer_alignment')
        if alignment is not None:
            self._orient(alignment)
            if not self._holding():
                raise RuntimeError(f'0도 정렬 중 박스를 놓침 ({self._hold_why})')
            report.log('45도 파지 후 0도 기준 자세로 정렬 완료')
        transfer_pose = r.pose()
        transfer_pose[:2] = zone[:2]
        self._move(transfer_pose)
        report('placing')
        if not self._holding():
            raise RuntimeError(f'측정존 도착 전 박스를 놓침 ({self._hold_why}) — 떨어진 위치 확인 필요')
        # 핑거 끝이 측정존 바닥 아래로 가지 않도록 동기 집기의 오프셋을 반영한다.
        floor_tcp = zone[2] + PLACE_MIN_ABOVE_FLOOR_MM - grip.get('tip_off', 0.0)
        self._place_with_force(contact_z, floor_tcp, report)
        self._open()
        self._move_z(transfer_z)
        return True

    def _place_with_force(self, contact_z, floor_tcp, report):
        """예상 높이에서의 접촉과 정지를 확인한 경우에만 반환한다."""
        upper_z = contact_z + PLACE_CONTACT_Z_TOL_MM
        stop_z = max(floor_tcp, contact_z - PLACE_CONTACT_Z_TOL_MM)
        if not math.isfinite(upper_z) or not math.isfinite(floor_tcp) or stop_z >= upper_z:
            raise RuntimeError('배치 하한과 예상 접촉 구간이 맞지 않음; 파지 유지')

        self._move_z(contact_z + PLACE_SLOW_FROM_MM)
        self._wait_still()
        if not self._holding():
            raise RuntimeError(f'배치 하강 전 파지 실패: {self._hold_why}')

        details = {}
        if self.sim_boxes is not None:
            self._move_z(max(stop_z, contact_z))
            status, stopped_pose = 'contact', self.robot.pose()
            details['z'] = stopped_pose[2]
        else:
            status, stopped_pose = self._force_down(
                stop_z, PLACE_FORCE_TARGET_N, PLACE_CONTACT_N, PLACE_SAFETY_N,
                check_xyz=True, contact_details=details)
        self._wait_still()
        stopped_z = stopped_pose[2]
        if status != 'contact':
            raise RuntimeError(f'배치 접촉 없음: 예상 Z={contact_z:.1f}, 정지 Z={stopped_z:.1f}; 파지 유지')
        detected_z = details['z']
        force_note = f", ΔFz={details['dfz']:.1f} N" if 'dfz' in details else ''
        note = f'예상 Z={contact_z:.1f}, 접촉 Z={detected_z:.1f}, 정지 Z={stopped_z:.1f}{force_note}'
        if not (stop_z <= detected_z <= upper_z and stop_z <= stopped_z <= upper_z):
            raise RuntimeError(f'배치 접촉이 허용 높이 밖: {note}; 파지 유지')
        if not self._holding():
            raise RuntimeError(f'개방 직전 파지 실패: {self._hold_why}; {note}')
        report.log(f'배치 접촉 및 정지 확인: {note}')

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
