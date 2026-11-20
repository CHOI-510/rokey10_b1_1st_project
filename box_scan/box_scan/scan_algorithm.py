#!/usr/bin/env python3
"""
scan_algorithm.py — box_scan 탐색·집기·이송 (담당: 김창균)

scan(poses, report)는 Goal마다 물품 한 개를 측정존에 배치하고 WAYPOINT로 복귀한다.
성공 시 True, Y 본탐색과 X 확인탐색을 모두 마친 경우에만 False를 반환한다.
단계·경로·재개 커서는 Goal 사이에 유지한다. 탐침/그립/배치는 축 공통으로 사용한다.
GRASP_FAILED는 탐색 커서를 유지해 관제 재요청 시 같은 경로에서 이어서 탐색한다.
다른 오류가 발생하거나 남은 경로를 모두 훑으면 커서를 초기화한다.

- space_scan: 닫힌 RG2로 Y → X 지그재그 탐색. 단계별 자세·외력 축·경로별 기준값을 뺀
  변화량의 절댓값이 FORCE_LIMIT 이상이면 정지한다. 보정표 범위 밖에서는
  판정하지 않는다. 재개 시 멈춘 줄의 원래 시작점으로 돌아가고,
  재출발 후 40 mm 구간은 감지에서 제외한다.
- picking: 측면 정지점을 기준으로 진행 방향 앞쪽 반원에서 윗면을 탐침한다.
  연속 검출 각도의 양 끝 평균 방향으로 직선 탐침해 먼 경계를 좁힌다.
  측면 정지점과 경계의 중간에서 첫 윗면 높이를 사용하고, 0°, +90°, +45° 순으로 집기한다.
  반원 후보를 모두 실제로 탐침해 미검출이면, 측면 정지점 XY에서 바닥 기준으로 한 번 집는다.
  동기 닫기, 재조임 및 파지 판별은 기존 통합 방식에 따른다.
- moving: 수직 상승 후 파지를 확인하고 측정존까지 수평 직선 이동한다.
  성공 각도와 무관하게 이송 높이에서 HOME 기준 0도 자세로 정렬하고 파지를 다시 확인한다.
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

from box_comm.common import GRIPPER, PartError, TRAVEL_SPEED_MM_S, TRAVEL_Z
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


def scan_axis(direction):
    if direction not in ('-Y', '+Y', '-X', '+X'):
        raise ValueError(f'탐색 방향 오류: {direction}')
    return 0 if direction[-1] == 'X' else 1


def scan_settings(direction):
    axis = 'X' if scan_axis(direction) == 0 else 'Y'
    return next(p for p in SCAN_PASSES if p['axis'] == axis)


def scan_point(profile, lane_index, end=False):
    axis = 0 if profile['axis'] == 'X' else 1
    forward = lane_index % 2 == 0
    value = profile['end'] if forward == end else profile['start']
    xyz = [0.0, 0.0, profile['z']]
    xyz[axis], xyz[1-axis] = value, profile['lanes'][lane_index]
    return xyz


def lane_direction(index, profile=None):
    p = SCAN_PASSES[0] if profile is None else profile
    sign = 1 if p['end'] > p['start'] else -1
    if index % 2:
        sign = -sign
    return ('+' if sign > 0 else '-') + p['axis']


def validate_config():
    if not 0 < PLACE_CONTACT_N < PLACE_FORCE_TARGET_N < PLACE_SAFETY_N:
        raise ValueError('배치: 0 < 접촉 < 목표 < 안전 힘')
    if not 0 < PLACE_CONTACT_Z_TOL_MM < PLACE_SLOW_FROM_MM:
        raise ValueError('배치 접촉 Z 허용차는 시작 여유보다 작아야 함')
    if not math.isfinite(PLACE_MAX_BELOW_EXPECTED_MM) or PLACE_MAX_BELOW_EXPECTED_MM <= 0:
        raise ValueError('배치 예상 아래 탐색 거리는 양수여야 함')
    if not math.isfinite(PLACE_STOP_Z_TOL_MM) or PLACE_STOP_Z_TOL_MM < 0:
        raise ValueError('배치 정지 Z 확인 여유는 0 이상이어야 함')
    if tuple(p['axis'] for p in SCAN_PASSES) != ('Y', 'X'):
        raise ValueError('탐색 단계는 Y 다음 X여야 함')
    for p in SCAN_PASSES:
        if not p['lanes'] or set(p['baseline']) != set(range(1, len(p['lanes'])+1)):
            raise ValueError('단계별 보정표가 모든 경로를 덮어야 함')
        if len(p['limits']) != len(p['lanes']) or not all(math.isfinite(v) and v > 0 for v in p['limits']):
            raise ValueError('경로별 외력 한계는 경로 수와 일치하는 양수여야 함')
        nodes = p['nodes']
        if len(nodes) < 2 or not all(math.isfinite(v) for v in nodes) or any(b<=a for a,b in zip(nodes,nodes[1:])):
            raise ValueError('보간 노드는 유한한 오름차순이어야 함')
        if not min(p['start'],p['end']) <= nodes[0] < nodes[-1] <= max(p['start'],p['end']):
            raise ValueError('보정 범위가 이동 범위 밖임')
        if not all(math.isfinite(p[k]) and p[k]>0 for k in ('vel','acc','limit','tilt_vel','tilt_acc','transfer_vel','transfer_acc')):
            raise ValueError('단계별 속도·가속도·임계값 오류')
        if not all(math.isfinite(p[k]) for k in ('start','end','z','tilt','roll','wait')) or p['wait']<0 or p['start']==p['end']:
            raise ValueError('단계별 경로·자세·대기 설정 오류')
        if p['z'] >= PROBE_START_Z:
            raise ValueError('탐색 높이는 탐침 시작 높이보다 낮아야 함')
        cross_key = 'y_mm' if p['axis']=='X' else 'x_mm'
        for lane, cross in enumerate(p['lanes'],1):
            entry=p['baseline'][lane]
            if entry[cross_key]!=cross or entry['direction']!=lane_direction(lane-1,p):
                raise ValueError(f"{p['axis']} {lane}번 경로 보정표 위치/방향 불일치")
            values=entry['values']
            if len(values)!=len(nodes) or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
                raise ValueError('보정표 값 오류')
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


def baseline_force(position_mm, direction, lane, profile=None):
    p = scan_settings(direction) if profile is None else profile
    if direction[-1] != p['axis']:
        raise ValueError('외력 축과 탐색 단계가 다름')
    entry = p['baseline'].get(lane)
    if entry is None or entry['direction'] != direction:
        raise ValueError(f'보정표 없음: lane={lane}, direction={direction}')
    position = float(position_mm)
    if not math.isfinite(position):
        raise ValueError('보간 위치는 유한해야 함')
    nodes, values = p['nodes'], entry['values']
    if not nodes[0] <= position <= nodes[-1]:
        return None
    i = min(bisect_right(nodes, position)-1, len(nodes)-2)
    fraction = (position-nodes[i])/(nodes[i+1]-nodes[i])
    return values[i] + fraction*(values[i+1]-values[i])


baseline_fy = baseline_force  # 기존 Y 보간 함수 호출 호환


def backward_orientation(reference, direction, profile=None):
    """진행 방향 반대로 BACKWARD_TILT_DEG만큼 기울인 ZYZ 자세"""
    p = scan_settings(direction) if profile is None else profile
    tilt = p['tilt']
    if tilt == 0:
        return list(reference)
    r = zyz_rotation(reference)
    if r[2][2] > -0.98:
        raise RuntimeError('기울기 계산에는 툴 Z가 거의 아래를 향한 HOME 자세가 필요')
    angle = math.radians(tilt if direction in ('-Y', '+X') else -tilt)
    c, t = math.cos(angle), math.sin(angle)
    rx = ([[1, 0, 0], [0, c, -t], [0, t, c]] if scan_axis(direction)==1
          else [[c, 0, t], [0, 1, 0], [-t, 0, c]])
    q = [[sum(rx[i][k]*r[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    b = math.acos(max(-1.0, min(1.0, q[2][2])))
    if abs(math.sin(b)) < 1e-8:
        a = math.atan2(-q[0][1], -q[0][0]) if q[2][2] < 0 else math.atan2(q[1][0], q[0][0])
        return [math.degrees(a), math.degrees(b), 0.0]
    return [math.degrees(math.atan2(q[1][2], q[0][2])), math.degrees(b),
            math.degrees(math.atan2(q[2][1], -q[2][0]))]


def scan_orientation(reference, direction, profile=None):
    """단계별 후방 기울임과 툴 로컬 Z 회전. 집기 HOME 기준은 변경하지 않음."""
    p = scan_settings(direction) if profile is None else profile
    angles = backward_orientation(reference, direction, p)
    angles[2] += p['roll']
    return angles


def probe_direction(scan_direction, angle):
    """진행 방향 앞쪽 반원을 BASE XY 단위벡터로 변환한다."""
    axis = scan_axis(scan_direction)
    forward = [0.0, 0.0]
    forward[axis] = 1.0 if scan_direction[0]=='+' else -1.0
    radians = math.radians(angle)
    c,t = math.cos(radians),math.sin(radians)
    return forward[0]*c-forward[1]*t, forward[0]*t+forward[1]*c


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
SIM_PROBE_HALF_MM = 40.0    # 단순 sim_boxes 모델의 가상 윗면 반폭 (50mm 반원 탐침의 중심 추정용)
SIM_TOUCH_MM = 25.0         # 탐색 중 이 거리 안이면 접촉 (핑거 두께 포함)
SIM_CONTACT_FY = 4.0        # 가상 접촉 힘 [N]


class ProbeMiss(Exception):
    """정상적으로 내려갔지만 유효한 윗면 높이가 없음"""


class DifferentProbeObject(Exception):
    """유효한 접촉이지만 첫 윗면과 높이가 다름."""


class Recorder:
    """CSV 기록 (원본 그대로, 보정표 재측정용). csv_dir 파라미터를 줄 때만 만든다."""
    FIELDS = ('elapsed_s', 'segment_elapsed_s', 'actual_interval_s', 'lane', 'phase', 'direction',
              'x_mm', 'y_mm', 'z_mm', 'force_raw_n', 'force_baseline_n', 'force_corrected_n', 'motion_state',
              'detection_enabled', 'contact', 'speed_mm_s', 'acc_mm_s2', 'baseline_loaded', 'force_axis', 'scan_stage')

    def __init__(self, root):
        self.directory = Path(root).expanduser() / (
            'raster_y_then_x_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.rows, self.index = [], 0
        self.origin = self.segment_origin = self.previous = None

    def add(self, stamp, lane, phase, direction, pose, fy, baseline, residual, status, enabled, contact, speed, acceleration, force_axis='Fy', scan_stage='Y'):
        if self.origin is None:
            self.origin = stamp
        if self.segment_origin is None:
            self.segment_origin = stamp
        interval = '' if self.previous is None else stamp-self.previous
        self.rows.append((stamp-self.origin, stamp-self.segment_origin, interval, lane, phase, direction,
                          *pose[:3], fy, '' if baseline is None else baseline,
                          '' if residual is None else residual, status, int(enabled), int(contact),
                          speed, acceleration, int(baseline is not None), force_axis, scan_stage))
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
        self.log.info('[scan] Y 본탐색 → X 확인탐색. 단계·경로 커서는 Goal 사이에 유지.')

    # ---- 탐색 커서 (Goal 사이에 유지) ----
    def _reset_cursor(self):
        self.scan_pass = 0       # 0=Y 본탐색, 1=X 확인탐색
        self.lane = 0            # 현재 단계의 선 index
        self.resume = None       # 다음 Goal에서 이어 훑을 접촉 정보 (None이면 선 시작점부터)

    @property
    def settings(self):
        return SCAN_PASSES[self.scan_pass]

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
        self._last_miss = None
        r.movej(home, vel=ANGULAR_VEL, acc=ANGULAR_ACC)
        here = r.pose()
        reference, home_z = here[3:6], here[2]
        self._close()
        while self.scan_pass < len(SCAN_PASSES):
            p = self.settings
            axis = 0 if p['axis']=='X' else 1
            directions = (lane_direction(0,p), lane_direction(1,p))
            orient = {d: scan_orientation(reference,d,p) for d in directions}
            report.log(f"{p['axis']}축 탐색 {self.lane+1}/{len(p['lanes'])}선, v{p['vel']:g}/a{p['acc']:g}, 한계{p['limit']:g}N")
            if self.resume is not None:
                self._go_resume(self.resume, orient)
                ignore = RESUME_IGNORE_MM
            else:
                d = lane_direction(self.lane,p)
                xyz = scan_point(p,self.lane)
                safe_z = max(TRAVEL_Z, r.pose()[2])
                r.move_via_travel_z([xyz[0], xyz[1], safe_z, *orient[d]],
                                    vel=LINEAR_VEL, acc=LINEAR_ACC)
                self._move_z(p['z'])
                r.wait(SETTLE_SECONDS)
                ignore = 0.0
            while self.lane < len(p['lanes']):
                lane = self.lane+1
                d = lane_direction(self.lane,p)
                target = scan_point(p,self.lane,end=True)
                o = orient[d]
                self._orient(o, scanning=True)
                r.wait(p['wait'])
                while abs(r.pose()[axis]-target[axis]) > 1.0:
                    contact = self._scan_line([*target,*o],d,lane,ignore)
                    if contact is None:
                        break
                    sp = contact['stopped_pose']
                    report.log(f"물품 접촉: {p['axis']} {lane}번 {d}, X={sp[0]:.0f}, Y={sp[1]:.0f}, dF{p['axis']}={contact['residual_n']:+.2f}N")
                    self.resume = contact
                    report('picking')
                    grip = self._find_and_grip(contact,reference,report)
                    if grip is not None and self._carry_and_place(grip,zone,home_z,report):
                        r.move_to_waypoint(zone, orientation=reference,
                                           vel=LINEAR_VEL, acc=LINEAR_ACC,
                                           travel_vel=TRAVEL_SPEED_MM_S)
                        report.log('물품 1개 배치 완료, 측정존 WAYPOINT 복귀. 다음 요청에서 현재 단계·경로 재개')
                        return True
                    failure = '반원·직선 중심 탐지 실패' if grip is None else '이송 높이에서 파지 확인 실패'
                    if self._last_miss and self._last_miss[0]==lane and abs(self._last_miss[1]-sp[axis])<HEIGHT_MISS_SAME_SPOT_MM:
                        misses=self._last_miss[2]+1
                    else:
                        misses=1
                    self._last_miss=(lane,sp[axis],misses)
                    if misses>=HEIGHT_MISS_LIMIT:
                        self._on_scan_blocked(contact,misses,report,failure)
                    report.log(f'{failure} ({misses}/{HEIGHT_MISS_LIMIT}) → 탐색 계속','WARN')
                    report('space_scan')
                    self._go_resume(contact,orient)
                    ignore=RESUME_IGNORE_MM
                self.lane+=1
                self.resume=None
                ignore=0.0
                if self.lane<len(p['lanes']):
                    r.wait(p['wait'])
                    next_xyz=scan_point(p,self.lane)
                    previous_cross=p['lanes'][self.lane-1]
                    transfer_sign='+' if next_xyz[1-axis]>previous_cross else '-'
                    # 라인 간 이동은 작업공간을 훑는 연속 경로이므로 scan Z를 유지한다.
                    self._scan_line([*next_xyz,*o],transfer_sign+('Y' if axis==0 else 'X'),lane,phase='transfer')
                    r.wait(p['wait'])
            report.log(f"{p['axis']}축 전체 경로 완료" + (' → X축 마지막 확인 시작' if self.scan_pass==0 else ''))
            self.scan_pass+=1
            self.lane=0
            self.resume=None
            self._last_miss=None
        r.move_to_waypoint(zone, orientation=reference,
                           vel=LINEAR_VEL, acc=LINEAR_ACC,
                           travel_vel=TRAVEL_SPEED_MM_S)
        self._reset_cursor()
        report.log('Y 본탐색 및 X 확인탐색 완료: 물품 없음, 측정존 WAYPOINT 복귀')
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

    def _orient(self, abc, scanning=False):
        """제자리에서 자세만 바꾸기 (원본 set_scan_orientation)"""
        p = self.robot.pose()
        if rotation_error(p[3:6], abc) <= 1e-6:
            return
        vel = self.settings['tilt_vel'] if scanning else ANGULAR_VEL
        acc = self.settings['tilt_acc'] if scanning else ANGULAR_ACC
        self.robot.movel([*p[:3], *abc], vel=[5.0,vel], acc=[10.0,acc], tol=1.0)
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
        """현재 단계의 원래 선 시작점으로 상승 복귀한 뒤 같은 방향으로 재개한다."""
        lane,d = contact['lane'],contact['direction']
        p=self.settings
        if (contact.get('scan_pass',self.scan_pass)!=self.scan_pass or not 1<=lane<=len(p['lanes'])
                or d!=lane_direction(lane-1,p)):
            raise ValueError('재개할 단계·경로·방향 오류')
        xyz=scan_point(p,lane-1)
        safe_z=max(TRAVEL_Z, self.robot.pose()[2])
        self.robot.move_via_travel_z([xyz[0], xyz[1], safe_z, *orient[d]],
                                     vel=LINEAR_VEL, acc=LINEAR_ACC)
        self._move_z(p['z'])
        self.robot.wait(SETTLE_SECONDS)

    # ==================================================================
    # 탐색 한 줄 (원본 Robot.scan_line)
    # ==================================================================
    def _scan_line(self, target, direction, lane, ignore_mm=0.0, phase='scan'):
        """활성 축의 외력을 보정해 판정한다. 경로 연결 이동에는 판정을 적용하지 않는다."""
        r = self.robot
        p = self.settings
        axis = scan_axis(direction)
        start_position = r.pose()[axis]
        sign = 1.0 if direction[0] == '+' else -1.0
        scanning = phase == 'scan'
        speed, acceleration = (p['vel'],p['acc']) if scanning else (p['transfer_vel'],p['transfer_acc'])
        r.amovel(target, vel=[speed, 10.0], acc=[acceleration, 20.0])
        started = next_sample = time.monotonic()
        try:
            while True:
                status = r.motion_state()
                now = time.monotonic()
                if now >= next_sample:
                    fy = r.force('base')[axis]                  # 정지 확인 + 1초마다 진행 알림 포함
                    pose = r.pose()
                    offset = baseline_force(pose[axis],direction,lane,p) if scanning else None
                    if self.sim_boxes is not None:
                        fy = self._sim_fy(pose, offset)
                    residual = None if offset is None else fy - offset
                    enabled = status != 0 and residual is not None and sign*(pose[axis]-start_position) >= ignore_mm
                    contact = enabled and abs(residual) >= p['limits'][lane-1]
                    if self.recorder:
                        self.recorder.add(now, lane, phase, direction, pose, fy, offset, residual,
                                          status, enabled, contact, speed, acceleration, 'Fx' if axis==0 else 'Fy', p['axis'])
                    if contact:
                        if not r.move_stop(1):
                            raise RuntimeError('접촉 후 빠른 정지 요청 실패')
                        self._wait_still()
                        return {'lane': lane, 'direction': direction, 'scan_pass': self.scan_pass,
                                'force_axis': p['axis'], 'sample_pose': pose,
                                'stopped_pose': r.pose(), 'force_n': fy, 'baseline_n': offset,
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
        if direction not in ('-Y', '+Y', '-X', '+X'):
            raise ValueError(f'높이 탐침 방향 오류: {direction}')
        self._move_z(PROBE_START_Z)
        probe_orientation = list(reference)
        axis = scan_axis(direction)
        probe_orientation[2] += PROBE_TOOL_Z_ROTATION_DEG - (90.0 if axis==0 else 0.0)
        self._orient(probe_orientation)
        p=self.settings
        travel_min,travel_max=sorted((p['start'],p['end']))
        cross_min,cross_max=min(p['lanes']),max(p['lanes'])
        x_min,x_max=(travel_min,travel_max) if axis==0 else (cross_min,cross_max)
        y_min,y_max=(cross_min,cross_max) if axis==0 else (travel_min,travel_max)

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
        fan_misses = 0
        first_z = None
        try:
            fan_angles = (PROBE_FAN_ANGLES_DEG if direction in ('-Y','+X')
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
                    fan_misses += 1
                    continue
                if first_z is None:
                    first_z = top[2]
                angles.append(angle)
            if not angles:
                if fan_misses == len(PROBE_FAN_ANGLES_DEG):
                    self._move_z(PROBE_START_Z)
                    report.log(f'반원 {fan_misses}회 미검출 → 충돌 정지점 '
                               f'X={origin[0]:.1f}, Y={origin[1]:.1f}에서 바닥 기준 1회 집기', 'WARN')
                    return self._grip([origin[0], origin[1], None], reference, report,
                                      probe_orientation)
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
        ray_angle = math.degrees(math.atan2(ux, -uy)) - (90.0 if axis==0 else 0.0)
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
        top_z = top[2]
        collision_fallback = top_z is None
        floor_tip = TABLE_Z + SYNC_GRIP['tip_above_floor_mm']
        tip_z = floor_tip if collision_fallback else calculate_grip_z(top_z)
        on_floor = tip_z <= floor_tip
        tip_z = max(tip_z, floor_tip)
        sg = make_sync_gripper(r, tip_z - SYNC_GRIP['tip_above_floor_mm'],
                               log=lambda m: self.log.info(f'[scan] {m}'))
        retry_z = PROBE_START_Z if collision_fallback else max(PROBE_START_Z, top_z + GRIP_RETRY_LIFT_MM)
        self._move([top[0], top[1], retry_z, *r.pose()[3:6]])   # 높이 탐지 뒤 위로
        initial = list(grip_orientation)
        angles = (0.0,) if collision_fallback else GRIP_RETRY_ANGLES
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
                tip_offset = TCP['pos'][2] - tip_h_mm(raw_hold)
                if collision_fallback:
                    height_note = f'충돌 정지점 집기, 핑거 끝 Z={hold[2] + tip_offset:.1f} mm'
                else:
                    depth = top_z - (hold[2] + tip_offset)
                    note = ', 얇아서 작업대 바닥 + 2.6 기준' if on_floor else ''
                    height_note = f'핑거 끝 윗면 − {depth:.0f} mm{note}'
                r.wait(GRIPPER['settle_after_grip_s'])     # 40N이 다 걸린 뒤 들기
                p = hold[:]
                p[2] = hold[2] + min(GRIPPER['lift_slow_mm'], PICK_LIFT_MM)
                slow = GRIPPER['lift_slow_vel']
                r.movel(p, vel=[slow, 20.0], acc=[slow * 3, 40.0], tol=1.0)
                return {'grip_z': hold[2], 'top_z': top_z,
                        'transfer_alignment': list(reference),  # 성공 각도와 무관하게 HOME 기준 0도로 복원
                        'tip_off': tip_offset,
                        'grasp_summary': (f'{n}/{len(angles)}번째, {angle:+g}도, 폭 {width:.1f} mm, '
                                          f'{height_note}')}
            self._open()
            nxt = f'다음 {angles[n]:+g}도' if n < len(angles) else '포기'
            report.log(f'집기 실패 {n}/{len(angles)} ({angle:+g}도): {why} → {nxt}', 'WARN')
            self._move_z(retry_z)
        self._move_z(retry_z)
        self._orient(reference)   # 누적 회전 방지
        if collision_fallback:
            self._close()  # 탐색 재개 자세로 복원
            report.log(f'충돌 정지점 집기 실패: {why}', 'WARN')
            return None
        raise PartError('GRASP_FAILED', f'{len(angles)}회 모두 실패 ({why}). 그리퍼 연 채 높이 {retry_z:.0f}mm')

    # ==================================================================
    # 측정존 이동 + 내려놓기
    # ==================================================================
    def _carry_and_place(self, grip, zone, home_z, report):
        r = self.robot
        contact_z = zone[2] + (grip['grip_z'] - TABLE_Z)   # 예상: 박스 바닥이 측정존 바닥(z)에 닿는 TCP Z
        top_z = grip['top_z']
        clearance_z = PROBE_START_Z if top_z is None else top_z + PROBE_CLEARANCE_MM
        transfer_z = max(home_z, TRAVEL_Z, clearance_z,
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
            report.log('파지 각도와 무관하게 HOME 기준 0도 자세로 정렬 완료')
        transfer_pose = r.pose()
        transfer_pose[:2] = zone[:2]
        r.move_via_travel_z(transfer_pose, travel_z=TRAVEL_Z,
                            vel=LINEAR_VEL, acc=LINEAR_ACC,
                            travel_vel=TRAVEL_SPEED_MM_S)
        report('placing')
        # 파지는 _place_with_force에서 Force 하강 직전에 한 번 확인한다.
        # 핑거 끝이 측정존 바닥 아래로 가지 않도록 동기 집기의 오프셋을 반영한다.
        floor_tcp = zone[2] + PLACE_MIN_ABOVE_FLOOR_MM - grip.get('tip_off', 0.0)
        self._place_with_force(contact_z, floor_tcp, report)
        self._open()
        self._move_z(transfer_z)
        return True

    def _place_with_force(self, contact_z, floor_tcp, report):
        """배치 전 파지 1회 확인 → Force 접촉 → 하한/조기 접촉 확인 후 개방 허용.
        _move_z는 동기 이동, _force_down은 stop으로 정지 확인까지 하므로 중복 대기를 하지 않는다.
        """
        if not math.isfinite(contact_z) or not math.isfinite(floor_tcp):
            raise RuntimeError('배치 높이가 유효하지 않음; 파지 유지')
        upper_z = contact_z + PLACE_CONTACT_Z_TOL_MM
        stop_z = max(floor_tcp, contact_z - PLACE_MAX_BELOW_EXPECTED_MM)
        if stop_z >= upper_z:
            raise RuntimeError('배치 하한과 예상 접촉 구간이 맞지 않음; 파지 유지')

        self._move_z(contact_z + PLACE_SLOW_FROM_MM)
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
        stopped_z = stopped_pose[2]
        if status != 'contact':
            raise RuntimeError(f'배치 접촉 없음: 예상 Z={contact_z:.1f}, 정지 Z={stopped_z:.1f}; 파지 유지')
        detected_z = details['z']
        force_note = f", ΔFz={details['dfz']:.1f} N" if 'dfz' in details else ''
        note = f'예상 Z={contact_z:.1f}, 접촉 Z={detected_z:.1f}, 정지 Z={stopped_z:.1f}{force_note}'
        if not math.isfinite(detected_z) or not math.isfinite(stopped_z):
            raise RuntimeError('배치 접촉/정지 높이가 유효하지 않음; 파지 유지')
        if detected_z > upper_z:
            raise RuntimeError(f'배치 조기 접촉: {note}; 파지 유지')
        if min(detected_z, stopped_z) < floor_tcp - PLACE_STOP_Z_TOL_MM:
            raise RuntimeError(f'배치 바닥 보호 하한 초과: {note}; 파지 유지')
        # 접촉 후 하중이 바닥으로 옮겨지며 폭/잡힘 신호가 바뀔 수 있어 개방 직전 쥠 재검사는 생략.
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
        if abs(pose[2] - self.settings['z']) < 5.0 and self._sim_box_at(pose[0], pose[1], SIM_TOUCH_MM):
            fy += SIM_CONTACT_FY
        return fy

    def _sim_probe(self, floor_z):
        p = self.robot.pose()
        # sim_boxes는 실제 형상을 넣지 않는 간략 모델이다. 접촉점에서
        # PROBE_FAN_RADIUS_MM만큼 떨어진 위치를 찍는 중심 탐지 경로가
        # 가상 윗면에 닿도록, 시뮬레이션 탐침 판정만 별도 반폭을 사용한다.
        b = self._sim_box_at(p[0], p[1], SIM_PROBE_HALF_MM)
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


