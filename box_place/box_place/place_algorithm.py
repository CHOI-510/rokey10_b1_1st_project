#!/usr/bin/env python3
"""
place_algorithm.py — 적재 알고리즘 (담당: 정수현)
================================================================
이 파일과 config/place_config.json은 적재 담당자 소유다. 관제 관련 코드는 들어 있지 않다.
  - 로봇은 생성자로 받은 robot(box_comm.dsr_io.DsrIO)으로만 움직인다.
    robot.movel / movej / grip / force_z / compliance_on / force_on 안에서 관제 정지 확인과
    진행 알림(keepalive)이 자동으로 된다 → 여기서 report.check()·keepalive를 부를 필요 없음.
  - 실패하면 예외를 그냥 던진다. box_comm이 로봇을 멈추고(힘·순응제어 해제 + move_stop) Result로 바꾼다.
    구역이 꽉 차면 미분류로 보낸다(11/18). 미분류 투하가 꺼져 있을 때만 PartError('NO_SLOT'). 나머지는 PLACE_FAILED.
  - 집기 확인 (11/17, 적재팀 재시도 로직을 Modbus 그리퍼로): 닫은 뒤 grip_close().grabbed로 잡혔는지,
    들어 올린 뒤 그리퍼 폭으로 계속 쥐고 있는지 확인. 못 잡았으면 같은 자세로 다시 (common.GRIP_RETRY = 3번,
    설정 grip_retry로 바꿀 수 있음). 다 실패하면 그리퍼 열고 이송 높이로 올라간 뒤 PartError('GRASP_FAILED')
    → 관제가 측정존을 비우고 공간 스캔부터 다시. 이송 중 놓쳤으면(적재 직전 빈손) PLACE_FAILED → 작업 종료
    쥐는 힘은 설정 grip_force_n (없으면 common.GRIPPER 40 N). 닫기는 sync(10 N) 뒤 이 힘으로 다시 조임
  - TCP·툴은 box_comm이 Goal마다 run 전에 맞춰 준다.

약속 (box_comm/place_interface.py start_algorithm)
  PlaceAlgorithm(node, robot)
  place(result_class, poses, report) -> int
    result_class : 'A' ~ 'D' 또는 'U'(미분류)
    poses        : {'home_j': [6], 'measure_zone': [x, y, z], 'place_zones': {'A': [x, y], ...}} (DB 값)
    report       : report('picking' / 'moving' / 'placing'), report.log('메시지')
    반환         : 놓은 (자리, 층)으로 잰 수량. U는 0. 구역 만재로 미분류에 보냈으면 ('U', 그 구역 정원) (11/18)

v0.4 적재
  - 매 Goal마다 1자리부터 박스를 쥔 채 힘 제어로 내려가 닿은 TCP Z로 층 판단
  - 꽉 찼으면(max_layers 초과 높이) 놓지 않고 올라가 다음 자리, 모든 자리가 차면 미분류 투하 위치로 (11/18, 예전 NO_SLOT)
  - 놓은 (자리, 층) → 수량 = (자리-1) * max_layers + 층  (common.count_from_position)
  - U(미분류)는 unclassified.release_pose로 이동 후 투하. 힘 제어/층 판단 없음, 0 반환

설정: box_place/config/place_config.json (설치 후 share/box_place/config/place_config.json)
  - 측정존 집기 = sync 집기 (11/18 실물 시험 후, box_comm/sync_grip.py = 판별 rg2_sync_grip 그대로, 판별과 같은 설정)
    RG2는 벌릴수록 핑거 끝이 올라간다(완전 열림 ↔ 닫힘 약 39mm). TCP(208.4)는 '닫힌 핑거 끝'이라, 예전처럼
    TCP를 바닥 + 5에 두고 닫으면 A 박스 60mm 면에서 핑거 끝이 바닥 위 약 17mm → 20mm 박스의 위 3mm만 물었다
    (11/18 실물: 들다가 2번 놓침). 이제는 100mm로 열고 핑거 끝이 바닥 + common.SYNC_GRIP tip(2.6)에 오게
    내려가서, 닫는 동안 핑거 끝이 내려가는 만큼 TCP를 올려 그 높이를 유지한다 → 폭과 상관없이 박스 아래쪽을 잡음.
    판별이 측정존 중심에 정해진 위상으로 놓아 주므로 하강 중 얹힘 감시는 하지 않는다.
    판별 위상 정렬 (11/18 판별 f8fdd3c): L1(가장 긴 변) = 측정존 x, L2 = y, 가장 짧은 변 = 높이, 측정존 중앙.
    집은 폭이 L1·L2 중 어느 변인지 로그에 남기고, 둘 다 아니면 WARN (_side_note).
    닫는 힘은 SYNC_GRIP close_force_n(10 N)으로 닫고 grip_force_n(기본 common 40 N)으로 다시 조인다.
    다시 조인 뒤 common.GRIPPER settle_after_grip_s(0.5 s) 기다리고, 처음 lift_slow_mm(15)은 천천히 든다 (11/18:
    40 N이 다 걸리기 전에 들면 박스가 핑거 사이로 빠짐). 설정에 grip_settle_s / lift_slow_vel을 넣으면 그 값을 씀.
  - 집은 높이가 폭마다 달라서(TCP − 박스 바닥 = hold_off, A 60mm 면 약 −9mm) 집을 때마다 재서 쓴다.
    층 판단·첫 접촉 Z 기준은 TCP Z − hold_off(= 박스 바닥 Z, 로그의 '바닥 Z')
  - 첫 접촉 Z 기준 (11/18 sync 집기 기준으로 다시 정함): 누르며 내려갈 때 박스가 핑거 사이로 조금 밀려
    예전 값(A 30.19, B·C 30.0)보다 2~3mm 낮게 닿는다. A를 1자리 3층 + 2자리 1층 쌓아 본 바닥 Z
    (27.69 / 48.46 / 68.55 / 27.37)의 가운데로 A 28.1, 다른 구역은 예전 차이(−0.19)만큼 27.9.
    기준을 바꾸면 상태 파일(~/.ros/place_real_v04_state.json)의 같은 자리 기준도 지워야 한다 (다르면 시작 거부).
  - 잡힘 판정: box_comm.gripper.check_grip/holding — 잡힘 비트 + 폭이 박스 범위 안 + 들어 올린 뒤 폭이 안 변함
  - 층 수·자리 수는 설정에 적지 않는다. box_comm/common.py STACK(levels / max_tiers)을 그대로 쓴다
    (관제·DB·UI가 같은 규칙으로 수량을 해석하는 공용 규칙. 바꿀 때는 관제와 합의 + 자리별 첫 접촉 Z 재측정)
  - 자리 이동: A/B=[dx0, dy-45], C/D=[dx0, dy-30] (CLASS_LAYOUT 우선 적용)
  - A/B는 집기·적재를 90도 회전 전 자세로 복구. C/D/U는 현재 JSON 자세 유지
  - slot_dx_mm은 기존 JSON 호환을 위해 읽지만, 자리 계산에는 분류별 slot_offset_mm을 사용
  - classes: 설정된 클래스만 적재 (지금 A/B/C. D는 설정 시 추가)

상태 파일 (~/.ros/place_real_v04_state.json, sim이면 place_sim_state.json)
  - 자리별 바닥 첫 접촉 Z 기준과 진행 중 작업(pending)을 저장
  - 작업 중 멈추면 pending이 남는다 (물건을 쥔 채 멈췄을 수 있으므로). 11/18부터 다음 Goal에서
    그리퍼를 읽어 비어 있으면 자동으로 지우고 진행, 쥐고 있으면 PLACE_FAILED (_resolve_pending).
    상태 파일을 지울 필요 없음 (지우면 자리별 첫 접촉 Z 기준까지 사라짐). -p clear_pending:=true 도 그대로 동작

파라미터
  place_config      설정 파일 경로 (기본: 설치 폴더)
  place_state       상태 파일 경로 (빈 값 = 기본 경로)
  sim_contact       에뮬레이터 전용: 힘 제어 대신 가상 높이까지 movel (실제 로봇에서 켜지 말 것)
  learn_empty_slots 첫 접촉 Z가 없는 빈 자리를 실제로 닿아서 학습
  show_force        하강 중 힘 출력
  clear_pending     이전 미완료 기록 삭제
================================================================
"""
import json
import math
import time
from pathlib import Path

from box_comm.common import (GRIP_RETRY, GRIPPER, SYNC_GRIP, PartError, RESULT_CLASSES, STACK, TCP, TRAVEL_Z,
                             UNCLASSIFIED, VALID_CLASSES, count_from_position)
from box_comm.gripper import check_grip, holding
from box_comm.sync_grip import GripMissed, make_sync_gripper

TIP_MM = SYNC_GRIP['tip_above_floor_mm']   # 집는 동안 핑거 끝 높이 (판별과 같은 공용 값). 집은 TCP − 박스 바닥은 항상 이 이하
PICK_SLOW_FROM_MM = 15.0    # 집는 높이 이 위부터 천천히
PICK_SLOW_VEL = 30.0


def default_config_path() -> str:
    """설치 폴더(share/box_place/config) → 소스 폴더(box_place/config) 순서로 찾음"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory('box_place')) / 'config' / 'place_config.json'
        if p.exists():
            return str(p)
    except Exception:
        pass
    return str(Path(__file__).resolve().parent.parent / 'config' / 'place_config.json')


def save_state(path, state):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def finite_vector(values, length):
    return (values is not None and len(values) == length and
            all(type(v) in (int, float) and math.isfinite(v) for v in values))


# 이번 배치 변경값. 아래 항목은 읽어 온 JSON의 기존 값보다 우선한다.
# slot_offset_mm: 다음 자리까지의 [dx, dy], 첫 자리는 관제가 보낸 DB XY를 그대로 사용.
# pick_orientation은 분류별 판별존 집기 자세. U는 기존 공통 pick_orientation을 사용한다.
CLASS_LAYOUT = {
    'A': {'slot_offset_mm': [0.0, -45.0],
          'pick_orientation': [148.0, -180.0, 148.0],
          'orientation': [120.0, 180.0, 120.0]},
    'B': {'slot_offset_mm': [0.0, -45.0],
          'pick_orientation': [148.0, -180.0, 148.0],
          'orientation': [120.0, -180.0, 120.0]},
    'C': {'slot_offset_mm': [0.0, -30.0]},
    'D': {'slot_offset_mm': [0.0, -30.0]},
}


# U 투하 좌표는 읽어 온 JSON/내장 CONFIG의 이전 값보다 우선 적용한다.
UNCLASSIFIED_RELEASE_POSE = [793.0, -113.0, 186.0, 171.0, -117.0, -92.0]
UNCLASSIFIED_APPROACH_Z = 186.0
UNCLASSIFIED_MOVE_TOL_MM = 5.0  # U 이동의 축별 XYZ 도달 허용 오차


def apply_class_layout(c):
    c.setdefault('unclassified', {}).update({
        'release_pose': list(UNCLASSIFIED_RELEASE_POSE),
        'approach_z': UNCLASSIFIED_APPROACH_Z,
    })
    for category, fields in CLASS_LAYOUT.items():
        if category in c['classes']:
            c['classes'][category].update({key: list(value) for key, value in fields.items()})
    return c


def validate_config(c):
    c = apply_class_layout(c)
    c['travel_z'] = TRAVEL_Z             # 공통 작업 고도가 Place 설정값보다 우선
    for k in ('travel_z', 'search_timeout_s', 'contact_force_n', 'desired_force_n',
              'abort_force_n', 'abs_force_limit_n', 'speed_mm_s', 'acc_mm_s2',
              'gripper_wait_s', 'baseline_duration_s', 'baseline_max_span_n',
              'z_stiffness_n_m', 'layer_tolerance_mm', 'home_speed_deg_s', 'home_acc_deg_s2'):
        if type(c[k]) not in (int, float) or not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError(f'유한한 양수 필요: {k}')
    if not 0 < c['contact_force_n'] < c['desired_force_n'] < c['abort_force_n']:
        raise ValueError('contact_force_n < desired_force_n < abort_force_n 필요')
    if c['contact_sign'] not in (-1, 1):
        raise ValueError('contact_sign은 +1 또는 -1이어야 합니다')
    for old in ('pick_above_floor_mm', 'pick_tip_above_floor_mm', 'sync_grip_force_n'):
        if old in c:
            raise ValueError(f'{old}는 11/18부터 공용 설정(common.SYNC_GRIP, 판별과 같은 sync 집기)으로 옮겼습니다 '
                             f'→ place_config.json에서 지우세요')
    off = TIP_MM                       # 집은 TCP − 박스 바닥은 항상 핑거 끝 높이 이하 → 높이 여유 검사에 씀
    if c['classes'] and off >= min(spec['height_mm'] for spec in c['classes'].values()) - 5.0:
        raise ValueError('SYNC_GRIP tip_above_floor_mm가 가장 낮은 박스 높이 − 5 이상입니다')
    for k in ('min_search_z', 'slot_dx_mm'):
        if type(c[k]) not in (int, float) or not math.isfinite(c[k]):
            raise ValueError(f'잘못된 {k}')
    if not c['min_search_z'] < c['travel_z']:
        raise ValueError('최저 Z는 이송 높이보다 낮아야 합니다')
    c.setdefault('force_start_bottom_z', 85.0)
    c.setdefault('force_start_clearance_mm', 10.0)
    for key in ('force_start_bottom_z', 'force_start_clearance_mm'):
        if type(c[key]) not in (int, float) or not math.isfinite(c[key]):
            raise ValueError(f'유한한 수 필요: {key}')
    if not c['min_search_z'] < c['force_start_bottom_z'] < c['travel_z']:
        raise ValueError('힘제어 전환 바닥 Z는 하한과 이송 높이 사이여야 합니다')
    if c['force_start_clearance_mm'] <= 0:
        raise ValueError('힘제어 전환 여유는 양수여야 합니다')
    # 적재 규칙(층·자리 수)은 공용 STACK 하나가 기준 → 여기서 채워 넣는다 (설정 파일에는 적지 않음)
    # 예전 설정에 키가 남아 있으면 STACK과 같을 때만 허용 (다르면 어느 쪽이 맞는지 모호하므로 시작 안 함)
    for key, shared in (('max_layers', STACK['levels']), ('slot_count', STACK['max_tiers'])):
        if key in c and c[key] != shared:
            raise ValueError(f"place_config의 {key}={c[key]}가 공용 STACK({shared})과 다릅니다. "
                             f"설정에서 {key}를 지우세요 (적재 규칙은 box_comm/common.py STACK 하나로 관리)")
        c[key] = shared
    if not c['classes'] or not set(c['classes']) <= set(VALID_CLASSES):
        raise ValueError(f'classes는 {VALID_CLASSES} 중에서 설정하세요')
    if not finite_vector(c['pick_orientation'], 3):
        raise ValueError('파지 자세(pick_orientation)를 설정하세요')
    for key, spec in c['classes'].items():
        h = spec['height_mm']
        if type(h) not in (int, float) or not math.isfinite(h) or h <= 0 or c['layer_tolerance_mm'] >= h/2:
            raise ValueError(f'{key}: 높이/층 허용 오차 오류')
        if not finite_vector(spec['orientation'], 3):
            raise ValueError(f'{key}: 적재 자세 오류')
        if not finite_vector(spec.get('slot_offset_mm'), 2):
            raise ValueError(f'{key}: 자리 간격 [dx, dy] 오류')
        if not finite_vector(spec.get('pick_orientation', c['pick_orientation']), 3):
            raise ValueError(f'{key}: 분류별 파지 자세 오류')
        refs = spec['first_contact_z_mm']
        if len(refs) != c['slot_count']:
            raise ValueError(f"{key}: 자리별 첫 접촉 Z {c['slot_count']}개 필요")
        for z in refs:
            if z is not None and (type(z) not in (int, float) or not math.isfinite(z) or not c['min_search_z'] < z < c['travel_z']):
                raise ValueError(f'{key}: 잘못된 첫 접촉 Z')
        for z in refs:
            if z is not None and z + off + c['max_layers'] * h + 10.0 >= c['travel_z']:
                raise ValueError(f"{key}: {c['max_layers']}층 위 파지 접촉 높이와 상부 Z 여유 부족")
    if UNCLASSIFIED in c['classes']:
        raise ValueError('U는 일반 classes가 아니라 unclassified 항목으로 설정하세요')
    u = c.get('unclassified')
    if u is not None:
        if not isinstance(u, dict) or type(u.get('enabled', False)) is not bool:
            raise ValueError('unclassified.enabled는 true/false여야 합니다')
        if u.get('enabled', False):
            if not finite_vector(u.get('release_pose'), 6):
                raise ValueError('미분류 투하 TCP 좌표 6개가 필요합니다')
            approach_z = u.get('approach_z')
            if (type(approach_z) not in (int, float) or not math.isfinite(approach_z)
                    or approach_z < c['travel_z'] or approach_z < u['release_pose'][2]
                    or u['release_pose'][2] <= c['min_search_z']):
                raise ValueError('미분류 접근 Z는 이송 Z/투하 Z 이상, 투하 Z는 최저 Z보다 높아야 합니다')
    return c


def level_from_z(z, floor_z, height, tolerance):
    below = round((z - floor_z) / height)
    if below < 0 or abs(z - floor_z - below * height) > tolerance:
        raise ValueError(f'층 판단 오차: Z={z:.2f}, 첫 접촉 Z={floor_z:.2f}, 물품 높이={height:.2f}')
    return below + 1


class PlaceAlgorithm:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot
        node.declare_parameter('place_config', default_config_path())
        node.declare_parameter('place_state', '')   # 빈 값 = 실제용/시뮬용 기본 경로
        node.declare_parameter('sim_contact', False)
        node.declare_parameter('learn_empty_slots', False)
        node.declare_parameter('show_force', False)
        node.declare_parameter('clear_pending', False)
        config_path = Path(node.get_parameter('place_config').value).expanduser()
        self.log.info(f'[place] 설정 파일: {config_path}')
        self.c = validate_config(json.loads(config_path.read_text()))
        self.log.info(
            f"[PRE-DESCENT ENABLED] 기본 박스 바닥 Z={self.c['force_start_bottom_z']:.1f}mm, "
            f"만재 접촉 여유={self.c['force_start_clearance_mm']:.1f}mm → movel 후 힘제어"
        )
        self.learn_empty = bool(node.get_parameter('learn_empty_slots').value)
        self.show_force = bool(node.get_parameter('show_force').value)
        self.sim_contact = bool(node.get_parameter('sim_contact').value)
        self._sim_stack = {}                # (클래스, 자리) → 가상으로 쌓인 수 (sim_contact 전용)
        self._grip_width = None             # 집었을 때 폭 (놓침 판정 기준)
        self._hold_off = 0.0                # 집은 TCP Z − 박스 바닥 Z (sync 집기, 집을 때마다 잼)
        self._hold_why = ''
        state_name = 'place_sim_state.json' if self.sim_contact else 'place_real_v04_state.json'
        state_param = node.get_parameter('place_state').value
        self.state_path = Path(state_param or str(Path.home() / '.ros' / state_name)).expanduser()
        if self.sim_contact:
            self.log.warn('[place] sim_contact=true — 힘 센서 대신 가상 높이까지 내려감. '
                          '에뮬레이터 전용, 실제 로봇에서 쓰지 말 것')
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else {
            'version': 1, 'references': {k: [None] * self.c['slot_count'] for k in self.c['classes']},
            'pending': None}
        if self.state['version'] != 1:
            raise ValueError('지원하지 않는 기준 상태 파일입니다')
        for k in self.c['classes']:                 # 설정에 새로 추가된 클래스(예: D)도 기준 칸을 만듦
            self.state['references'].setdefault(k, [None] * self.c['slot_count'])
        if self.state['pending']:
            if node.get_parameter('clear_pending').value:
                self.log.warn(f"[place] 이전 미완료 작업 기록 삭제 (clear_pending): {self.state['pending']}")
                self.state['pending'] = None
                save_state(self.state_path, self.state)
            else:
                self.log.warn(f"[place] 이전 작업 미완료: {self.state['pending']} — 다음 Goal에서 그리퍼를 확인해 "
                              f"비어 있으면 자동으로 지우고 진행합니다 (쥐고 있으면 PLACE_FAILED)")
        u = self.c.get('unclassified') or {}
        if u.get('enabled', False):
            if UNCLASSIFIED not in RESULT_CLASSES:
                self.log.warn('[place] U 분기 준비됨: 공용 통신 계층(box_comm)이 U Goal을 허용하지 않습니다')
            self.log.info(f"[place] 미분류(U) 투하: {u['release_pose']}, 접근 Z {u['approach_z']}")
        else:
            self.log.warn('[place] 미분류(U) 투하 꺼짐 (place_config.json unclassified.enabled) — U Goal은 실패')
        self.log.info(f"[place] 적재 규칙 {self.c['max_layers']}층 x {self.c['slot_count']}자리")
        for category, spec in self.c['classes'].items():
            self.log.info(f"[LAYOUT] {category}: 자리 간격={spec['slot_offset_mm']}, "
                          f"집기={spec.get('pick_orientation', self.c['pick_orientation'])}, "
                          f"적재={spec['orientation']}")

    # ------------------------------------------------------------------
    # 동작 단위 (robot 명령을 설정값과 묶음)
    # ------------------------------------------------------------------
    def _move(self, pose, tol=2.0):
        self.robot.movel(pose, vel=self.c['speed_mm_s'], acc=self.c['acc_mm_s2'], tol=tol)

    def _raise(self):
        pose = self.robot.pose()
        if pose[2] < self.c['travel_z']:
            pose[2] = self.c['travel_z']
            self._move(pose)

    def _grip(self, opened):
        """열기/닫기 (RG2 Modbus). 닫기는 GripResult(grabbed, width_mm, raw_mm)를 돌려준다."""
        if opened:
            self.robot.grip_open()
            return None
        return self.robot.grip_close(self.c.get('grip_force_n') or GRIPPER['grip_force_n'])

    def _holding(self) -> bool:
        """지금 박스를 쥐고 있는지: 잡힘 비트 + 폭이 박스 범위 안 + 집었을 때 폭에서 안 변함 (11/18)"""
        ok, self._hold_why, _w = holding(self.robot.gripper, self._grip_width)
        return ok

    def _ensure_holding(self, where):
        """이송 중 놓쳤는지 확인. 놓쳤으면 박스가 어디 떨어졌는지 모르므로 작업 종료(PLACE_FAILED)"""
        if not self._holding():
            raise PartError('PLACE_FAILED', f'{where}: 이송 중 박스를 놓침 ({self._hold_why}) → 떨어진 위치 확인 필요')

    def go_home(self, home):
        self.robot.movej(home, vel=self.c['home_speed_deg_s'], acc=self.c['home_acc_deg_s2'])

    def _side_note(self, category, width):
        """집은 폭이 판별이 정렬해 둔 어느 변인지 (11/18 판별 위상 정렬: L1(가장 긴 변) = 측정존 x,
        L2(두 번째) = y, 가장 짧은 변 = 높이, 측정존 중앙). (설명, 맞는지). 크기를 모르면 ('', True)"""
        dims = (self.c['classes'].get(category) or {}).get('dimensions_mm')
        if not dims or len(dims) != 3:
            return '', True
        l1, l2 = sorted(dims, reverse=True)[:2]
        if abs(width - l1) <= 5.0:
            return f' = L1 {l1:g}mm 면(측정존 x)', True
        if abs(width - l2) <= 5.0:
            return f' = L2 {l2:g}mm 면(측정존 y)', True
        return f' — {category} 규격 L1 {l1:g}·L2 {l2:g}mm 어느 쪽과도 5mm 넘게 다름', False

    def pick_from_measure_zone(self, zone, report, category=None):
        """측정존에서 sync 집기 (11/18, 판별과 같은 공용 설정). 못 잡거나 들다가 놓치면 같은 자리·자세로 다시
        (최대 grip_retry번). 다 실패하면 그리퍼 열고 이송 높이에서 GRASP_FAILED (관제가 측정존을 비우고 재스캔)."""
        c = self.c
        spec = c['classes'].get(category, {})
        orient = list(spec.get('pick_orientation', c['pick_orientation']))
        report.log(f'[PICK ORIENTATION] {category or UNCLASSIFIED}: {orient}')
        above = [zone[0], zone[1], c['travel_z'], *orient]
        grip_force = c.get('grip_force_n') or GRIPPER['grip_force_n']
        sg = make_sync_gripper(self.robot, zone[2], log=lambda m: self.log.info(f'[place] {m}'))
        tries = int(c.get('grip_retry') or GRIP_RETRY)
        self._raise()
        self._move(above)
        why = ''
        gz = None
        for attempt in range(1, tries + 1):
            self._pending(grasp_attempt=attempt)
            raw_open = self.robot.grip_open()                 # 100mm로 열고 실제 열린 원시 폭
            self._grip_width = None
            gz = sg.grasp_z_for_open(raw_open)                # 이 폭에서 핑거 끝이 바닥 + tip이 되는 TCP Z
            self._move([zone[0], zone[1], gz + PICK_SLOW_FROM_MM, *orient])
            self.robot.movel([zone[0], zone[1], gz, *orient], vel=PICK_SLOW_VEL, acc=PICK_SLOW_VEL * 3)
            try:
                res = sg.close_with_lift(SYNC_GRIP['est_raw_mm'])
            except GripMissed as e:
                why = str(e)
            else:
                width = res['width_mm']
                ok, why = check_grip(width, 1)
                if ok and grip_force > SYNC_GRIP['close_force_n']:
                    tight = self.robot.grip_close(grip_force)  # 같은 자리에서 더 센 힘으로 다시 조임
                    ok, why = check_grip(tight.width_mm, 1, width) if tight.grabbed else (False, '다시 조이다 놓침')
                    width = tight.width_mm if tight.grabbed else width
                if ok:
                    self._grip_width = width
                    self._hold_off = self.robot.pose()[2] - zone[2]   # TCP − 박스 바닥 (층 판단 보정)
                    # 11/18: 40N이 다 걸리기 전에 들면 박스가 핑거 사이로 빠짐 → 잠깐 기다린 뒤 처음 구간은 천천히 들기
                    self.robot.wait(float(c.get('grip_settle_s', GRIPPER['settle_after_grip_s'])))
                    here = self.robot.pose()
                    slow_vel = float(c.get('lift_slow_vel', GRIPPER['lift_slow_vel']))
                    self.robot.movel([*here[:2], here[2] + GRIPPER['lift_slow_mm'], *orient],
                                     vel=slow_vel, acc=slow_vel * 3)
                    self._move(above)
                    if self._holding():
                        note, side_ok = self._side_note(category, width)
                        if not side_ok:
                            report.log(f'집은 폭 {width:.1f} mm{note} (판별 정렬·집기 자세 확인)', 'WARN')
                            note = ''
                        report.log(f'집기 성공 ({attempt}/{tries}번째, 폭 {width:.1f} mm{note}, '
                                   f'TCP−바닥 {self._hold_off:+.1f} mm)')
                        return res
                    why = f'들어 올리는 중 놓침: {self._hold_why}'
            nxt = f'재시도 {attempt + 1}/{tries}' if attempt < tries else '포기'
            report.log(f'집기 실패 {attempt}/{tries}: {why} → {nxt}', 'WARN')
            if attempt < tries:
                self.robot.grip_open()
                self._move([zone[0], zone[1], gz + PICK_SLOW_FROM_MM, *orient])
        self.robot.grip_open()                     # 최종 실패: 열고 이송 높이로 올라간 뒤 중단
        self._grip_width = None
        self._raise()
        self._pending_done()          # 아무것도 안 쥐고 끝남 → 다음 Goal(재스캔 뒤)을 막지 않게 미완료 기록 지움
        raise PartError('GRASP_FAILED', f'{tries}회 시도 모두 실패 ({why}). 그리퍼 열고 이송 높이에서 중단')

    def _slot_xy(self, category, tier, first_xy):
        dx, dy = self.c['classes'][category]['slot_offset_mm']
        return [first_xy[0] + (tier - 1) * dx,
                first_xy[1] + (tier - 1) * dy]

    def move_above(self, category, x, y):
        # 박스를 쥔 채라도 옆으로 가기 전에 항상 수직으로 올라간다.
        self._raise()
        pose = [x, y, self.c['travel_z'], *self.c['classes'][category]['orientation']]
        self._move(pose)
        return pose

    def _move_to_force_start(self, category, approach, reference, report):
        """박스 바닥 기준 전환 높이까지 직선 하강. 첫 접촉 기준은 기존 층 판정과 같은 기준.

        만재 자리도 매번 탐색하므로 reference + 3 * height 위에서 힘제어를 시작한다.
        기준 미학습 시에는 빠른 하강을 생략한다. 물체 위치/높이가 등록 범위라는 전제다.
        """
        if reference is None:
            report.log('[PRE-DESCENT SKIP] 첫 접촉 기준 미설정: 상부에서 바로 힘제어 시작', 'WARN')
            return list(approach)
        c = self.c
        if type(self._hold_off) not in (int, float) or not math.isfinite(self._hold_off):
            raise PartError('PLACE_FAILED', '파지 TCP 높이 보정값 오류')
        full_contact_bottom_z = reference + c['max_layers'] * c['classes'][category]['height_mm']
        bottom_z = max(c['force_start_bottom_z'],
                       full_contact_bottom_z + c['force_start_clearance_mm'])
        tcp_z = bottom_z + self._hold_off
        actual = self.robot.pose()
        # 움직일 여유가 없으면 이송 높이에서 시작한다. 만재 접촉 높이 자체가
        # 현재 높이 이상이면 직선 이동/힘제어를 보내지 않는다.
        if actual[2] - self._hold_off <= full_contact_bottom_z:
            raise PartError('PLACE_FAILED', '상부 높이가 예상 만재 접촉 높이보다 낮습니다')
        if tcp_z >= actual[2]:
            report.log('[PRE-DESCENT SKIP] 빠른 하강 여유 없음: 현재 상부에서 힘제어 시작')
            return [*approach[:2], actual[2], *approach[3:]]
        self._ensure_holding(f'{category} 힘제어 전환 높이 이동 전')
        target = [*approach[:2], tcp_z, *approach[3:]]
        report.log(f'[PRE-DESCENT] {category}: movel → TCP Z={tcp_z:.2f} '
                   f'(박스 바닥 Z={bottom_z:.2f}), 이후 힘제어')
        self._move(target)
        return target

    def release(self):
        self._grip(True)
        self._grip_width = None

    # ------------------------------------------------------------------
    # 진행 중 작업 기록 (멈췄을 때 다음 Goal을 막기 위함)
    # ------------------------------------------------------------------
    def _pending(self, **fields):
        if self.state['pending'] is None:
            self.state['pending'] = {}
        self.state['pending'].update(fields)
        save_state(self.state_path, self.state)

    def _pending_done(self):
        self.state['pending'] = None
        save_state(self.state_path, self.state)

    def _resolve_pending(self, report):
        """이전 Goal이 중간에 멈춘 기록(pending)이 있을 때 (11/18 관제 추가 — 상태 파일을 rm으로 지우지 않아도 되게).
        적재는 매 Goal마다 1자리부터 실제로 닿아 보고 수량을 재므로(재고 숫자를 믿지 않음),
        미완료 기록이 막아야 하는 경우는 '그리퍼가 아직 무언가를 쥐고 있을 때'뿐이다.
          빈 그리퍼 → 기록만 지우고 이번 Goal 진행 (WARN 로그. 자리별 첫 접촉 Z 기준값은 그대로 유지)
          쥐고 있음 → PLACE_FAILED (사람이 빼야 함). 빼고 나면 다음 Goal에서 다시 확인해 자동으로 풀림
        -p clear_pending:=true 로 다시 실행하는 방법도 그대로 동작한다."""
        pending = self.state['pending']
        if self.robot is None:
            raise PartError('PLACE_FAILED', f'이전 작업 미완료 {pending} — 로봇 없이 그리퍼를 확인할 수 없음')
        try:
            held, _why, width = holding(self.robot.gripper, None, check_range=False)
        except Exception as e:
            raise PartError('PLACE_FAILED', f'이전 작업 미완료 {pending} — 그리퍼 상태 읽기 실패: {e}')
        if held:
            raise PartError('PLACE_FAILED', f'이전 작업 미완료 {pending} — 그리퍼가 아직 무언가를 쥐고 있음 '
                                            f'(폭 {width:.1f} mm). 사람이 빼고 다시 시작하세요')
        message = f'이전 작업 미완료 기록 {pending} → 그리퍼 비어 있음 확인, 기록 지우고 진행'
        self.log.warn(f'[place] {message}')
        report.log(message, 'WARN')
        self._pending_done()

    # ------------------------------------------------------------------
    # 힘 제어 접촉 탐색
    # ------------------------------------------------------------------
    def _descend(self, approach):
        c = self.c
        robot = self.robot
        lo = c['min_search_z'] + self._hold_off      # 하한은 박스 바닥 기준 (sync 집기는 TCP가 박스 바닥보다 아래일 수 있음)
        time.sleep(0.5)
        samples = []
        deadline = time.monotonic() + c['baseline_duration_s']
        while time.monotonic() < deadline:
            fz = robot.force_z()
            if abs(fz) >= c['abs_force_limit_n']:
                raise RuntimeError('기준 외력 상한 초과')
            samples.append(fz)
            time.sleep(0.05)
        if len(samples) < 5 or max(samples) - min(samples) > c['baseline_max_span_n']:
            raise RuntimeError('기준 외력 불안정')
        baseline = sum(samples) / len(samples)
        start_z = robot.pose()[2]
        if self.show_force:
            print(f'[BEFORE] Z={start_z:.2f} mm | baseline Fz={baseline:.2f} N', flush=True)
        robot.compliance_on([3000., 3000., c['z_stiffness_n_m'], 200., 200., 200.])
        # stack_test 실험에서 성공한 순서: 순응제어만 2초 → 목표 힘
        settle_until = time.monotonic() + 2.0
        while robot.ok() and time.monotonic() < settle_until:
            fz = robot.force_z()
            if abs(fz) >= c['abs_force_limit_n'] or abs(fz-baseline) >= c['abort_force_n']:
                raise RuntimeError(f'순응제어 중 외력 상한 초과: Fz={fz:.2f}')
            z = robot.pose()[2]
            if abs(z-start_z) >= 3.0 or z <= lo:
                raise RuntimeError(
                    f'순응제어 중 위치 변동 초과: 시작Z={start_z:.2f}, '
                    f'현재Z={z:.2f}, dZ={z-start_z:+.2f}mm, 상한=3.00mm, '
                    f'Fz={fz:.2f}N, baseline={baseline:.2f}N, dFz={fz-baseline:+.2f}N, '
                    f'TCP 하한={lo:.2f}mm (목표 힘 적용 전)'
                )
            time.sleep(0.05)
        if not robot.ok():
            raise RuntimeError('목표 힘 적용 전 ROS 종료')
        if self.show_force:
            print(f"[FORCE APPLY] BASE Z force={-c['desired_force_n']:.2f} N, REL mode", flush=True)
        robot.force_on([0., 0., -c['desired_force_n'], 0., 0., 0.], [0, 0, 1, 0, 0, 0], 0.5)
        force_started = time.monotonic()
        deadline = force_started + c['search_timeout_s']
        next_display = 0.0
        while robot.ok():
            fz = robot.force_z()
            change = fz - baseline
            if abs(fz) >= c['abs_force_limit_n'] or abs(change) >= c['abort_force_n']:
                raise RuntimeError(f'외력 상한 초과: Fz={fz:.2f}, delta={change:.2f}')
            if c['contact_sign'] * change >= c['contact_force_n']:
                contact_fz, contact_delta = fz, change
                robot.stop()                            # 추가 조회/로그보다 정지를 먼저 수행
                time.sleep(0.2)
                z, fz = robot.pose()[2], robot.force_z()
                delta_after = fz - baseline
                abs_limit_hit = abs(fz) >= c['abs_force_limit_n']
                delta_limit_hit = abs(delta_after) >= c['abort_force_n']
                detail = (
                    f'Z={z:.2f}mm, baseline={baseline:.2f}N, '
                    f'정지전 Fz={contact_fz:.2f}N/dFz={contact_delta:.2f}N, '
                    f'정지후 Fz={fz:.2f}N/dFz={delta_after:.2f}N, '
                    f'절대상한={c["abs_force_limit_n"]:.2f}N(초과={abs_limit_hit}), '
                    f'변화량상한={c["abort_force_n"]:.2f}N(초과={delta_limit_hit})'
                )
                self.log.info(f'[CONTACT AFTER STOP] {detail}')
                if abs_limit_hit or delta_limit_hit:
                    raise RuntimeError(f'정지 후 외력 상한 초과: {detail}')
                if c['contact_sign'] * (fz - baseline) < c['contact_force_n']:
                    raise RuntimeError('정지 후 접촉이 유지되지 않음')
                if not lo < z <= approach[2] + 2.0:
                    raise RuntimeError('접촉 Z 범위 오류')
                return z
            z = robot.pose()[2]
            if z <= lo or z > start_z + 2.0:
                raise RuntimeError('하강 Z 범위 초과')
            now = time.monotonic()
            if self.show_force and now >= next_display:
                print(f'[FORCE ON] t={now-force_started:.1f}s | Z={z:.2f} mm | '
                      f'Fz={fz:.2f} N | delta={c["contact_sign"]*change:.2f} N', flush=True)
                next_display = now + 1.0
            if now >= deadline:
                raise RuntimeError(f'접촉 탐색 시간 초과: Z={z:.2f}, Fz={fz:.2f}, delta={change:.2f}')
            time.sleep(0.05)
        raise RuntimeError('접촉 탐색 중 ROS 종료')

    def _sim_descend(self, category, tier, approach, reference):
        """sim_contact: 가상으로 쌓인 수만큼 높은 곳까지 movel로 내려가 그 Z를 '첫 접촉 Z'로 돌려줌"""
        c = self.c
        base = reference if reference is not None else c['min_search_z'] + 10.0
        stacked = min(self._sim_stack.get((category, tier), 0), c['max_layers'])
        z = base + stacked * c['classes'][category]['height_mm'] + self._hold_off   # 박스 바닥 기준 → TCP Z
        self._move([approach[0], approach[1], z, *approach[3:]])
        self.log.info(f'[place] (sim) {category}/{tier} 가상 접촉 Z={z:.2f} (이미 {stacked}개)')
        return z

    # ------------------------------------------------------------------
    # 좌표·기준
    # ------------------------------------------------------------------
    def _goal_poses(self, poses, category):
        home = poses.get('home_j')
        zone = poses.get('measure_zone')
        xy = (poses.get('place_zones') or {}).get(category)
        if not finite_vector(home, 6) or not finite_vector(zone, 3) or not finite_vector(xy, 2):
            raise ValueError('Goal의 HOME/측정존/해당 분류 적재 구역 좌표가 없습니다')
        if all(abs(v) < 1e-6 for v in zone):
            raise ValueError('측정존 (0,0,0)은 사용할 수 없습니다')
        if not self.c['min_search_z'] < zone[2] + TIP_MM < self.c['travel_z']:
            raise ValueError('측정존 바닥 z 범위 오류 (최저 Z ~ 이송 높이 사이여야 함)')
        return list(home), list(zone), list(xy)

    def _unclassified_poses(self, poses):
        """U 분기는 일반 place_zones나 바닥 접촉 기준을 사용하지 않는다."""
        u = self.c.get('unclassified', {})
        if not u.get('enabled', False):
            raise ValueError('U 미분류 투하가 비활성입니다. unclassified 설정을 확인하세요')
        home = poses.get('home_j')
        zone = poses.get('measure_zone')
        if not finite_vector(home, 6) or not finite_vector(zone, 3):
            raise ValueError('U 작업의 HOME/측정존 좌표가 없습니다')
        if all(abs(v) < 1e-6 for v in zone) or not self.c['min_search_z'] < zone[2] + TIP_MM < self.c['travel_z']:
            raise ValueError('U 작업의 측정존 TCP 파지 좌표가 잘못되었습니다')
        return list(home), list(zone), list(u['release_pose']), u['approach_z']

    def _reference_signature(self, category, x, y):
        return {'xy': [x, y], 'orientation': self.c['classes'][category]['orientation'],
                'height_mm': self.c['classes'][category]['height_mm'],
                'tcp': {'name': TCP['name'], 'pos': list(TCP['pos'])}}

    def _reference(self, category, tier, x, y):
        def checked(z):
            h = self.c['classes'][category]['height_mm']
            if type(z) not in (int, float) or not math.isfinite(z) or not self.c['min_search_z'] < z:
                raise ValueError('잘못된 저장 접촉 기준 Z')
            if z + TIP_MM + self.c['max_layers'] * h + 10.0 >= self.c['travel_z']:
                raise ValueError(f"{self.c['max_layers']}층 위 접촉 높이에 비해 이송 Z 여유가 부족합니다")
            return z
        record = self.state['references'][category][tier-1]
        signature = self._reference_signature(category, x, y)
        supplied = self.c['classes'][category]['first_contact_z_mm'][tier-1]
        if record is not None:
            if record['signature'] != signature:
                raise ValueError('저장된 바닥 접촉 기준의 위치/자세/물건 높이/TCP가 현재 설정과 다릅니다')
            saved = record['first_contact_z_mm']
            if supplied is not None and abs(saved-supplied) > 0.01:
                raise ValueError(f'{category}/{tier}번 자리 바닥 접촉 기준이 설정(place_config.json {supplied:.2f})과 '
                                 f'상태 파일({self.state_path} {saved:.2f})에서 다릅니다 → 적재 구역이 비어 있으면 '
                                 f'상태 파일을 지우고 다시 실행 (rm {self.state_path})')
            return checked(saved)
        if supplied is not None:
            return checked(supplied)
        if not self.learn_empty:
            raise ValueError(f'{category}/{tier}: 바닥 접촉 TCP Z가 없습니다. 실측값 입력 또는 빈 자리 학습 필요')
        return None

    def _remember_reference(self, category, tier, x, y, z):
        height = self.c['classes'][category]['height_mm']
        if z + TIP_MM + self.c['max_layers'] * height + 10.0 >= self.c['travel_z']:
            raise ValueError('학습한 접촉 Z 기준으로 최상층 위 탐색 시 상부 Z 여유가 부족합니다')
        self.state['references'][category][tier-1] = {
            'signature': self._reference_signature(category, x, y), 'first_contact_z_mm': z}
        save_state(self.state_path, self.state)

    # ------------------------------------------------------------------
    # 미분류 U: 힘 제어 없이 투하 위치에서 개방
    # ------------------------------------------------------------------
    def _place_unclassified(self, poses, report):
        home, zone, release_pose, approach_z = self._unclassified_poses(poses)
        self._pending(**{'class': UNCLASSIFIED, 'route': 'unclassified', 'phase': 'picking'})
        report('picking')
        self.robot.move_to_waypoint(zone, vel=self.c['speed_mm_s'], acc=self.c['acc_mm_s2'])
        self.pick_from_measure_zone(zone, report)
        self._pending(phase='moving')
        report('moving')
        self._drop_unclassified(home, release_pose, approach_z, report)
        message = (f'U 미분류 경사로 투하 동작 완료: TCP={release_pose}, '
                   '정상 A/B/C/D 재고 갱신 제외 (경사로 유입은 별도 센서로 확인하지 않음)')
        self.log.info(message)
        report.log(message)
        self._pending_done()
        return 0

    def _drop_unclassified(self, home, release_pose, approach_z, report):
        """쥐고 있는 박스를 미분류 투하 위치에 놓고 HOME으로 (지금 위치에서 출발).
        미분류 Goal과 '구역 만재 → 미분류로 보내기'(11/18)가 같이 쓴다."""
        current = self.robot.pose()                     # 옆으로 가기 전에 접근 높이까지 수직 상승
        if current[2] < approach_z:
            self._move([current[0], current[1], approach_z, *current[3:]], tol=UNCLASSIFIED_MOVE_TOL_MM)
        above = [release_pose[0], release_pose[1], approach_z, *release_pose[3:]]
        self._move(above, tol=UNCLASSIFIED_MOVE_TOL_MM)
        self._pending(phase='placing', release_pose=release_pose)
        report('placing')
        self._ensure_holding('미분류 투하 전')
        if release_pose != above:
            self._move(release_pose, tol=UNCLASSIFIED_MOVE_TOL_MM)                    # 위치 이동만 (힘 제어 하강 없음)
        self._pending(phase='releasing')
        self.release()
        self._pending(phase='released')
        if release_pose != above:
            self._move(above, tol=UNCLASSIFIED_MOVE_TOL_MM)
        self._pending(phase='released_and_retracted')
        self.go_home(home)

    # ------------------------------------------------------------------
    # 적재 (Goal 하나)
    # ------------------------------------------------------------------
    def place(self, result_class, poses, report):
        """수량(int)을 반환. 구역이 꽉 차서 미분류로 보냈으면 ('U', 그 구역 수량) (11/18)"""
        c = self.c
        if self.state['pending']:
            self._resolve_pending(report)
        if result_class == UNCLASSIFIED:
            return self._place_unclassified(poses, report)
        if result_class not in c['classes']:
            raise ValueError(f'{result_class}: place_config.json에 설정이 없습니다 '
                             f'(지원: {sorted(c["classes"])})')

        home, zone, xy = self._goal_poses(poses, result_class)
        references = [self._reference(result_class, tier, *self._slot_xy(result_class, tier, xy))
                      for tier in range(1, c['slot_count']+1)]

        self._pending(**{'class': result_class, 'phase': 'picking'})
        report('picking')
        self.robot.move_to_waypoint(zone, vel=self.c['speed_mm_s'], acc=self.c['acc_mm_s2'])
        self.pick_from_measure_zone(zone, report, result_class)
        self._pending(phase='moving')
        report('moving')
        approach = self.move_above(result_class, xy[0], xy[1])
        report('placing')
        # v0.4: 매 Goal마다 1자리부터 확인 (이미 찼어도 실제로 닿아 봄)
        for tier in range(1, c['slot_count']+1):
            x, y = self._slot_xy(result_class, tier, xy)
            if tier > 1:
                approach = self.move_above(result_class, x, y)
            self._pending(phase='placing', tier=tier)
            self._ensure_holding(f'{result_class} {tier}번 자리 하강 전')
            reference = references[tier-1]
            approach = self._move_to_force_start(result_class, approach, reference, report)
            if self.sim_contact:
                z_tcp = self._sim_descend(result_class, tier, approach, reference)
            else:
                z_tcp = self._descend(approach)
            z = z_tcp - self._hold_off                  # 박스 바닥 Z (바닥에서 집었을 때와 같은 기준 = 첫 접촉 Z 기준)
            if reference is None:
                # learn_empty_slots: 기준이 없는 자리는 비어 있다고 보고 이 Z를 기준으로 학습
                reference = z
                self._remember_reference(result_class, tier, x, y, z)
                report.log(f'{result_class}/{tier} 빈 자리 첫 접촉 바닥 Z 학습: {z:.2f} mm')
            level = level_from_z(z, reference, c['classes'][result_class]['height_mm'],
                                 c['layer_tolerance_mm'])
            if level > c['max_layers']:
                message = f"{result_class}/{tier} {c['max_layers']}층 만재 → 파지 유지 후 상승"
                self.log.info(message)
                report.log(message)
                self._raise()
                continue
            self._pending(phase='releasing', contact_z_mm=z, level=level)
            self.release()
            self._raise()
            self._pending(phase='released_and_retracted')
            self.go_home(home)
            count = count_from_position(tier, level)
            if self.sim_contact:
                self._sim_stack[(result_class, tier)] = level
            self._remember_reference(result_class, tier, x, y, reference)   # 입력한 기준도 저장
            message = (f'{result_class} {tier}번 자리 {level}층 적재 → 수량 {count} '
                       f'(TCP Z={z_tcp:.2f}, 바닥 Z={z:.2f}, 기준 {reference:.2f})')
            self.log.info(message)
            report.log(message)
            self._pending_done()
            return count
        # 모든 자리 만재 (11/18 관제): 쥔 채 끝내지 않고 미분류 투하 위치로 보낸다 → (U, 정원) 반환
        #   관제: 재고 result_class = 정원(꽉 참을 실제로 확인한 값), U +1, 로그 ZONE_FULL. 작업은 계속.
        #   미분류 투하가 꺼져 있으면 예전처럼 쥔 채 NO_SLOT (작업 종료, 사람이 빼야 함)
        full = count_from_position(c['slot_count'], c['max_layers'])
        if not (c.get('unclassified') or {}).get('enabled', False):
            raise PartError('NO_SLOT', f"{result_class} {c['slot_count']}자리 모두 {c['max_layers']}층 만재. "
                                       f"미분류 투하 꺼짐 → 물품 파지 유지")
        _h, _z, release_pose, approach_z = self._unclassified_poses(poses)
        message = (f"{result_class} {c['slot_count']}자리 모두 {c['max_layers']}층 만재({full}개) "
                   f"→ 미분류 투하 위치로 보냄")
        self.log.warn(f'[place] {message}')
        report.log(message, 'WARN')
        self._pending(route='unclassified (zone full)')
        self._drop_unclassified(home, release_pose, approach_z, report)
        self._pending_done()
        return UNCLASSIFIED, full
