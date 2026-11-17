#!/usr/bin/env python3
"""
place_algorithm.py — 적재 알고리즘 (담당: 정수현)
================================================================
이 파일과 config/place_config.json은 적재 담당자 소유다. 관제 관련 코드는 들어 있지 않다.
  - 로봇은 생성자로 받은 robot(box_comm.dsr_io.DsrIO)으로만 움직인다.
    robot.movel / movej / grip / force_z / compliance_on / force_on 안에서 관제 정지 확인과
    진행 알림(keepalive)이 자동으로 된다 → 여기서 report.check()·keepalive를 부를 필요 없음.
  - 실패하면 예외를 그냥 던진다. box_comm이 로봇을 멈추고(힘·순응제어 해제 + move_stop) Result로 바꾼다.
    적재 칸이 없을 때만 PartError('NO_SLOT')를 직접 쓴다. 나머지는 PLACE_FAILED로 보고된다.
  - 집기 확인 (11/17, 적재팀 재시도 로직을 Modbus 그리퍼로): 닫은 뒤 grip_close().grabbed로 잡혔는지,
    들어 올린 뒤 그리퍼 폭으로 계속 쥐고 있는지 확인. 못 잡았으면 같은 자세로 다시 (common.GRIP_RETRY = 3번,
    설정 grip_retry로 바꿀 수 있음). 다 실패하면 그리퍼 열고 이송 높이로 올라간 뒤 PartError('GRASP_FAILED')
    → 관제가 측정존을 비우고 공간 스캔부터 다시. 이송 중 놓쳤으면(적재 직전 빈손) PLACE_FAILED → 작업 종료
    잡는 힘은 설정 grip_force_n (없으면 common.GRIPPER 20 N)
  - TCP·툴은 box_comm이 Goal마다 run 전에 맞춰 준다.

약속 (box_comm/place_interface.py start_algorithm)
  PlaceAlgorithm(node, robot)
  place(result_class, poses, report) -> int
    result_class : 'A' ~ 'D' 또는 'U'(미분류)
    poses        : {'home_j': [6], 'measure_zone': [x, y, z], 'place_zones': {'A': [x, y], ...}} (DB 값)
    report       : report('picking' / 'moving' / 'placing'), report.log('메시지')
    반환         : 놓은 (자리, 층)으로 잰 수량. U는 0

v0.4 적재
  - 매 Goal마다 1자리부터 박스를 쥔 채 힘 제어로 내려가 닿은 TCP Z로 층 판단
  - 꽉 찼으면(max_layers 초과 높이) 놓지 않고 올라가 다음 자리, 모든 자리가 차면 NO_SLOT
  - 놓은 (자리, 층) → 수량 = (자리-1) * max_layers + 층  (common.count_from_position)
  - U(미분류)는 unclassified.release_pose로 이동 후 투하. 힘 제어/층 판단 없음, 0 반환

설정: box_place/config/place_config.json (설치 후 share/box_place/config/place_config.json)
  - 층 수·자리 수는 설정에 적지 않는다. box_comm/common.py STACK(levels / max_tiers)을 그대로 쓴다
    (관제·DB·UI가 같은 규칙으로 수량을 해석하는 공용 규칙. 바꿀 때는 관제와 합의 + 자리별 첫 접촉 Z 재측정)
  - slot_dx_mm: 다음 자리 위치 = 1자리 x + (자리-1) * slot_dx_mm, y는 같음
  - classes: 설정된 클래스만 적재 (지금 A/B/C. D는 설정 시 추가)

상태 파일 (~/.ros/place_real_v04_state.json, sim이면 place_sim_state.json)
  - 자리별 바닥 첫 접촉 Z 기준과 진행 중 작업(pending)을 저장
  - 작업 중 멈추면 pending이 남아 다음 Goal을 거부한다 (물건을 쥔 채 멈췄을 수 있으므로).
    그리퍼·적재 상태를 확인한 뒤 -p clear_pending:=true 로 한 번 실행하면 지워짐

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

from box_comm.common import (GRIP_RETRY, GRIPPER, PartError, RESULT_CLASSES, STACK, TCP, UNCLASSIFIED,
                             VALID_CLASSES, count_from_position)


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


def validate_config(c):
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
    for k in ('min_search_z', 'slot_dx_mm'):
        if type(c[k]) not in (int, float) or not math.isfinite(c[k]):
            raise ValueError(f'잘못된 {k}')
    if not c['min_search_z'] < c['travel_z']:
        raise ValueError('최저 Z는 이송 높이보다 낮아야 합니다')
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
        refs = spec['first_contact_z_mm']
        if len(refs) != c['slot_count']:
            raise ValueError(f"{key}: 자리별 첫 접촉 Z {c['slot_count']}개 필요")
        for z in refs:
            if z is not None and (type(z) not in (int, float) or not math.isfinite(z) or not c['min_search_z'] < z < c['travel_z']):
                raise ValueError(f'{key}: 잘못된 첫 접촉 Z')
        for z in refs:
            if z is not None and z + c['max_layers'] * h + 10.0 >= c['travel_z']:
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
        self.learn_empty = bool(node.get_parameter('learn_empty_slots').value)
        self.show_force = bool(node.get_parameter('show_force').value)
        self.sim_contact = bool(node.get_parameter('sim_contact').value)
        self._sim_stack = {}                # (클래스, 자리) → 가상으로 쌓인 수 (sim_contact 전용)
        self._home_initialized = False      # 첫 Goal에서 한 번만 HOME 자세로
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
                self.log.warn(f"[place] 이전 작업 미완료: {self.state['pending']} — 그리퍼·적재 상태 확인 후 "
                              f"-p clear_pending:=true 로 다시 실행해야 적재를 받습니다")
        u = self.c.get('unclassified') or {}
        if u.get('enabled', False):
            if UNCLASSIFIED not in RESULT_CLASSES:
                self.log.warn('[place] U 분기 준비됨: 공용 통신 계층(box_comm)이 U Goal을 허용하지 않습니다')
            self.log.info(f"[place] 미분류(U) 투하: {u['release_pose']}, 접근 Z {u['approach_z']}")
        else:
            self.log.warn('[place] 미분류(U) 투하 꺼짐 (place_config.json unclassified.enabled) — U Goal은 실패')
        self.log.info(f"[place] 적재 규칙 {self.c['max_layers']}층 x {self.c['slot_count']}자리, "
                      f"자리 간격 x {self.c['slot_dx_mm']:+.0f}mm, 클래스 {sorted(self.c['classes'])}")

    # ------------------------------------------------------------------
    # 동작 단위 (robot 명령을 설정값과 묶음)
    # ------------------------------------------------------------------
    def _move(self, pose):
        self.robot.movel(pose, vel=self.c['speed_mm_s'], acc=self.c['acc_mm_s2'])

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
        """지금 박스를 쥐고 있는지 (잡힘 비트 + 패드 사이 간격이 빈손 기준보다 큼)"""
        g = self.robot.gripper
        st = g.status()
        return bool(st['grip']) and g.calibrated(st['width_raw_mm']) > GRIPPER['empty_width_mm']

    def _ensure_holding(self, where):
        """이송 중 놓쳤는지 확인. 놓쳤으면 박스가 어디 떨어졌는지 모르므로 작업 종료(PLACE_FAILED)"""
        if not self._holding():
            raise PartError('PLACE_FAILED', f'{where}: 이송 중 박스를 놓침 (그리퍼 빈손) → 떨어진 위치 확인 필요')

    def go_home(self, home):
        self.robot.movej(home, vel=self.c['home_speed_deg_s'], acc=self.c['home_acc_deg_s2'])

    def _home_once(self, home):
        """첫 Goal에서만 HOME 자세로 (알려진 관절 자세에서 시작)"""
        if not self._home_initialized:
            self.go_home(home)
            self._home_initialized = True

    def pick_from_measure_zone(self, zone, report):
        """측정존에서 집기. 못 잡거나 들다가 놓치면 같은 자세로 다시 (최대 grip_retry번).
        다 실패하면 그리퍼 열고 이송 높이에서 GRASP_FAILED (관제가 측정존을 비우고 재스캔)."""
        pick = [*zone, *self.c['pick_orientation']]
        above = [pick[0], pick[1], self.c['travel_z'], *pick[3:]]
        tries = int(self.c.get('grip_retry') or GRIP_RETRY)
        self._raise()
        self._move(above)
        why = ''
        at_pick = False
        for attempt in range(1, tries + 1):
            self._pending(grasp_attempt=attempt)
            self._grip(True)                       # 같은 파지 위치에서 다시 열기 (적재팀 원래 방식)
            if not at_pick:
                self._move(pick)
                at_pick = True
            res = self._grip(False)
            if not res.grabbed:
                why = f'빈손으로 닫힘 (폭 {res.width_mm:.1f} mm)'
            else:
                self._move(above)
                at_pick = False
                if self._holding():
                    if attempt > 1:
                        report.log(f'집기 성공 ({attempt}/{tries}번째 시도, 폭 {res.width_mm:.1f} mm)')
                    return res
                why = '들어 올리는 중 놓침'
            nxt = f'재시도 {attempt + 1}/{tries}' if attempt < tries else '포기'
            report.log(f'집기 실패 {attempt}/{tries}: {why} → {nxt}', 'WARN')
        self._grip(True)                           # 최종 실패: 열고 이송 높이로 올라간 뒤 중단
        self._raise()
        self._pending_done()          # 아무것도 안 쥐고 끝남 → 다음 Goal(재스캔 뒤)을 막지 않게 미완료 기록 지움
        raise PartError('GRASP_FAILED', f'{tries}회 시도 모두 실패 ({why}). 그리퍼 열고 이송 높이에서 중단')

    def move_above(self, category, x, y):
        # 박스를 쥔 채라도 옆으로 가기 전에 항상 수직으로 올라간다.
        self._raise()
        pose = [x, y, self.c['travel_z'], *self.c['classes'][category]['orientation']]
        self._move(pose)
        return pose

    def release(self):
        self._grip(True)

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

    # ------------------------------------------------------------------
    # 힘 제어 접촉 탐색
    # ------------------------------------------------------------------
    def _descend(self, approach):
        c = self.c
        robot = self.robot
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
        next_display = 0.0
        while robot.ok() and time.monotonic() < settle_until:
            fz = robot.force_z()
            if abs(fz) >= c['abs_force_limit_n'] or abs(fz-baseline) >= c['abort_force_n']:
                raise RuntimeError(f'순응제어 중 외력 상한 초과: Fz={fz:.2f}')
            z = robot.pose()[2]
            now = time.monotonic()
            if self.show_force and now >= next_display:
                print(f'[COMPLIANCE ONLY] Z={z:.2f} mm | dZ={z-start_z:+.2f} mm | '
                      f'Fz={fz:.2f} N', flush=True)
                next_display = now + 0.5
            if abs(z-start_z) >= 2.0 or z <= c['min_search_z']:
                raise RuntimeError(f'순응제어 중 위치 변동 초과: Z={z:.2f}, dZ={z-start_z:+.2f}')
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
                robot.stop()                            # 힘·순응제어 해제 후 멈춤 확인
                time.sleep(0.2)
                z, fz = robot.pose()[2], robot.force_z()
                if abs(fz) >= c['abs_force_limit_n'] or abs(fz - baseline) >= c['abort_force_n']:
                    raise RuntimeError('정지 후 외력 상한 초과')
                if c['contact_sign'] * (fz - baseline) < c['contact_force_n']:
                    raise RuntimeError('정지 후 접촉이 유지되지 않음')
                if not c['min_search_z'] < z <= approach[2] + 2.0:
                    raise RuntimeError('접촉 Z 범위 오류')
                return z
            z = robot.pose()[2]
            if z <= c['min_search_z'] or z > start_z + 2.0:
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
        z = base + stacked * c['classes'][category]['height_mm']
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
        if not self.c['min_search_z'] < zone[2] < self.c['travel_z']:
            raise ValueError('측정존 TCP 파지 Z 범위 오류')
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
        if all(abs(v) < 1e-6 for v in zone) or not self.c['min_search_z'] < zone[2] < self.c['travel_z']:
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
            if z + self.c['max_layers'] * h + 10.0 >= self.c['travel_z']:
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
                raise ValueError('JSON과 저장 파일의 바닥 접촉 기준이 다릅니다')
            return checked(saved)
        if supplied is not None:
            return checked(supplied)
        if not self.learn_empty:
            raise ValueError(f'{category}/{tier}: 바닥 접촉 TCP Z가 없습니다. 실측값 입력 또는 빈 자리 학습 필요')
        return None

    def _remember_reference(self, category, tier, x, y, z):
        height = self.c['classes'][category]['height_mm']
        if z + self.c['max_layers'] * height + 10.0 >= self.c['travel_z']:
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
        self._home_once(home)
        self.pick_from_measure_zone(zone, report)
        self._pending(phase='moving')
        report('moving')
        current = self.robot.pose()                     # 옆으로 가기 전에 접근 높이까지 수직 상승
        if current[2] < approach_z:
            self._move([current[0], current[1], approach_z, *current[3:]])
        above = [release_pose[0], release_pose[1], approach_z, *release_pose[3:]]
        self._move(above)
        self._pending(phase='placing', release_pose=release_pose)
        report('placing')
        self._ensure_holding('미분류 투하 전')
        self._move(release_pose)                        # 위치 이동만 (힘 제어 하강 없음)
        self._pending(phase='releasing')
        self.release()
        self._pending(phase='released')
        self._move(above)
        self._pending(phase='released_and_retracted')
        self.go_home(home)
        message = (f'U 미분류 경사로 투하 동작 완료: TCP={release_pose}, '
                   '정상 A/B/C/D 재고 갱신 제외 (경사로 유입은 별도 센서로 확인하지 않음)')
        self.log.info(message)
        report.log(message)
        self._pending_done()
        return 0

    # ------------------------------------------------------------------
    # 적재 (Goal 하나)
    # ------------------------------------------------------------------
    def place(self, result_class, poses, report) -> int:
        c = self.c
        if self.state['pending']:
            raise ValueError('이전 작업 미완료: 파지 물품/실제 적재 상태 확인 후 '
                             'place_main을 -p clear_pending:=true 로 다시 실행하세요')
        if result_class == UNCLASSIFIED:
            return self._place_unclassified(poses, report)
        if result_class not in c['classes']:
            raise ValueError(f'{result_class}: place_config.json에 설정이 없습니다 '
                             f'(지원: {sorted(c["classes"])})')

        home, zone, xy = self._goal_poses(poses, result_class)
        references = [self._reference(result_class, tier, xy[0]+(tier-1)*c['slot_dx_mm'], xy[1])
                      for tier in range(1, c['slot_count']+1)]

        self._pending(**{'class': result_class, 'phase': 'picking'})
        report('picking')
        self._home_once(home)
        self.pick_from_measure_zone(zone, report)
        self._pending(phase='moving')
        report('moving')
        approach = self.move_above(result_class, xy[0], xy[1])
        report('placing')
        # v0.4: 매 Goal마다 1자리부터 확인 (이미 찼어도 실제로 닿아 봄)
        for tier in range(1, c['slot_count']+1):
            x, y = xy[0]+(tier-1)*c['slot_dx_mm'], xy[1]
            if tier > 1:
                approach = self.move_above(result_class, x, y)
            self._pending(phase='placing', tier=tier)
            self._ensure_holding(f'{result_class} {tier}번 자리 하강 전')
            reference = references[tier-1]
            if self.sim_contact:
                z = self._sim_descend(result_class, tier, approach, reference)
            else:
                z = self._descend(approach)
            if reference is None:
                # learn_empty_slots: 기준이 없는 자리는 비어 있다고 보고 이 Z를 기준으로 학습
                reference = z
                self._remember_reference(result_class, tier, x, y, z)
                report.log(f'{result_class}/{tier} 빈 자리 첫 접촉 TCP Z 학습: {z:.2f} mm')
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
            message = f'{result_class} {tier}번 자리 {level}층 적재 → 수량 {count} (TCP Z={z:.2f})'
            self.log.info(message)
            report.log(message)
            self._pending_done()
            return count
        raise PartError('NO_SLOT', f"{result_class} {c['slot_count']}자리 모두 {c['max_layers']}층 만재. 물품 파지 유지")
