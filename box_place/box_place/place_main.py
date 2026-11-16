#!/usr/bin/env python3
"""
place_main.py — 적재 실제 로봇 제어 (담당: 정수현, 관제 통합 정리)
================================================================
v0.4 적재 약속 구현: run(result_class, report) -> 수량(int)
  - 매 Goal마다 1자리부터 박스를 쥔 채 힘 제어로 내려가 닿은 TCP Z로 층 판단
  - 꽉 찼으면(max_layers 초과 높이) 놓지 않고 올라가 다음 자리, 모든 자리가 차면 NO_SLOT
  - 놓은 (자리, 층) → 수량 = (자리-1) * max_layers + 층

설정: box_place/config/place_config.json (설치 후 share/box_place/config/place_config.json)
  - max_layers / slot_count 는 공용 box_comm/common.py STACK(levels / max_tiers)과 같아야 함
    (다르면 시작하지 않음 — 관제·DB·UI가 같은 규칙으로 수량을 해석하기 때문)
  - slot_dx_mm: 다음 자리 위치 = 1자리 x + (자리-1) * slot_dx_mm, y는 같음 (현재 −x 방향)
  - classes: 설정된 클래스만 적재 (지금 A/B/C. D·미분류는 설정이 생기면 추가)

정지 (관제 비상정지·로봇 이상·진행 감시)
  - 모든 이동 전과 힘 제어 하강 루프 안에서 report.check() → 관제가 취소하면 CANCELED로 빠져나옴
    (빠져나올 때 기존 _stop(emergency=True)로 힘·순응제어 해제 + move_stop)
  - 하강처럼 오래 걸리는 구간은 1초마다 report.keepalive() → 관제 진행 감시가 멈춘 것으로 오인하지 않음

상태 파일 (~/.ros/place_real_v04_state.json)
  - 자리별 바닥 첫 접촉 Z 기준과 진행 중 작업(pending)을 저장
  - 작업 중 멈추면 pending이 남아 다음 Goal을 거부한다 (물건을 쥔 채 멈췄을 수 있으므로).
    그리퍼·적재 상태를 사람이 확인한 뒤 -p clear_pending:=true 로 한 번 실행하면 지워짐

실행
  ros2 run box_place place_main
  ros2 run box_place place_main --ros-args -p clear_pending:=true      # 멈춘 뒤 확인 완료
  ros2 run box_place place_main --ros-args -p show_force:=true         # 하강 중 힘 출력
  ros2 run box_place place_main --ros-args -p place_config:=/경로/place_config.json
  ros2 run box_place place_main --ros-args -p sim_contact:=true        # 에뮬레이터 시험 (아래)

sim_contact (에뮬레이터 전용, 기본 false)
  에뮬레이터는 힘 센서 값이 없어 '닿을 때까지 하강'이 끝나지 않는다.
  true면 힘 제어 대신 '가상으로 쌓인 높이'까지 movel로 내려가 닿았다고 친다.
    가상 적재 수는 노드가 켜질 때 0에서 시작 (fake DB도 start_inventory 0으로 둘 것)
    상태 파일은 실제용과 분리: ~/.ros/place_sim_state.json
  실제 로봇에서는 절대 켜지 말 것 (박스를 바닥에 밀어 넣음).
================================================================
"""
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from box_comm.place_interface import PartError, start
from box_comm.robot import connect_dsr
from box_comm.common import ROBOT, STACK, TCP, TOOL, VALID_CLASSES
from dsr_msgs2.srv import (GetCurrentPosx, GetToolForce, CheckMotion, MoveStop,
                          TaskComplianceCtrl, SetDesiredForce, ReleaseForce,
                          ReleaseComplianceCtrl, GetCurrentTcp, SetCurrentTcp,
                          ConfigCreateTcp, SetRobotMode, GetRobotMode,
                          GetCurrentTool, SetCurrentTool, ConfigCreateTool)


KEEPALIVE_SEC = 1.0      # 하강 중 관제에 '살아 있음'을 보내는 간격


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
    if type(c['max_layers']) is not int or type(c['slot_count']) is not int:
        raise ValueError('max_layers / slot_count는 정수여야 합니다')
    if c['max_layers'] != STACK['levels'] or c['slot_count'] != STACK['max_tiers']:
        raise ValueError(f"place_config의 {c['max_layers']}층/{c['slot_count']}자리가 공용 STACK "
                         f"({STACK['levels']}층/{STACK['max_tiers']}자리)과 다릅니다. "
                         f"box_comm/common.py STACK과 같이 맞추세요 (관제·DB·UI가 같은 규칙을 씀)")
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
            raise ValueError(f'{key}: 자리별 첫 접촉 Z 2개 필요')
        for z in refs:
            if z is not None and (type(z) not in (int, float) or not math.isfinite(z) or not c['min_search_z'] < z < c['travel_z']):
                raise ValueError(f'{key}: 잘못된 첫 접촉 Z')
        for z in refs:
            if z is not None and z + c['max_layers'] * h + 10.0 >= c['travel_z']:
                raise ValueError(f"{key}: {c['max_layers']}층 위 파지 접촉 높이와 상부 Z 여유 부족")
    return c


def count_from_position(tier, level, levels=3):
    return (tier - 1) * levels + level


def level_from_z(z, floor_z, height, tolerance):
    below = round((z - floor_z) / height)
    if below < 0 or abs(z - floor_z - below * height) > tolerance:
        raise ValueError(f'층 판단 오차: Z={z:.2f}, 첫 접촉 Z={floor_z:.2f}, 물품 높이={height:.2f}')
    return below + 1


class PlaceWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        node.declare_parameter('place_config', default_config_path())
        node.declare_parameter('place_state', '')   # 빈 값 = 실제용/시뮬용 기본 경로
        node.declare_parameter('sim_contact', False)
        node.declare_parameter('learn_empty_slots', False)
        node.declare_parameter('show_force', False)
        node.declare_parameter('clear_pending', False)
        config_path = Path(node.get_parameter('place_config').value).expanduser()
        self.log.info(f'[place_main] 설정 파일: {config_path}')
        self.c = validate_config(json.loads(config_path.read_text()))
        self.learn_empty = bool(node.get_parameter('learn_empty_slots').value)
        self.show_force = bool(node.get_parameter('show_force').value)
        self.sim_contact = bool(node.get_parameter('sim_contact').value)
        self._sim_stack = {}                # (클래스, 자리) → 가상으로 쌓인 수 (sim_contact 전용)
        state_name = 'place_sim_state.json' if self.sim_contact else 'place_real_v04_state.json'
        state_param = node.get_parameter('place_state').value
        self.state_path = Path(state_param or str(Path.home() / '.ros' / state_name)).expanduser()
        if self.sim_contact:
            self.log.warn('[place_main] sim_contact=true — 힘 센서 대신 가상 높이까지 내려감. '
                          '에뮬레이터 전용, 실제 로봇에서 쓰지 말 것')
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else {
            'version': 1, 'references': {k: [None, None] for k in self.c['classes']}, 'pending': None}
        if self.state['version'] != 1:
            raise ValueError('지원하지 않는 기준 상태 파일입니다')
        if self.state['pending']:
            if node.get_parameter('clear_pending').value:
                self.log.warn(f"[place_main] 이전 미완료 작업 기록 삭제 (clear_pending): {self.state['pending']}")
                self.state['pending'] = None
                save_state(self.state_path, self.state)
            else:
                self.log.warn(f"[place_main] 이전 작업 미완료: {self.state['pending']} — 그리퍼·적재 상태 확인 후 "
                              f"-p clear_pending:=true 로 다시 실행해야 적재를 받습니다")
        self._report = None                 # run() 동안 관제 report (정지 확인·keepalive용)
        self._last_keepalive = 0.0
        self.lock = threading.Lock()
        self.search_active = self.force_active = self.compliance_active = False
        self.dr, self.dsr_node = connect_dsr('place_real_dsr')
        self.clients = {}
        for key, typ, suffix in (
            ('tool_get', GetCurrentTool, 'tool/get_current_tool'),
            ('tool_set', SetCurrentTool, 'tool/set_current_tool'),
            ('tool_create', ConfigCreateTool, 'tool/config_create_tool'),
            ('tcp_get', GetCurrentTcp, 'tcp/get_current_tcp'),
            ('tcp_set', SetCurrentTcp, 'tcp/set_current_tcp'),
            ('tcp_create', ConfigCreateTcp, 'tcp/config_create_tcp'),
            ('mode_get', GetRobotMode, 'system/get_robot_mode'),
            ('mode_set', SetRobotMode, 'system/set_robot_mode'),
            ('pose', GetCurrentPosx, 'aux_control/get_current_posx'),
            ('force', GetToolForce, 'aux_control/get_tool_force'),
            ('motion', CheckMotion, 'motion/check_motion'),
            ('stop', MoveStop, 'motion/move_stop'),
            ('compliance', TaskComplianceCtrl, 'force/task_compliance_ctrl'),
            ('target_force', SetDesiredForce, 'force/set_desired_force'),
            ('release_force', ReleaseForce, 'force/release_force'),
            ('release_compliance', ReleaseComplianceCtrl, 'force/release_compliance_ctrl')):
            self.clients[key] = self.dsr_node.create_client(typ, ROBOT['service_ns'].rstrip('/') + '/' + suffix)
        self.log.info(f"[place_main] 적재 규칙 {self.c['max_layers']}층 x {self.c['slot_count']}자리, "
                      f"자리 간격 x {self.c['slot_dx_mm']:+.0f}mm, 클래스 {sorted(self.c['classes'])}")

    # ------------------------------------------------------------------
    # 관제 정지 대응 (box_comm Reporter)
    # ------------------------------------------------------------------
    def _check(self):
        """관제가 정지를 요청했으면 PartError('CANCELED') — 모든 이동 전에 호출"""
        if self._report is not None:
            self._report.check()

    def _keepalive(self):
        """오래 걸리는 루프 안에서 매 반복 호출: 정지 확인 + 1초마다 관제에 '살아 있음'"""
        if self._report is None:
            return
        self._report.check()
        now = time.monotonic()
        if now - self._last_keepalive >= KEEPALIVE_SEC:
            self._report.keepalive()
            self._last_keepalive = now

    def _request(self, key, request, timeout=2.0, allow_failure=False):
        client = self.clients[key]
        if not client.service_is_ready():
            raise RuntimeError(f'두산 서비스 준비 안 됨: {key}')
        future = client.call_async(request)
        executor = self.dsr_node.executor
        if executor is not None and executor.is_spinning:
            ready = threading.Event()
            future.add_done_callback(lambda _: ready.set())
            ready.wait(timeout)
        else:
            rclpy.spin_until_future_complete(self.dsr_node, future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'두산 서비스 시간 초과: {key}')
        result = future.result()
        if result is None or (not result.success and not allow_failure):
            raise RuntimeError(f'두산 서비스 실패: {key}')
        return result

    def _ensure_robot(self):
        # Reuse existing verified profiles. Do not overwrite named profiles.
        for kind, cfg, get_type, set_type, create_type in (
            ('tcp', TCP, GetCurrentTcp, SetCurrentTcp, ConfigCreateTcp),
            ('tool', TOOL, GetCurrentTool, SetCurrentTool, ConfigCreateTool)):
            name = cfg['name']
            if self._request(kind + '_get', get_type.Request()).info == name:
                continue
            req = SetRobotMode.Request()
            req.robot_mode = 0
            self._request('mode_set', req)
            if self._request('mode_get', GetRobotMode.Request()).robot_mode != 0:
                raise RuntimeError('수동 모드 확인 실패')
            selection = set_type.Request()
            selection.name = name
            if not self._request(kind + '_set', selection, allow_failure=True).success:
                registration = create_type.Request()
                registration.name = name
                if kind == 'tcp':
                    registration.pos = list(cfg['pos'])
                else:
                    registration.weight = cfg['weight']
                    registration.cog = list(cfg['cog'])
                    registration.inertia = list(cfg['inertia'])
                self._request(kind + '_create', registration)
                self._request(kind + '_set', selection)
            if self._request(kind + '_get', get_type.Request()).info != name:
                raise RuntimeError(f'{kind} 적용 확인 실패: {name}')
        if self._request('mode_get', GetRobotMode.Request()).robot_mode != 1:
            req = SetRobotMode.Request()
            req.robot_mode = 1
            self._request('mode_set', req)
        if self._request('mode_get', GetRobotMode.Request()).robot_mode != 1:
            raise RuntimeError('자동 모드 확인 실패')
        self.log.info('TCP/Tool ready: GripperDA_v1 / Tool Weight')

    def _call(self, name, *args, **kwargs):
        result = getattr(self.dr, name)(*args, **kwargs)
        if isinstance(result, (int, float)) and not isinstance(result, bool) and result < 0:
            raise RuntimeError(f'{name} 실패: {result}')
        return result

    def _pose(self):
        req = GetCurrentPosx.Request()
        req.ref = 0
        values = self._request('pose', req).task_pos_info
        if not values or len(values[0].data) < 6:
            raise RuntimeError('TCP 좌표 응답 오류')
        pose = list(values[0].data[:6])
        if not all(math.isfinite(v) for v in pose):
            raise RuntimeError('유효하지 않은 TCP 좌표')
        return pose

    def _fz(self):
        req = GetToolForce.Request()
        req.ref = 0
        values = self._request('force', req).tool_force
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('유효하지 않은 외력 응답')
        return float(values[2])

    def _move(self, pose):
        self._check()
        self._call('movel', self.dr.posx(pose), vel=self.c['speed_mm_s'],
                   acc=self.c['acc_mm_s2'], ref=self.dr.DR_BASE)
        actual = self._pose()
        if any(abs(actual[i] - pose[i]) > 2.0 for i in range(3)):
            raise RuntimeError('이동 목표 미도달')

    def _raise(self):
        pose = self._pose()
        if pose[2] < self.c['travel_z']:
            pose[2] = self.c['travel_z']
            self._move(pose)

    def _grip(self, opened):
        self._check()
        self._call('set_digital_output', 1, 0 if opened else 1)
        self._call('set_digital_output', 2, 1 if opened else 0)
        self._call('wait', self.c['gripper_wait_s'])

    def _stop(self, emergency=False):
        errors = []
        if self.force_active:
            try:
                req = ReleaseForce.Request()
                req.time = 0.0
                self._request('release_force', req)
                self.force_active = False
            except Exception as exc:
                errors.append(exc)
        if self.compliance_active:
            try:
                self._request('release_compliance', ReleaseComplianceCtrl.Request())
                self.compliance_active = False
            except Exception as exc:
                errors.append(exc)
        if emergency or errors:
            req = MoveStop.Request()
            req.stop_mode = 1
            self._request('stop', req, timeout=5.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._request('motion', CheckMotion.Request()).status == 0:
                self.search_active = False
                if errors:
                    raise RuntimeError(f'힘·순응제어 해제 실패: {errors}')
                return
            time.sleep(0.05)
        raise RuntimeError('정지 확인 실패')

    def _descend(self, approach):
        c = self.c
        time.sleep(0.5)
        samples = []
        deadline = time.monotonic() + c['baseline_duration_s']
        while time.monotonic() < deadline:
            self._keepalive()
            fz = self._fz()
            if abs(fz) >= c['abs_force_limit_n']:
                raise RuntimeError('기준 외력 상한 초과')
            samples.append(fz)
            time.sleep(0.05)
        if len(samples) < 5 or max(samples) - min(samples) > c['baseline_max_span_n']:
            raise RuntimeError('기준 외력 불안정')
        baseline = sum(samples) / len(samples)
        start_z = self._pose()[2]
        self.search_active = self.compliance_active = True
        req = TaskComplianceCtrl.Request()
        req.stx = [3000., 3000., c['z_stiffness_n_m'], 200., 200., 200.]
        req.ref, req.time = 0, 0.0
        if self.show_force:
            print(f'[BEFORE] Z={start_z:.2f} mm | baseline Fz={baseline:.2f} N', flush=True)
        self._request('compliance', req)
        # Restore the sequence used in the successful stack_test experiment.
        settle_until = time.monotonic() + 2.0
        next_display = 0.0
        while rclpy.ok() and time.monotonic() < settle_until:
            self._keepalive()
            fz = self._fz()
            if abs(fz) >= c['abs_force_limit_n'] or abs(fz-baseline) >= c['abort_force_n']:
                raise RuntimeError(f'순응제어 중 외력 상한 초과: Fz={fz:.2f}')
            z = self._pose()[2]
            now = time.monotonic()
            if self.show_force and now >= next_display:
                print(f'[COMPLIANCE ONLY] Z={z:.2f} mm | dZ={z-start_z:+.2f} mm | '
                      f'Fz={fz:.2f} N', flush=True)
                next_display = now + 0.5
            if abs(z-start_z) >= 2.0 or z <= c['min_search_z']:
                raise RuntimeError(f'순응제어 중 위치 변동 초과: Z={z:.2f}, dZ={z-start_z:+.2f}')
            time.sleep(0.05)
        if not rclpy.ok():
            raise RuntimeError('목표 힘 적용 전 ROS 종료')
        if self.show_force:
            print(f"[FORCE APPLY] BASE Z force={-c['desired_force_n']:.2f} N, REL mode", flush=True)
        self.force_active = True
        req = SetDesiredForce.Request()
        req.fd = [0., 0., -c['desired_force_n'], 0., 0., 0.]
        req.dir = [0, 0, 1, 0, 0, 0]
        req.ref, req.time, req.mod = 0, 0.5, self.dr.DR_FC_MOD_REL
        self._request('target_force', req)
        force_started = time.monotonic()
        deadline = force_started + c['search_timeout_s']
        next_display = 0.0
        while rclpy.ok():
            self._keepalive()
            fz = self._fz()
            change = fz - baseline
            if abs(fz) >= c['abs_force_limit_n'] or abs(change) >= c['abort_force_n']:
                raise RuntimeError(f'외력 상한 초과: Fz={fz:.2f}, delta={change:.2f}')
            if c['contact_sign'] * change >= c['contact_force_n']:
                self._stop()
                time.sleep(0.2)
                z, fz = self._pose()[2], self._fz()
                if abs(fz) >= c['abs_force_limit_n'] or abs(fz - baseline) >= c['abort_force_n']:
                    raise RuntimeError('정지 후 외력 상한 초과')
                if c['contact_sign'] * (fz - baseline) < c['contact_force_n']:
                    raise RuntimeError('정지 후 접촉이 유지되지 않음')
                if not c['min_search_z'] < z <= approach[2] + 2.0:
                    raise RuntimeError('접촉 Z 범위 오류')
                return z
            z = self._pose()[2]
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
        self._keepalive()
        self._move([approach[0], approach[1], z, *approach[3:]])
        self.log.info(f'[place_main] (sim) {category}/{tier} 가상 접촉 Z={z:.2f} (이미 {stacked}개)')
        return z

    def _goal_poses(self, category):
        home = self.poses.get('home_j')
        zone = self.poses.get('measure_zone')
        xy = (self.poses.get('place_zones') or {}).get(category)
        if not finite_vector(home, 6) or not finite_vector(zone, 3) or not finite_vector(xy, 2):
            raise ValueError('Goal의 HOME/측정존/해당 분류 적재 구역 좌표가 없습니다')
        if all(abs(v) < 1e-6 for v in zone):
            raise ValueError('측정존 (0,0,0)은 사용할 수 없습니다')
        if not self.c['min_search_z'] < zone[2] < self.c['travel_z']:
            raise ValueError('측정존 TCP 파지 Z 범위 오류')
        return list(home), list(zone), list(xy)

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
                raise ValueError('3층 위 접촉 높이에 비해 이송 Z 여유가 부족합니다')
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
            raise ValueError('학습한 접촉 Z 기준으로 3층 위 탐색 시 상부 Z 여유가 부족합니다')
        self.state['references'][category][tier-1] = {
            'signature': self._reference_signature(category, x, y), 'first_contact_z_mm': z}
        save_state(self.state_path, self.state)

    def pick_from_measure_zone(self, zone):
        pick = [*zone, *self.c['pick_orientation']]
        self._raise()
        self._move([pick[0], pick[1], self.c['travel_z'], *pick[3:]])
        self._grip(True)
        self._move(pick)
        self._grip(False)
        self._move([pick[0], pick[1], self.c['travel_z'], *pick[3:]])

    def move_above(self, category, x, y):
        # Always rise vertically before moving laterally, including while holding a box.
        self._raise()
        pose = [x, y, self.c['travel_z'], *self.c['classes'][category]['orientation']]
        self._move(pose)
        return pose

    def release(self):
        self._grip(True)

    def go_home(self, home):
        self._check()
        self._call('movej', self.dr.posj(home), vel=self.c['home_speed_deg_s'],
                   acc=self.c['home_acc_deg_s2'])

    def run(self, result_class, report) -> int:
        if not self.lock.acquire(blocking=False):
            raise PartError('PLACE_FAILED', '적재 작업이 이미 실행 중입니다')
        motion_started = False
        self._report = report
        self._last_keepalive = time.monotonic()
        try:
            c = self.c
            if result_class not in c['classes']:
                raise ValueError(f'{result_class}: place_config.json에 설정이 없습니다 '
                                 f'(지원: {sorted(c["classes"])})')
            if self.state['pending']:
                raise ValueError('이전 작업 미완료: 파지 물품/실제 적재 상태 확인 후 '
                                 'place_main을 -p clear_pending:=true 로 다시 실행하세요')
            home, zone, xy = self._goal_poses(result_class)
            references = [self._reference(result_class, tier,
                           xy[0]+(tier-1)*c['slot_dx_mm'], xy[1])
                          for tier in range(1, c['slot_count']+1)]
            for client in self.clients.values():
                if not client.wait_for_service(timeout_sec=10.0):
                    raise RuntimeError(f'서비스 준비 안 됨: {client.srv_name}')
            self._ensure_robot()
            self._call('set_ref_coord', self.dr.DR_BASE)
            self.state['pending'] = {'class': result_class, 'phase': 'picking'}
            save_state(self.state_path, self.state)
            report('picking')
            # Initialize a known joint posture on the first accepted Goal only.
            if not getattr(self, '_home_initialized', False):
                motion_started = True
                self.go_home(home)
                self._home_initialized = True
            motion_started = True
            self.pick_from_measure_zone(zone)
            self.state['pending']['phase'] = 'moving'
            save_state(self.state_path, self.state)
            report('moving')
            approach = self.move_above(result_class, xy[0], xy[1])
            report('placing')
            # The v0.4 contract probes the first slot on EVERY Goal, even if it was full.
            for tier in range(1, c['slot_count']+1):
                x, y = xy[0]+(tier-1)*c['slot_dx_mm'], xy[1]
                if tier > 1:
                    approach = self.move_above(result_class, x, y)
                self.state['pending'].update(phase='placing', tier=tier)
                save_state(self.state_path, self.state)
                reference = references[tier-1]
                if self.sim_contact:
                    z = self._sim_descend(result_class, tier, approach, reference)
                else:
                    z = self._descend(approach)
                if reference is None:
                    # learn_empty_slots explicitly asserts these uncalibrated slots are empty.
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
                self.state['pending'].update(phase='releasing', contact_z_mm=z, level=level)
                save_state(self.state_path, self.state)
                self.release()
                self._raise()
                self.state['pending']['phase'] = 'released_and_retracted'
                save_state(self.state_path, self.state)
                self.go_home(home)
                count = count_from_position(tier, level, c['max_layers'])
                if self.sim_contact:
                    self._sim_stack[(result_class, tier)] = level
                # Save manually supplied references as well as learned ones.
                self._remember_reference(result_class, tier, x, y, reference)
                message = f'{result_class} {tier}번 자리 {level}층 적재 → 수량 {count} (TCP Z={z:.2f})'
                self.log.info(message)
                report.log(message)
                self.state['pending'] = None
                save_state(self.state_path, self.state)
                return count
            raise PartError('NO_SLOT', f"{result_class} {c['slot_count']}자리 모두 {c['max_layers']}층 만재. 물품 파지 유지")
        except BaseException as exc:
            if motion_started or self.search_active or self.force_active or self.compliance_active:
                try:
                    self._stop(emergency=True)
                except Exception as stop_exc:
                    code = exc.code if isinstance(exc, PartError) else 'PLACE_FAILED'   # CANCELED 등 유지
                    raise PartError(code, f'{exc}; 정지 확인 실패: {stop_exc}') from exc
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            if isinstance(exc, PartError):
                raise
            raise PartError('PLACE_FAILED', str(exc)) from exc
        finally:
            self._report = None
            self.lock.release()


def main():
    start(PlaceWork)


if __name__ == '__main__':
    main()