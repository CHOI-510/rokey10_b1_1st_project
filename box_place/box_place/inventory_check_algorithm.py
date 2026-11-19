#!/usr/bin/env python3
"""시작 사이클 재고 점검 — 첫 자리 3층일 때만 둘째 자리를 추가 측정.

배치: box_place/box_place/inventory_check_algorithm.py
별도 JSON 없이 DEFAULT_CONFIG를 사용한다. 기존 inventory_check_config 파라미터로 외부 설정도 가능.
자리 간격/AB 자세는 CLASS_LAYOUT가 우선한다. 첫 자리 XY는 관제가 Goal.poses로 보낸 DB 좌표.

둘째 자리: A/B 첫 자리에서 [0,-45], C/D [0,-30].
기준 첫 자리: A(480,-200), B(630,-200), C(635,-30), D(635,115).
예상 둘째 자리: A(480,-245), B(630,-245), C(635,-60), D(635,85).

교정: A/B/C/D의 첫째·둘째 자리 8곳을 모두 비운 뒤,
서버 시작 시 inventory_calibrate=true, inventory_empty_confirmed=true로 각 분류 Goal을 실행한다.
완료 후 두 파라미터가 false인 일반 모드로 서버를 재시작한다.
각 자리의 빈 그리퍼 바닥 접촉 TCP Z를 별도로 저장한다. 기존 적재용 기준 Z는 사용하지 않는다.
위치·자세가 달라진 기존 기준은 거부하며 자동으로 새 좌표의 기준으로 바꾸지 않는다.

첫 자리 0~2개: 그 수량만 반환, 둘째 자리 생략.
첫 자리 3개: 둘째 자리 0~3개를 추가 측정, 합계 3~6개 반환.
따라서 첫 자리에서 사람이 물체를 빼고 둘째 자리에 남긴 상황은 이 조건부 점검으로는 합산되지 않는다.
두 번째 자리 측정 실패 시 첫 자리만의 부분 수량을 반환하지 않는다.

관제는 ControlInterface에서 /inventory_check Goal을 A/B/C/D 순서로 보내고
각 결과가 모두 0~6 범위인지 확인한 뒤에만 재고 기준값으로 사용한다.
실행 진입점 inventory_check_main.py와 공용 part_server.py/Place Action 정의는 기존과 같다.
common.STAGES의 inventory_check 등록은 필요하다. Recovery에는 연결하지 않는다.
"""
from copy import deepcopy
import json
import math
import time
from pathlib import Path

from box_comm.common import GRIPPER, STACK, TCP, TOOL, VALID_CLASSES, PartError
from box_comm.gripper import holding


# 외부 JSON 없이 실행 가능. 분류별 자리 간격과 자세는 이 값이 우선한다.
DEFAULT_CONFIG = {'travel_z': 160.0,
 'speed_mm_s': 60.0,
 'acc_mm_s2': 100.0,
 'home_speed_deg_s': 10.0,
 'home_acc_deg_s2': 10.0,
 'baseline_duration_s': 1.5,
 'baseline_max_span_n': 1.0,
 'z_stiffness_n_m': 500.0,
 'contact_sign': 1,
 'min_probe_tcp_z': 0.0,
 'max_probe_travel_mm': 160.0,
 'contact_force_n': 6.0,
 'desired_force_n': 8.0,
 'abort_force_n': 9.0,
 'abs_force_limit_n': 20.0,
 'search_timeout_s': 180.0,
 'contact_hold_s': 0.2,
 'contact_z_span_mm': 1.0,
 'layer_tolerance_mm': 3.0,
 'classes': {'A': {'height_mm': 20.0,
                   'orientation': [120.0, 180.0, 120.0],
                   'slot_offset_mm': [0.0, -45.0]},
             'B': {'height_mm': 25.0,
                   'orientation': [120.0, -180.0, 120.0],
                   'slot_offset_mm': [0.0, -45.0]},
             'C': {'height_mm': 20.0,
                   'orientation': [175.0, -180.0, -175.0],
                   'slot_offset_mm': [0.0, -30.0]},
             'D': {'height_mm': 20.0,
                   'orientation': [3.5, 180.0, 4.0],
                   'slot_offset_mm': [0.0, -30.0]}}}

CLASS_LAYOUT = {
    'A': {'slot_offset_mm': [0.0, -45.0], 'orientation': [120.0, 180.0, 120.0]},
    'B': {'slot_offset_mm': [0.0, -45.0], 'orientation': [120.0, -180.0, 120.0]},
    'C': {'slot_offset_mm': [0.0, -30.0]},
    'D': {'slot_offset_mm': [0.0, -30.0]},
}


def normalize_state(state):
    """v1 첫 자리 기준을 v2의 1번 기준으로 보존한다. 2번 기준을 임의 복제하지 않는다."""
    state = deepcopy(state)
    if not isinstance(state, dict) or not isinstance(state.get('references'), dict):
        raise ValueError('점검 기준 상태 파일 형식 오류')
    if state.get('version') == 1:
        state['references'] = {name: {'1': record} for name, record in state['references'].items()}
        state['version'] = 2
    if state.get('version') != 2:
        raise ValueError('지원하지 않는 점검 상태 버전')
    for name, slots in state['references'].items():
        if not isinstance(slots, dict) or any(key not in ('1', '2') for key in slots):
            raise ValueError(f'{name}: 자리별 점검 기준 형식 오류')
        if any(record is not None and not isinstance(record, dict) for record in slots.values()):
            raise ValueError(f'{name}: 점검 기준 레코드 형식 오류')
    return state


def vector(v, size):
    return isinstance(v, (list, tuple)) and len(v) == size and all(
        type(n) in (int, float) and math.isfinite(n) for n in v)


def count_from_contact(z, empty_z, height, tolerance, levels):
    raw = (z - empty_z) / height
    count = round(raw)
    if not 0 <= count <= levels or abs(z - empty_z - count * height) > tolerance:
        raise PartError('PLACE_FAILED', f'층수 판정 불가: 접촉 Z={z:.2f}, 빈 바닥 Z={empty_z:.2f}')
    return count


def validate(c):
    for name, overrides in CLASS_LAYOUT.items():
        if name in c.get('classes', {}):
            c['classes'][name].update(deepcopy(overrides))
    if STACK['max_tiers'] != 2 or STACK['levels'] != 3:
        raise ValueError('이번 시작 점검은 자리 2개 × 3층 기준입니다. 공용 STACK을 확인하세요')
    for key in ('travel_z', 'speed_mm_s', 'acc_mm_s2', 'home_speed_deg_s', 'home_acc_deg_s2',
                'contact_force_n', 'desired_force_n', 'abort_force_n', 'abs_force_limit_n',
                'z_stiffness_n_m', 'baseline_duration_s', 'baseline_max_span_n', 'search_timeout_s',
                'contact_hold_s', 'contact_z_span_mm', 'layer_tolerance_mm', 'max_probe_travel_mm'):
        if type(c.get(key)) not in (int, float) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'유한한 양수 필요: {key}')
    if not 0 < c['contact_force_n'] < c['desired_force_n'] < c['abort_force_n']:
        raise ValueError('접촉 힘 < 목표 힘 < 중단 힘 필요')
    if c.get('contact_sign') not in (-1, 1):
        raise ValueError('contact_sign은 -1 또는 1이어야 합니다')
    if type(c.get('min_probe_tcp_z')) not in (int, float) or not math.isfinite(c['min_probe_tcp_z']):
        raise ValueError('min_probe_tcp_z를 설정하세요')
    if c['min_probe_tcp_z'] >= c['travel_z']:
        raise ValueError('TCP 하한은 상부보다 낮아야 합니다')
    if not c.get('classes') or not set(c['classes']) <= set(VALID_CLASSES):
        raise ValueError('A/B/C/D 점검 분류 설정이 필요합니다')
    for name, spec in c['classes'].items():
        h = spec.get('height_mm')
        if type(h) not in (int, float) or not math.isfinite(h) or h <= 0:
            raise ValueError(f'{name} 높이 오류')
        if c['layer_tolerance_mm'] >= h / 2 or not vector(spec.get('orientation'), 3):
            raise ValueError(f'{name} 자세/허용 오차 오류')
        if not vector(spec.get('slot_offset_mm'), 2):
            raise ValueError(f'{name} 자리 간격 [dx, dy] 오류')
    return c


class InventoryCheckAlgorithm:
    def __init__(self, node, robot):
        self.robot = robot
        self.log = node.get_logger()
        node.declare_parameter('inventory_check_config', '')  # 빈 경로 = 내장 설정
        node.declare_parameter('inventory_check_state', str(Path.home() / '.ros' / 'inventory_check_state.json'))
        node.declare_parameter('inventory_calibrate', False)
        node.declare_parameter('inventory_empty_confirmed', False)
        config_path = node.get_parameter('inventory_check_config').value
        config = json.loads(Path(config_path).expanduser().read_text()) if config_path else deepcopy(DEFAULT_CONFIG)
        self.c = validate(config)
        self.path = Path(node.get_parameter('inventory_check_state').value).expanduser()
        self.calibrate = bool(node.get_parameter('inventory_calibrate').value)
        self.empty_confirmed = bool(node.get_parameter('inventory_empty_confirmed').value)
        if self.calibrate and not self.empty_confirmed:
            raise ValueError('교정 모드에서는 첫째·둘째 자리 모두 빈 상태임을 확인해야 합니다: inventory_empty_confirmed=true')
        raw_state = json.loads(self.path.read_text()) if self.path.exists() else {'version': 2, 'references': {}}
        self.state = normalize_state(raw_state)
        if raw_state.get('version') == 1:
            backup = self.path.with_name(self.path.name + '.v1.bak')
            if not backup.exists():
                with backup.open('xb') as stream:
                    stream.write(self.path.read_bytes())
            self.log.info(f'v1 첫 자리 기준 보존. 둘째 자리 기준은 별도 교정 필요. 백업: {backup}')

    def _slot_xy(self, result_class, tier, first_xy):
        dx, dy = self.c['classes'][result_class]['slot_offset_mm']
        return [first_xy[0] + (tier - 1) * dx, first_xy[1] + (tier - 1) * dy]

    def _reference(self, result_class, tier, xy):
        record = self.state['references'].get(result_class, {}).get(str(tier))
        if not record or record.get('signature') != self._signature(result_class, xy):
            raise PartError('PLACE_FAILED', f'{result_class}/{tier}번 자리: 빈 그리퍼 바닥 접촉 기준 교정 필요')
        empty_z = record.get('empty_contact_tcp_z')
        if type(empty_z) not in (int, float) or not math.isfinite(empty_z):
            raise PartError('PLACE_FAILED', f'{result_class}/{tier}번 자리: 저장된 기준 Z 오류')
        return empty_z

    def _signature(self, category, xy):
        # 위치·자세·힘 설정이 달라지면 기존 교정값을 자동으로 재사용하지 않는다.
        keys = ('contact_force_n', 'desired_force_n', 'contact_sign', 'z_stiffness_n_m')
        return {'xy': list(xy), 'orientation': self.c['classes'][category]['orientation'],
                'height_mm': self.c['classes'][category]['height_mm'],
                'tcp': TCP, 'tool': TOOL, 'probe': 'closed_empty_gripper',
                'probe_force': {k: self.c[k] for k in keys},
                'close_force_n': GRIPPER['grip_force_n']}

    def _save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(self.path.suffix + '.tmp')
        tmp.write_text(json.dumps(self.state, ensure_ascii=False, indent=2) + '\n')
        tmp.replace(self.path)

    def _move(self, pose):
        self.robot.movel(pose, vel=self.c['speed_mm_s'], acc=self.c['acc_mm_s2'])

    def _raise(self):
        p = self.robot.pose()
        if p[2] < self.c['travel_z'] - 0.1:
            p[2] = self.c['travel_z']
            self._move(p)

    def _force_limit(self, fz, baseline=None):
        if abs(fz) >= self.c['abs_force_limit_n'] or (
                baseline is not None and abs(fz - baseline) >= self.c['abort_force_n']):
            raise PartError('OVER_FORCE', f'점검 외력 상한 초과 Fz={fz:.2f}N')

    def _contact(self):
        """재고를 모르는 점검이므로 상부부터 힘제어. 85mm 위치제어 하강은 적용하지 않는다."""
        r, c = self.robot, self.c
        start_z = r.pose()[2]
        samples = []
        deadline = time.monotonic() + c['baseline_duration_s']
        while time.monotonic() < deadline:
            f = r.force_z()               # 공용 정지 확인·keepalive 포함
            self._force_limit(f)
            samples.append(f)
            r.wait(0.05)
        if len(samples) < 3 or max(samples)-min(samples) > c['baseline_max_span_n']:
            raise PartError('PLACE_FAILED', '점검 기준 외력 불안정')
        baseline = sum(samples) / len(samples)
        r.compliance_on([3000., 3000., c['z_stiffness_n_m'], 200., 200., 200.])
        # 기존 적재와 같은 순응제어 정착 순서.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            f, z = r.force_z(), r.pose()[2]
            self._force_limit(f, baseline)
            if abs(z-start_z) >= 2.0:
                raise PartError('PLACE_FAILED', '순응제어 정착 중 위치 변동 초과')
            r.wait(0.05)
        r.force_on([0., 0., -c['desired_force_n'], 0., 0., 0.], [0, 0, 1, 0, 0, 0], 0.5)
        deadline = time.monotonic() + c['search_timeout_s']
        candidate_since = None
        contact_zs = []
        while r.ok():
            f, z = r.force_z(), r.pose()[2]
            self._force_limit(f, baseline)
            if z < c['min_probe_tcp_z'] or z > start_z+2.0 or start_z-z > c['max_probe_travel_mm']:
                raise PartError('PLACE_FAILED', f'점검 TCP 탐색 범위 초과 Z={z:.2f}')
            now = time.monotonic()
            if c['contact_sign'] * (f-baseline) >= c['contact_force_n']:
                if candidate_since is None:
                    candidate_since = now
                contact_zs.append(z)
                # 짧은 힘 스파이크를 층수로 기록하지 않는다. 접촉 중 높이도 안정돼야 한다.
                if now-candidate_since >= c['contact_hold_s']:
                    if max(contact_zs)-min(contact_zs) <= c['contact_z_span_mm']:
                        r.stop()        # 힘/순응 해제와 정지 확인은 공용 DsrIO가 처리
                        return sum(contact_zs)/len(contact_zs)
                    candidate_since, contact_zs = now, [z]
            else:
                candidate_since, contact_zs = None, []
            if now >= deadline:
                raise PartError('PLACE_FAILED', f'점검 접촉 시간 초과 Z={z:.2f}')
            r.wait(0.05)
        raise PartError('CANCELED', '점검 중 ROS 종료')

    def _measure_slot(self, result_class, poses, report, tier):
        if result_class not in self.c['classes']:
            raise PartError('PLACE_FAILED', f'점검 대상이 아님: {result_class}')
        xy = (poses.get('place_zones') or {}).get(result_class)
        home = poses.get('home_j')
        if not vector(xy, 2) or not vector(home, 6):
            raise PartError('PLACE_FAILED', '점검 구역 XY/HOME 좌표가 없습니다')
        xy = self._slot_xy(result_class, tier, xy)
        signature = self._signature(result_class, xy)
        empty_z = None if self.calibrate else self._reference(result_class, tier, xy)
        held, why, width = holding(self.robot.gripper, None, check_range=False)
        if held:
            raise PartError('PLACE_FAILED', f'점검 전 물건을 쥐고 있습니다: {why}, 폭={width:.2f}')
        report('moving')
        # 관제가 다른 파트를 정지하고 로봇을 넘겼다는 전제. HOME 경로는 기존처럼 확인된 경로 사용.
        self.robot.movej(home, vel=self.c['home_speed_deg_s'], acc=self.c['home_acc_deg_s2'])
        self._raise()
        closed = self.robot.grip_close(GRIPPER['grip_force_n'])
        if closed.grabbed or not math.isfinite(closed.width_mm) or closed.width_mm > GRIPPER['empty_width_mm']:
            raise PartError('PLACE_FAILED', '점검 전 그리퍼가 비어 있지 않습니다')
        orient = self.c['classes'][result_class]['orientation']
        self._move([*xy, self.c['travel_z'], *orient])
        report('placing')
        try:
            z = self._contact()
        except Exception:
            # 자동 후퇴를 하지 않는다. 공용 AlgorithmWork의 실패 정지가 최종 처리한다.
            self.robot.stop(emergency=True)
            raise
        if self.calibrate:
            if not self.c['min_probe_tcp_z'] < z < self.c['travel_z']:
                raise PartError('PLACE_FAILED', '교정 접촉 Z 범위 오류')
            count = 0
        else:
            count = count_from_contact(z, empty_z, self.c['classes'][result_class]['height_mm'],
                                       self.c['layer_tolerance_mm'], STACK['levels'])
        self._raise()
        self.robot.movej(home, vel=self.c['home_speed_deg_s'], acc=self.c['home_acc_deg_s2'])
        if self.calibrate:
            self.state['references'].setdefault(result_class, {})[str(tier)] = {'signature': signature, 'empty_contact_tcp_z': z}
            self._save()
            report.log(f'{result_class}/{tier}번 자리 빈 그리퍼 바닥 Z 교정={z:.2f}mm (재고 갱신에 사용하지 않음)')
        else:
            report.log(f'{result_class}/{tier}번 자리 재고={count}개, 접촉 TCP Z={z:.2f}mm, 기준={empty_z:.2f}mm')
        return count


    def measure(self, result_class, poses, report):
        """정상 결과 int 0~6. 첫 자리가 3층일 때만 둘째 자리 추가 측정.

        교정 모드는 두 자리 모두 비운 상태에서 각 바닥을 측정하며, 재고 결과를 반환하지 않는다.
        """
        if result_class not in self.c['classes']:
            raise PartError('PLACE_FAILED', f'점검 대상이 아님: {result_class}')
        if self.calibrate:
            for tier in (1, 2):
                self._measure_slot(result_class, poses, report, tier)
            return ('CALIBRATED', 0)
        first = self._measure_slot(result_class, poses, report, 1)
        if type(first) is not int or not 0 <= first <= STACK['levels']:
            raise PartError('PLACE_FAILED', f'{result_class}/1번 자리 측정 수량 오류: {first!r}')
        if first < STACK['levels']:
            report.log(f'{result_class}: 첫 자리 {first}개 → 둘째 자리 점검 생략')
            return first
        report.log(f'{result_class}: 첫 자리 {first}층 만재 → 둘째 자리 추가 점검')
        second = self._measure_slot(result_class, poses, report, 2)
        if type(second) is not int or not 0 <= second <= STACK['levels']:
            raise PartError('PLACE_FAILED', f'{result_class}/2번 자리 측정 수량 오류: {second!r}')
        total = first + second
        report.log(f'{result_class}: 첫 자리 {first} + 둘째 자리 {second} = {total}개')
        return total
