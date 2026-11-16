#!/usr/bin/env python3
"""
preflight.py — 작업 시작 전 점검 (관제 전용)
================================================================
control.py가 GetJobData 직후, 시작 대기(STANDBY) 전에 run_all()을 한 번 호출한다.
점검 하나가 FAIL이면 작업을 시작하지 않는다. WARN은 기록만 하고 진행한다.

점검 목록
  1. 분류 기준(specs)  : A~D가 모두 있고 값이 양수인지
  2. 고정 좌표          : DB에서 받은 HOME·측정존 좌표가 있고 형식이 맞는지 (기준은 DB, v0.3)
                         관제는 DB 값으로 이미 덮어쓴 상태 → common.py와 비교하지 않음
  3. 적재 구역 (v0.4)  : A~D 구역 1단 x, y가 모두 있는지 (없으면 FAIL, (0, 0)이면 WARN)
  4. 로봇 상태         : robot_monitor=dsr/fake면 STANDBY·MOVING이어야 함 (아니면 FAIL),
                         off면 WARN (작업 중 로봇 정지를 감지하지 못함)
  (추가 예정) safety 항목들

점검 추가하는 법
  def check_xxx(ctrl) -> list[(level, message)]  를 만들고 CHECKS에 넣는다.
  level: 'OK' / 'WARN' / 'FAIL'
================================================================
"""

from box_comm.common import POSE_TOLERANCE, VALID_CLASSES, robot_stop_code

OK, WARN, FAIL = 'OK', 'WARN', 'FAIL'

_POSE_LABEL = {'home_j': 'HOME 위치', 'measure_zone': '측정존 위치'}


# ----------------------------------------------------------------------
# 점검 1: 분류 기준
# ----------------------------------------------------------------------
def check_specs(ctrl):
    specs = ctrl.specs
    got = {s['class_id'] for s in specs}
    missing = [c for c in VALID_CLASSES if c not in got]
    if missing:
        return [(FAIL, f'분류 기준에 {missing} 없음 (DB 물품값 확인)')]
    bad = [s['class_id'] for s in specs
           if min(s['width_mm'], s['depth_mm'], s['height_mm'], s['weight_g']) <= 0]
    if bad:
        return [(FAIL, f'분류 기준 {bad}에 0 이하 값 있음 (DB 물품값 확인)')]
    return [(OK, f'분류 기준 {len(specs)}개')]


# ----------------------------------------------------------------------
# 점검 2: 고정 좌표 (기준 = DB)
#   DB 값이 없으면 파트에 보낼 좌표가 없으므로 FAIL
#   측정존이 (0, 0, 0)이면 WARN (아직 설정 안 한 값으로 봄)
# ----------------------------------------------------------------------
def check_fixed_poses(ctrl):
    db = ctrl.poses
    if db is None:
        return [(FAIL, 'DB에 고정 좌표가 없습니다. DB에 HOME·측정존 좌표를 설정한 뒤 실행해 주세요.')]

    results = []
    expected_len = {'home_j': 6, 'measure_zone': 3}
    for key, label in _POSE_LABEL.items():
        got = db.get(key)
        if got is None or len(got) != expected_len[key]:
            results.append((FAIL, f'{label}: DB 값 형식 오류 ({got})'))
            continue
        if key == 'measure_zone' and all(abs(v) < POSE_TOLERANCE for v in got):
            results.append((WARN, f'{label}: DB 값이 (0, 0, 0) — 아직 설정하지 않은 값으로 보입니다'))
            continue
        results.append((OK, f'{label}: DB 값 사용 {got}'))
    return results


# ----------------------------------------------------------------------
# 점검 3: 적재 구역 (v0.4, 기준 = DB)
#   클래스마다 1단 x, y가 있어야 한다. 없으면 그 클래스를 적재할 수 없으므로 FAIL
# ----------------------------------------------------------------------
def check_place_zones(ctrl):
    zones = (ctrl.poses or {}).get('place_zones') or {}
    missing = [c for c in VALID_CLASSES if zones.get(c) is None]
    if missing:
        return [(FAIL, f'적재 구역 {missing}의 1단 x, y가 DB에 없습니다. DB에 설정한 뒤 실행해 주세요.')]
    results = []
    for c in VALID_CLASSES:
        x, y = zones[c]
        if abs(x) < POSE_TOLERANCE and abs(y) < POSE_TOLERANCE:
            results.append((WARN, f'{c} 구역: (0, 0) — 아직 설정하지 않은 값으로 보입니다'))
        else:
            results.append((OK, f'{c} 구역 1단: x={x:.1f}, y={y:.1f}'))
    return results


# ----------------------------------------------------------------------
# 점검 4: 로봇 상태 (정지 기능)
# ----------------------------------------------------------------------
def check_robot(ctrl):
    state, name = ctrl.io.robot_state()
    if state is None:
        if name == 'robot_monitor=off':
            return [(WARN, '로봇 상태 감시 꺼짐 (robot_monitor=off) — 작업 중 로봇 정지·비상정지 상태를 감지하지 못합니다. '
                           '실제·에뮬레이터는 -p robot_monitor:=dsr')]
        return [(FAIL, f'{name} — 로봇 bringup·연결을 확인해 주세요.')]
    if robot_stop_code(state):
        return [(FAIL, f'로봇 상태 {name}({state}) — STANDBY에서 시작해야 합니다 (정지·비상정지 해제 확인).')]
    return [(OK, f'로봇 상태 {name}')]


CHECKS = [
    ('분류 기준', check_specs),
    ('고정 좌표', check_fixed_poses),
    ('적재 구역', check_place_zones),
    ('로봇 상태', check_robot),
]


def run_all(ctrl) -> bool:
    """모든 점검 실행. FAIL이 하나라도 있으면 False."""
    log = ctrl.log
    passed = True
    for name, fn in CHECKS:
        try:
            results = fn(ctrl)
        except Exception as e:                     # 점검 코드 자체의 오류도 FAIL로
            results = [(FAIL, f'점검 오류: {type(e).__name__}: {e}')]
        for level, msg in results:
            line = f'[preflight] {level:4s} {name}: {msg}'
            if level == FAIL:
                log.error(line)
                passed = False
            elif level == WARN:
                log.warn(line)
            else:
                log.info(line)
    if passed:
        log.info('[preflight] 통과')
    else:
        log.error('[preflight] 실패')
    return passed
