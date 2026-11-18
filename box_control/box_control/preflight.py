#!/usr/bin/env python3
"""
preflight.py — 작업 시작 전 점검 (관제 전용)
================================================================
control.py가 GetJobData 직후, 시작 대기(STANDBY) 전에 run_all()을 한 번 호출한다.
점검 하나가 FAIL이면 작업을 시작하지 않는다. WARN은 기록만 하고 진행한다.

점검 목록
  1. 분류 기준(specs)  : A~D가 모두 있고 치수가 양수인지 (무게 0 = 미측정 → WARN, 11/18)
  2. 고정 좌표          : DB에서 받은 HOME·측정존 좌표가 있고 형식이 맞는지 (기준은 DB, v0.3)
                         관제는 DB 값으로 이미 덮어쓴 상태 → common.py와 비교하지 않음
  3. 적재 구역 (v0.4)  : A~D 구역 1단 x, y가 모두 있는지 (없으면 FAIL, (0, 0)이면 WARN)
  4. 로봇 상태         : robot_monitor=dsr(기본)/fake면 STANDBY·MOVING이어야 함 (아니면 FAIL),
                         dsr인데 bringup·dsr_msgs2가 없으면 FAIL,
                         off(로봇 없이 테스트)면 WARN (작업 중 로봇 정지를 감지하지 못함)
  5. 그리퍼 (11/17)    : RG2 Modbus에 접속해 상태를 읽을 수 있는지 (접속 후 바로 끊음)
                         실패하면 FAIL — OnRobot 드라이버가 접속을 잡고 있는지 확인
                         BOX_GRIPPER=fake(에뮬레이터)면 OK(가짜), robot_monitor≠dsr면 확인 안 함(WARN)
  6. 측정존 비우기 (11/17) : 관제 복구 동작(recovery.py) 준비. push_path가 없으면 WARN
                         (판별·적재 집기 실패 시 측정존을 못 비워 작업이 바로 종료됨). fake 모드면 OK

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
           if min(s['width_mm'], s['depth_mm'], s['height_mm']) <= 0]
    if bad:
        return [(FAIL, f'분류 기준 {bad}에 치수 0 이하 값 있음 (DB 물품값 확인)')]
    # 무게 0 = 미측정 (11/18: 판별이 아직 무게를 쓰지 않음 → 경고만)
    bad_w = [s['class_id'] for s in specs if s['weight_g'] < 0]
    if bad_w:
        return [(FAIL, f'분류 기준 {bad_w}에 음수 무게 (DB 물품값 확인)')]
    no_w = [s['class_id'] for s in specs if s['weight_g'] == 0]
    if no_w:
        return [(WARN, f'분류 기준 {len(specs)}개, 무게 미측정 {no_w} (판별이 무게를 쓰기 전까지는 영향 없음)')]
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
                           '로봇 없이 테스트할 때만 쓰세요 (실제·에뮬레이터는 기본값 dsr).')]
        return [(FAIL, f'{name} — 로봇 bringup·연결을 확인해 주세요.')]
    if robot_stop_code(state):
        return [(FAIL, f'로봇 상태 {name}({state}) — STANDBY에서 시작해야 합니다 (정지·비상정지 해제 확인).')]
    return [(OK, f'로봇 상태 {name}')]


# ----------------------------------------------------------------------
# 점검 5: 그리퍼 (RG2 Modbus, 11/17)
#   Compute Box는 동시 접속 1개 → 다른 프로그램(OnRobot 드라이버 등)이 잡고 있으면 파트가 그리퍼를 못 씀
# ----------------------------------------------------------------------
def check_gripper(ctrl):
    from box_comm.gripper import RG2, gripper_mode
    if gripper_mode() == 'fake':
        return [(OK, '가짜 그리퍼 (BOX_GRIPPER=fake, 에뮬레이터용)')]
    monitor = getattr(ctrl.io, '_monitor', 'dsr')
    if monitor != 'dsr':
        return [(WARN, f'robot_monitor={monitor} → 그리퍼 접속 확인 안 함 (fake 파트 테스트용)')]
    try:
        g = RG2()
    except Exception as e:
        return [(FAIL, f'RG2 Modbus 접속 실패 ({type(e).__name__}: {e}) — OnRobot 드라이버가 떠 있으면 '
                       '끄세요 (pkill -f OnRobotRGControllerServer), 그리퍼 전원·랜선 확인')]
    try:
        st = g.status()
    finally:
        g.close()
    if any(st['safety']):
        return [(FAIL, f'그리퍼 안전 상태 비트 켜짐 {st["safety"]} — Compute Box 확인')]
    return [(OK, f'RG2 접속 OK (패드 간격 {st["width_mm"]:.1f} mm)')]


# ----------------------------------------------------------------------
# 점검 6: 측정존 비우기 경로 (집기 실패 복구, 11/17)
# ----------------------------------------------------------------------
def check_recovery(ctrl):
    from box_comm.common import RECOVERY
    mode = getattr(getattr(ctrl, 'recovery', None), 'mode', 'robot')
    if mode != 'robot':
        return [(OK, f'측정존 비우기 {mode} (로봇 안 움직임, 시험용)')]
    if not RECOVERY['push_path']:
        return [(WARN, '측정존 밀어내기 경로 미정 (common.RECOVERY push_path) — 판별·적재에서 못 집으면 '
                       '비우기를 건너뛰고(fake) 바로 재스캔합니다. 스캔하는 동안 사람이 측정존을 치워 주세요')]
    return [(OK, f'측정존 밀어내기 경로 {len(RECOVERY["push_path"])}점')]


CHECKS = [
    ('분류 기준', check_specs),
    ('고정 좌표', check_fixed_poses),
    ('적재 구역', check_place_zones),
    ('로봇 상태', check_robot),
    ('그리퍼', check_gripper),
    ('측정존 비우기', check_recovery),
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
