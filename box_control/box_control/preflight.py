#!/usr/bin/env python3
"""
preflight.py — 작업 시작 전 점검 (관제 전용)
================================================================
control.py가 GetJobData 직후, 시작 대기(STANDBY) 전에 run_all()을 한 번 호출한다.
점검 하나가 FAIL이면 작업을 시작하지 않는다. WARN은 기록만 하고 진행한다.

점검 목록
  1. 분류 기준(specs)  : A~D가 모두 있고 값이 양수인지
  2. 고정 좌표          : DB 사본이 box_comm/common.py의 FIXED_POSES(원본)와 같은지
  (추가 예정) safety 항목들

점검 추가하는 법
  def check_xxx(ctrl) -> list[(level, message)]  를 만들고 CHECKS에 넣는다.
  level: 'OK' / 'WARN' / 'FAIL'
================================================================
"""

from box_comm.common import FIXED_POSES, POSE_TOLERANCE, VALID_CLASSES

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
# 점검 2: 고정 좌표 (common.py 원본 vs DB 사본)
# ----------------------------------------------------------------------
def check_fixed_poses(ctrl):
    db = ctrl.db_poses
    if db is None:
        return [(WARN, 'DB에 고정 좌표 사본 없음 → 비교 건너뜀')]

    results = []
    for key, label in _POSE_LABEL.items():
        ref = FIXED_POSES.get(key)
        if ref is None:
            results.append((WARN, f'{label}: common.py 값 미설정 → 비교 건너뜀'))
            continue
        got = db.get(key)
        if got is None or len(got) != len(ref):
            results.append((FAIL, f'{label}: DB 값 형식 오류 ({got})'))
            continue
        diff = max(abs(a - b) for a, b in zip(ref, got))
        if diff > POSE_TOLERANCE:
            results.append((FAIL, f'{label} 설정값이 DB와 다릅니다. 재설정 후 실행해 주세요. '
                                  f'(common={ref}, DB={got})'))
        else:
            results.append((OK, f'{label} 일치'))
    return results


CHECKS = [
    ('분류 기준', check_specs),
    ('고정 좌표', check_fixed_poses),
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
