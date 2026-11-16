#!/usr/bin/env bash
# =====================================================================
# fake_stop.sh — 남아 있는 fake 노드·관제를 모두 끈다
#   ./scripts/fake_stop.sh            (fake_run.sh가 끝날 때도 자동으로 부름)
# =====================================================================
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

PATTERN='((box_control/(fake_database|fake_scan|fake_judge|fake_place|test_scan_motion|test_judge_motion|test_place_motion|control)|box_place/place_main)( |$))|(ros2 run (box_control (fake_database|fake_scan|fake_judge|fake_place|test_scan_motion|test_judge_motion|test_place_motion|control)|box_place place_main)( |$))'

if pgrep -f "$PATTERN" >/dev/null; then
    [ "$QUIET" = 1 ] || echo "[fake_stop] 정리할 노드:"
    [ "$QUIET" = 1 ] || pgrep -a -f "$PATTERN" | sed 's/^/  /'
    pkill -INT -f "$PATTERN"
    sleep 1
    pkill -9 -f "$PATTERN" 2>/dev/null
    [ "$QUIET" = 1 ] || echo "[fake_stop] 완료"
else
    [ "$QUIET" = 1 ] || echo "[fake_stop] 실행 중인 fake 노드 없음"
fi
exit 0
