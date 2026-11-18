#!/usr/bin/env bash
# =====================================================================
# db_check.sh — DB 노드 혼자 시험 (관제·로봇 없이)
#   터미널 1: ros2 run box_database database_main --ros-args -p store:=file
#   터미널 2: ./scripts/db_check.sh
#
#   관제가 보내는 것(서비스 3개, /control_status, /event_log)을 흉내 내고
#   버튼 창구(/estop, /start)를 눌러 본 뒤 state와 로그 파일을 보여 준다.
#   끝나면 재고를 시험 전 값으로 되돌린다.
#   store=supabase인 DB 노드에 하려면 YES=1 (실제 Supabase state 행이 잠깐 바뀜)
# =====================================================================
set -u
PORT="${BOX_DB_PORT:-8765}"
LOG_DIR="${BOX_LOG_DIR:-$HOME/box_logs}"
URL="http://127.0.0.1:$PORT"
JOB="check_$(date +%H%M%S)"
QOS=(--qos-reliability reliable --qos-durability transient_local)
PASS=0; FAIL=0

ok()   { echo "  ✔ $1"; PASS=$((PASS+1)); }
bad()  { echo "  ✘ $1"; FAIL=$((FAIL+1)); }
step() { echo; echo "── $1"; }
state_field() { curl -s "$URL/state" | python3 -c "import json,sys; print(json.load(sys.stdin).get('$1'))"; }
srv() { timeout 10 ros2 service call "$@" 2>&1 | tail -n 1; }

step "0. DB 노드 확인"
HEALTH=$(curl -s --max-time 2 "$URL/health") || { echo "  DB 노드가 없음 → 먼저: ros2 run box_database database_main --ros-args -p store:=file"; exit 1; }
echo "  $HEALTH"
STORE=$(echo "$HEALTH" | python3 -c "import json,sys; print(json.load(sys.stdin)['store'])")
if [ "$STORE" = supabase ] && [ "${YES:-0}" != 1 ]; then
    echo "  store=supabase → 실제 Supabase state가 바뀝니다. 계속하려면 YES=1 ./scripts/db_check.sh"; exit 1
fi
BEFORE=$(curl -s "$URL/state")
INV0=$(echo "$BEFORE" | python3 -c "import json,sys; s=json.load(sys.stdin); print(' '.join(str(s.get(k) or 0) for k in 'abcdu'))")
read -r A0 B0 C0 D0 U0 <<< "$INV0"
echo "  시험 전 재고 A $A0 B $B0 C $C0 D $D0 U $U0"

step "1. get_job_data (관제가 작업 시작 때 부름)"
OUT=$(srv /get_job_data box_interfaces/srv/GetJobData "{}")
echo "$OUT" | grep -q "success=True" && ok "specs·재고·좌표 받음" || bad "get_job_data 실패: $OUT"

step "2. set_job_status RUNNING"
srv /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}" >/dev/null
[ "$(state_field status)" = RUNNING ] && ok "status=RUNNING" || bad "status가 RUNNING 아님"
[ "$(state_field started_at)" != None ] && ok "started_at 기록" || bad "started_at 없음"

step "3. update_inventory (같은 box_id 두 번 → 두 번째는 무시)"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_001', inventory: {a: $((A0+1)), b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(state_field a)" = "$((A0+1))" ] && ok "A $A0 → $((A0+1))" || bad "A 재고가 안 바뀜"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_001', inventory: {a: 99, b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(state_field a)" = "$((A0+1))" ] && ok "중복 box_id 무시" || bad "중복 box_id가 반영됨"

step "4. /control_status (관제 단계·로봇 상태) → state 열"
timeout 10 ros2 topic pub --once -w 1 "${QOS[@]}" /control_status std_msgs/msg/String \
  "{data: '{\"state\": \"SCANNING\", \"job_id\": \"$JOB\", \"box_seq\": 1, \"box_id\": \"${JOB}_002\", \"robot_state_name\": \"MOVING\", \"robot_ok\": true, \"robot_connected\": true, \"robot_hint\": \"\"}'}" >/dev/null 2>&1
sleep 1.5
[ "$(state_field stage)" = SCANNING ] && ok "stage=SCANNING" || bad "stage 안 바뀜 ($(state_field stage))"
[ "$(state_field robot_ok)" = True ] && ok "robot_ok=true, robot_state=$(state_field robot_state)" || bad "robot_ok 안 바뀜"
[ "$(state_field control_seen_at)" != None ] && ok "control_seen_at 기록" || bad "control_seen_at 없음"

step "5. /event_log → 로그 파일 (seq 1,2 보낸 뒤 1을 다시 → 중복 무시)"
for s in 1 2 1; do
  timeout 10 ros2 topic pub --once -w 1 --qos-depth 100 "${QOS[@]}" /event_log box_interfaces/msg/LogEvent \
    "{seq: $s, stamp: '$(date '+%F %T')', job_id: '$JOB', source: 'control', level: 'INFO', code: 'BOX_DONE', expected: -1, measured: -1, message: '시험 로그 $s'}" >/dev/null 2>&1
done
sleep 1
N=$(grep -c '"code": "BOX_DONE"' "$LOG_DIR/$JOB.jsonl" 2>/dev/null || echo 0)
[ "$N" = 2 ] && ok "로그 2줄 ($LOG_DIR/$JOB.jsonl)" || bad "로그 줄 수 $N (기대 2)"

step "6. 버튼 창구 (관제가 없으니 거절이 정상)"
R=$(curl -s -X POST "$URL/estop"); echo "  /estop → $R"
echo "$R" | grep -q '"ok"' && ok "비상정지 응답 옴" || bad "비상정지 응답 없음"
R=$(curl -s -X POST "$URL/start"); echo "  /start → $R"
echo "$R" | grep -q '준비 안 됨' && ok "READY 아님 → 시작 거절" || bad "시작 응답이 예상과 다름"
timeout 10 ros2 topic pub --once -w 1 "${QOS[@]}" /control_status std_msgs/msg/String \
  "{data: '{\"state\": \"READY\", \"job_id\": \"$JOB\", \"robot_state_name\": \"STANDBY\", \"robot_ok\": true}'}" >/dev/null 2>&1
sleep 1
R=$(curl -s -X POST "$URL/start"); echo "  (READY로 바꾼 뒤) /start → $R"
echo "$R" | grep -q '/job_command' && ok "관제 /job_command 없음 → 거절 (관제 상주 모드 전까지 정상)" \
    || echo "  (관제가 /job_command를 받으면 ok:true가 나옴)"

step "7. set_job_status DONE → finished_at"
srv /set_job_status box_interfaces/srv/SetJobStatus "{status: 'DONE'}" >/dev/null
[ "$(state_field status)" = DONE ] && [ "$(state_field finished_at)" != None ] && ok "DONE + finished_at" || bad "DONE 기록 안 됨"

step "8. 정리 (재고 되돌리기)"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_restore', inventory: {a: $A0, b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(state_field a)" = "$A0" ] && ok "재고 원래대로" || bad "재고 되돌리기 실패"

step "결과: 통과 $PASS / 실패 $FAIL"
echo "  지금 state:  curl -s $URL/state | python3 -m json.tool"
echo "  로그 파일:   tail $LOG_DIR/$JOB.jsonl"
if [ "$STORE" = file ]; then
    FOLDER=$(echo "$HEALTH" | python3 -c "import json,sys; print(json.load(sys.stdin)['folder'])")
    echo "  테이블 파일: $FOLDER/{product,loc,state}.json"
    echo "  state에 쓴 기록 (마지막 8줄, $FOLDER/state_updates.jsonl):"
    tail -n 8 "$FOLDER/state_updates.jsonl" 2>/dev/null | sed 's/^/    /'
fi
[ "$FAIL" = 0 ]
