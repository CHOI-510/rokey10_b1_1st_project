#!/usr/bin/env bash
# =====================================================================
# db_check.sh — DB 노드 혼자 시험 (관제·로봇 없이)
#   터미널 1: ros2 run box_database database_main --ros-args -p store:=file
#   터미널 2: ./scripts/db_check.sh
#
#   관제가 보내는 것(서비스 3개, /control_status, /event_log)을 흉내 내고 /state를 확인한다.
#   DB(파일/Supabase)에 쓰이는 건 재고뿐이고, 작업·로봇 상태는 /state(메모리)에만 있어야 한다 (11/19).
#   끝나면 재고를 시험 전 값으로 되돌린다.
#   store=supabase인 DB 노드에 하려면 YES=1 (실제 Supabase inventory 행이 잠깐 바뀜)
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
st()   { curl -s "$URL/state" | python3 -c "import json,sys; s=json.load(sys.stdin); v=s; [v:=v[k] for k in '$1'.split('.')]; print(v)"; }
srv()  { timeout 10 ros2 service call "$@" 2>&1; }
status_pub() {
  timeout 10 ros2 topic pub --once -w 1 "${QOS[@]}" /control_status std_msgs/msg/String "{data: '$1'}" >/dev/null 2>&1
  sleep 1
}

step "0. DB 노드 확인"
HEALTH=$(curl -s --max-time 2 "$URL/health") || { echo "  DB 노드가 없음 → 먼저: ros2 run box_database database_main --ros-args -p store:=file"; exit 1; }
echo "  $HEALTH"
STORE=$(echo "$HEALTH" | python3 -c "import json,sys; print(json.load(sys.stdin)['store'])")
if [ "$STORE" = supabase ] && [ "${YES:-0}" != 1 ]; then
    echo "  store=supabase → 실제 Supabase inventory가 잠깐 바뀝니다. 계속하려면 YES=1 ./scripts/db_check.sh"; exit 1
fi
read -r A0 B0 C0 D0 U0 <<< "$(curl -s "$URL/inventory" | python3 -c "import json,sys; s=json.load(sys.stdin); print(' '.join(str(s[k]) for k in 'abcdu'))")"
echo "  시험 전 재고 A $A0 B $B0 C $C0 D $D0 U $U0"

step "1. get_job_data (관제가 작업 시작 때 부름)"
OUT=$(srv /get_job_data box_interfaces/srv/GetJobData "{}")
echo "$OUT" | grep -q "success=True" && ok "specs·재고·좌표 받음" || bad "get_job_data 실패: $(echo "$OUT" | tail -n 1)"

step "2. /control_status STANDBY → /state status=STARTING, 카운트다운"
status_pub "{\"state\": \"STANDBY\", \"standby_sec\": 600, \"start_at\": $(($(date +%s)+600)), \"robot_ok\": true, \"robot_hint\": \"\"}"
[ "$(st status)" = STARTING ] && ok "status=STARTING" || bad "status $(st status) (기대 STARTING)"
[ "$(st start_at)" != None ] && ok "start_at $(st start_at)" || bad "start_at 없음"
[ "$(st control_ok)" = True ] && ok "control_ok=true" || bad "control_ok 아님"

step "3. set_job_status RUNNING + 작업 단계 → status=RUNNING (메모리)"
srv /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}" >/dev/null
status_pub "{\"state\": \"SCANNING\", \"robot_ok\": true, \"robot_hint\": \"\"}"
[ "$(st status)" = RUNNING ] && ok "status=RUNNING, started_at $(st started_at)" || bad "status $(st status)"

step "4. update_inventory → DB에 재고 (같은 box_id 두 번 → 두 번째는 무시)"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_001', inventory: {a: $((A0+1)), b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(st inventory.a)" = "$((A0+1))" ] && ok "A $A0 → $((A0+1))" || bad "A 재고가 안 바뀜"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_001', inventory: {a: 99, b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(st inventory.a)" = "$((A0+1))" ] && ok "중복 box_id 무시" || bad "중복 box_id가 반영됨"

step "5. 로봇 이상 → robot_ok=false + 안내 문구"
status_pub "{\"state\": \"PLACING\", \"robot_ok\": false, \"robot_hint\": \"비상정지 — 해제 후 리셋\"}"
[ "$(st robot_ok)" = False ] && ok "robot_ok=false, hint '$(st robot_hint)'" || bad "robot_ok 안 바뀜"

step "6. /event_log → 로그 파일 (seq 1,2 보낸 뒤 1을 다시 → 중복 무시)"
for s in 1 2 1; do
  timeout 10 ros2 topic pub --once -w 1 --qos-depth 100 "${QOS[@]}" /event_log box_interfaces/msg/LogEvent \
    "{seq: $s, stamp: '$(date '+%F %T')', job_id: '$JOB', source: 'control', level: 'INFO', code: 'BOX_DONE', expected: -1, measured: -1, message: '시험 로그 $s'}" >/dev/null 2>&1
done
sleep 1
N=$(grep -c '"code": "BOX_DONE"' "$LOG_DIR/$JOB.jsonl" 2>/dev/null || echo 0)
[ "$N" = 2 ] && ok "로그 2줄 ($LOG_DIR/$JOB.jsonl)" || bad "로그 줄 수 $N (기대 2)"

step "7. DONE → READY: 결과가 남는지"
srv /set_job_status box_interfaces/srv/SetJobStatus "{status: 'DONE'}" >/dev/null
status_pub "{\"state\": \"READY\", \"robot_ok\": true, \"robot_hint\": \"\"}"
[ "$(st status)" = DONE ] && [ "$(st finished_at)" != None ] && ok "READY여도 status=DONE 유지 + finished_at" || bad "status $(st status)"

step "8. 웹 창구 (관제가 없으니 거절이 정상)"
R=$(curl -s -X POST "$URL/estop"); echo "  /estop → $R"
echo "$R" | grep -q '"ok"' && ok "비상정지 응답 옴" || bad "비상정지 응답 없음"
R=$(curl -s -X POST "$URL/start_now"); echo "  /start_now → $R"
echo "$R" | grep -q '알 수 없는 명령' && ok "바로 시작은 웹에서 불가 (404)" || bad "start_now가 통과함"

step "9. 정리 (재고 되돌리기)"
srv /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: '${JOB}_restore', inventory: {a: $A0, b: $B0, c: $C0, d: $D0, u: $U0}}" >/dev/null
[ "$(st inventory.a)" = "$A0" ] && ok "재고 원래대로" || bad "재고 되돌리기 실패"

step "결과: 통과 $PASS / 실패 $FAIL"
echo "  지금 화면 값:  curl -s $URL/state | python3 -m json.tool"
echo "  로그 파일:     tail $LOG_DIR/$JOB.jsonl"
if [ "$STORE" = file ]; then
    FOLDER=$(echo "$HEALTH" | python3 -c "import json,sys; print(json.load(sys.stdin)['folder'])")
    echo "  DB 파일:       $FOLDER/{product,loc,inventory}.json  ← 이 셋만 있어야 함 (작업·로봇 상태는 없음)"
    echo "  재고 쓴 기록 (마지막 5줄, $FOLDER/inventory_updates.jsonl):"
    tail -n 5 "$FOLDER/inventory_updates.jsonl" 2>/dev/null | sed 's/^/    /'
fi
[ "$FAIL" = 0 ]
