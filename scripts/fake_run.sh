#!/usr/bin/env bash
# =====================================================================
# fake_run.sh — 로봇 없이 전체 흐름 시험 (터미널 하나)
#   DB 노드(database_main, 파일 모드)와 fake 파트 3개(scan, judge, place)를 백그라운드로 띄우고
#   관제(control)는 앞에서 실행한다 (q / s 키 그대로 사용 가능).
#   관제가 끝나거나 Ctrl+C를 누르면 백그라운드 노드도 같이 끈다.
#
# 사용
#   ./scripts/fake_run.sh                         # 기본 (관제: robot_monitor=fake, 대기 3초)
#   ./scripts/fake_run.sh -p progress_timeout_sec:=5.0     # 관제 파라미터 추가 (기본값 대신 이걸 씀)
#   BOX_COUNT=6 CLASSES="A,A,B,C,D,D" ./scripts/fake_run.sh
#   DB_ARGS="-p estop_at:=2" ./scripts/fake_run.sh          # 2번째 박스 뒤 UI 비상정지 흉내
#   DB_ARGS="-p start_inventory:=[0,5,0,0,0]" ./scripts/fake_run.sh   # 재고 B=5로 시작 (불일치 시험)
#   DB_STORE=auto ./scripts/fake_run.sh                     # Supabase 키가 있으면 Supabase로 (기본은 파일, 실제 데이터 보호)
#   RESIDENT=1 ./scripts/fake_run.sh                        # 관제 상주 모드 (READY에서 시작 대기, 11/18)
#   (11/18: fake_database는 없어지고 DB는 항상 database_main. 파일 모드 ~/box_logs/fake_db/*.json,
#    매번 재고 0으로 시작 = -p start_inventory:=[0,0,0,0,0]. DB_MAIN은 더 이상 필요 없음)
#     → 다른 터미널: curl -s -X POST localhost:8765/start  /  .../cancel  (관제 터미널 s: 시작, READY에서 q: 종료)
#     RESIDENT 없이 실행하면 관제는 -p auto_start:=true (예전처럼 바로 한 번 실행하고 끝)
#     → 다른 터미널: curl -s localhost:8765/state | python3 -m json.tool,  curl -s -X POST localhost:8765/estop
#   PLACE_ARGS="-p miscount_at:=2" ./scripts/fake_run.sh    # 재고 불일치 시험
#   MOTION=1 BOX_COUNT=2 ./scripts/fake_run.sh               # 로봇이 실제로 움직임 (아래 참고)
#   PLACE_MAIN=1 SIM=1 BOX_COUNT=4 ./scripts/fake_run.sh     # 적재만 실제 코드(place_main), 에뮬레이터
#   PLACE_MAIN=1 SIM=1 BOX_COUNT=3 CLASSES="A,U,B" ./scripts/fake_run.sh   # 미분류(U) 포함
#   JUDGE_MAIN=1 PLACE_MAIN=1 BOX_COUNT=1 ./scripts/fake_run.sh  # 실제 로봇: 판별·적재 실제 코드 (측정존에 박스를 손으로 놓음)
#   SCAN_ARGS="-p grasp_fail_at:=2" ./scripts/fake_run.sh    # 탐지 집기 실패 → 재스캔 시험 (11/17)
#   SCAN_MAIN=1 SIM=1 ./scripts/fake_run.sh                  # 탐지만 실제 코드(scan_main), 에뮬레이터 + 가상 박스 2개
#   SCAN_MAIN=1 SIM=1 SIM_BOXES="400,50,50" ./scripts/fake_run.sh   # 가상 박스 위치·높이 지정 ("x,y,높이;...")
#   SCAN_MAIN=1 SIM=1 PLACE_MAIN=1 ./scripts/fake_run.sh     # 탐지·적재 실제 코드, 에뮬레이터
#   SCAN_MAIN=1 BOX_COUNT=1 ./scripts/fake_run.sh            # 실제 로봇: 탐지 실제 코드 (작업공간에 박스를 놓음)
#   JUDGE_ARGS="-p grasp_fail_at:=2" ./scripts/fake_run.sh   # 판별 집기 실패 → 관제 측정존 비우기 → 재스캔
#   PLACE_ARGS="-p grasp_fail_at:=1" ./scripts/fake_run.sh -p robot_monitor:=fake -p clear_zone:=fake_fail
#                                                            # 측정존 비우기 실패 → 작업 종료(ERROR)
#   PLACE_ARGS="-p grasp_fail_at:=1,2,3,4" ./scripts/fake_run.sh  # 연속 4번 → RETRY_LIMIT 종료
#   관제 clear_zone 기본 auto: robot_monitor=dsr면 실제 로봇으로 비우기, 아니면 fake_ok(비운 셈 치고 박스를 되돌림)
#
# 그리퍼 (11/17 Modbus): BOX_GRIPPER 기본 fake(가짜 그리퍼). JUDGE_MAIN=1, 또는 PLACE_MAIN=1에 SIM 없이(실제 로봇)면 modbus.
#
# JUDGE_MAIN=1 (실제 로봇 전용, 11/17): 판별을 box_judge judge_main으로. 탐지는 fake(움직이지 않음) →
#   측정존에 박스를 손으로 놓고 시작. 테이블 기준 파일(~/.rokey_judge_table_ref.json)과 OnRobot 드라이버 끄기 필요.
#   SIM 적재에서 집기 실패 흉내: BOX_FAKE_GRIP_FAILS=2 → 처음 2번 닫기가 빈손
#
# SCAN_MAIN=1 (에뮬레이터·실제 로봇, bringup 필요, 11/17)
#   탐지를 fake_scan 대신 box_scan scan_main으로 띄운다. 판별·적재는 fake(또는 JUDGE_MAIN/PLACE_MAIN).
#   SIM=1 → scan_main에 -p sim_boxes:="$SIM_BOXES" (에뮬레이터는 힘 센서가 없어서 필요. 기본 2개)
#   실제 로봇에서는 SIM 없이: 그리퍼 modbus, 판별·적재가 fake면 측정존에 놓인 박스는 사람이 치운다.
#   탐지 Goal은 박스가 없으면 작업공간 전체(11선)를 다 훑고 끝나서 몇 분 걸린다.
#
# MOTION=1 (에뮬레이터·실제 로봇, bringup 필요)
#   탐지·판별·적재를 fake 대신 test_*_motion 노드로 띄운다 → 로봇이 HOME에서 50mm씩 위아래로 움직임.
#   DB는 database_main 파일 모드. 관제 기본값: robot_monitor=dsr, 대기 5초.
#   움직이는 중에 비상정지를 걸어 로봇이 멈추는지 확인하는 용도. 작업공간을 비우고 실행할 것.
#
# PLACE_MAIN=1 (에뮬레이터·실제 로봇, bringup 필요)
#   탐지·판별은 fake(움직이지 않음), 적재만 box_place place_main으로 띄운다.
#   CLASSES 기본값은 A,B,C (place_main은 D를 받지 않음). 관제 기본값: robot_monitor=dsr, 대기 5초.
#   SIM=1 → place_main에 -p sim_contact:=true (에뮬레이터는 힘 센서가 없어서 필요). 실제 로봇에서는 SIM 없이.
#
# 환경 변수 (기본값)
#   BOX_COUNT=4  CLASSES="A,B,B,D"  DELAY=0.5
#   DB_ARGS=""  SCAN_ARGS=""  JUDGE_ARGS=""  PLACE_ARGS=""
#
# 작업 중 다른 터미널에서 정지 시험
#   ros2 param set /control fake_robot_state 5      # 로봇 안전 정지 흉내
#   ros2 service call /emergency_stop box_interfaces/srv/EmergencyStop "{source: 'cli'}"
#
# 로그: /tmp/box_fake_logs/*.log  (fake 노드 출력)
# 한 번에 하나만 실행한다. 다른 터미널에서 이미 돌고 있으면 시작하지 않는다.
#   (그래도 실행하려면 FORCE=1 → 기존 것을 끄고 시작)
# 남은 노드 강제 정리: ./scripts/fake_stop.sh
# =====================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"          # src/box_project/scripts → 워크스페이스
LOG_DIR=/tmp/box_fake_logs
export RCUTILS_LOGGING_BUFFERED_STREAM=0   # 노드 로그가 파일에 바로 써지게 (준비 확인이 로그 기반)
# 그리퍼: 에뮬레이터·fake 시험은 가짜 그리퍼, 실제 판별·적재(JUDGE_MAIN=1, PLACE_MAIN=1 SIM 없이)는 실제 RG2(Modbus)
if { { [ "${PLACE_MAIN:-0}" = 1 ] || [ "${SCAN_MAIN:-0}" = 1 ]; } && [ "${SIM:-0}" != 1 ]; } || [ "${JUDGE_MAIN:-0}" = 1 ]; then
    export BOX_GRIPPER="${BOX_GRIPPER:-modbus}"
else
    export BOX_GRIPPER="${BOX_GRIPPER:-fake}"
fi
rm -f "${BOX_FAKE_RETURN_FILE:-/tmp/box_fake_return}"   # 관제(fake)가 밀어낸 박스 기록 (fake_scan이 읽음)

BOX_COUNT="${BOX_COUNT:-4}"
if [ "${PLACE_MAIN:-0}" = 1 ]; then
    CLASSES="${CLASSES:-A,B,C}"
else
    CLASSES="${CLASSES:-A,B,B,D}"
fi
DELAY="${DELAY:-0.5}"
# fake 노드의 fake_delay_sec는 float 파라미터 → 4처럼 정수로 주면 노드가 타입 오류로 죽음. 4 → 4.0
[[ "$DELAY" =~ ^[0-9]+$ ]] && DELAY="$DELAY.0"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
else
    echo "[fake_run] $WS_DIR/install/setup.bash 없음 — 먼저 colcon build 하세요." >&2
    exit 1
fi

# 다른 터미널에서 이미 시험 중이면 멈춤 (서로 노드를 끄는 사고 방지)
RUNNING_PATTERN='((box_control/(fake_scan|fake_judge|fake_place|test_scan_motion|test_judge_motion|test_place_motion|control)|box_place/place_main|box_scan/scan_main|box_judge/judge_main|box_database/(database_main|database_template))( |$))'
if pgrep -f "$RUNNING_PATTERN" >/dev/null; then
    if [ "${FORCE:-0}" = 1 ]; then
        echo "[fake_run] FORCE=1 → 실행 중인 노드를 끄고 시작합니다."
        bash "$SCRIPT_DIR/fake_stop.sh"
        if pgrep -f "$RUNNING_PATTERN" >/dev/null; then
            echo "[fake_run] 정리하지 못한 노드가 남아 있습니다. 직접 확인하세요:" >&2
            pgrep -a -f "$RUNNING_PATTERN" | sed 's/^/  /' >&2
            exit 1
        fi
    else
        echo "[fake_run] 이미 실행 중인 fake 노드·관제가 있습니다 (다른 터미널?):" >&2
        pgrep -a -f "$RUNNING_PATTERN" | sed 's/^/  /' >&2
        echo "[fake_run] 그쪽을 끝내거나 ./scripts/fake_stop.sh 후 다시 실행하세요. (끄고 시작: FORCE=1)" >&2
        exit 1
    fi
fi

mkdir -p "$LOG_DIR"
PIDS=()

alive() {          # 이 스크립트가 띄운 노드 중 살아 있는 게 있으면 0
    for pid in "${PIDS[@]}"; do
        kill -0 -- "-$pid" 2>/dev/null && return 0      # 프로세스 그룹 (ros2 run + 실제 노드)
        kill -0 "$pid" 2>/dev/null && return 0           # 그룹이 아직 안 만들어졌을 때
    done
    return 1
}

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "[fake_run] fake 노드 정리 중 ..."
    # 이 스크립트가 띄운 노드만 끈다 (프로세스 그룹 단위 → ros2 run과 실제 노드가 같이 받음)
    for pid in "${PIDS[@]}"; do
        kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    done
    for _ in $(seq 1 30); do          # 최대 3초 정상 종료 대기
        alive || break
        sleep 0.1
    done
    if alive; then                    # Ctrl+C를 무시한 노드 → TERM으로 한 번 더
        for pid in "${PIDS[@]}"; do
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
        done
        for _ in $(seq 1 10); do
            alive || break
            sleep 0.1
        done
    fi
    if alive; then
        echo "[fake_run] 응답 없는 노드 강제 종료"
        for pid in "${PIDS[@]}"; do
            kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null
        done
    fi
    echo "[fake_run] DB 마지막 기록 ($LOG_DIR/database.log):"
    grep -E "\[database\] (SetJobStatus|재고|UI |관제 단계)" "$LOG_DIR/database.log" 2>/dev/null | tail -n 5 | sed 's/^/  /'
    echo "[fake_run] 종료"
}
trap cleanup EXIT INT TERM

start_bg() {      # start_bg 이름 실행파일 [인자...]
    local name="$1"; shift
    # setsid: 노드마다 자기 프로세스 그룹 → 정리할 때 그룹째 신호를 보냄
    # </dev/null: 백그라운드 노드가 키 입력(q/s)을 가로채지 않게
    setsid ros2 run "${PKG:-box_control}" "$@" >"$LOG_DIR/$name.log" 2>&1 </dev/null &
    local pid=$!
    disown "$pid" 2>/dev/null          # 끝날 때 'Killed' 같은 job 메시지 안 띄움
    PIDS+=("$pid")
    echo "[fake_run] $name 시작 (pid $pid, 로그 $LOG_DIR/$name.log)"
}

# DB 노드 (실제 코드). 기본 파일 모드 → ~/box_logs/fake_db/state.json, state_updates.jsonl
#   시험마다 재고 0으로 시작 (DB_ARGS의 start_inventory가 있으면 그게 우선 — 뒤에 온 값이 이김)
echo "[fake_run] DB 노드 database_main (store=${DB_STORE:-file})"
# shellcheck disable=SC2086
PKG=box_database start_bg database database_main --ros-args -p store:="${DB_STORE:-file}" \
    -p start_inventory:="[0,0,0,0,0]" $DB_ARGS
if [ "${MOTION:-0}" = 1 ]; then
    echo "[fake_run] MOTION=1 → 로봇이 실제로 움직입니다 (test_*_motion). 작업공간을 비우세요."
    # shellcheck disable=SC2086
    start_bg scan  test_scan_motion  --ros-args -p robot_setup:=on -p box_count:="$BOX_COUNT" $SCAN_ARGS
    # shellcheck disable=SC2086
    start_bg judge test_judge_motion --ros-args -p robot_setup:=on $JUDGE_ARGS
    # shellcheck disable=SC2086
    start_bg place test_place_motion --ros-args -p robot_setup:=on $PLACE_ARGS
else
    if [ "${SCAN_MAIN:-0}" = 1 ]; then
        if [ "${SIM:-0}" = 1 ]; then
            SIM_BOXES="${SIM_BOXES:-420,100,50;510,-20,60}"
            echo "[fake_run] SCAN_MAIN=1 SIM=1 → 탐지 실제 코드(scan_main, 가상 박스 $SIM_BOXES). 에뮬레이터 전용."
            # shellcheck disable=SC2086
            PKG=box_scan start_bg scan scan_main --ros-args -p sim_boxes:="$SIM_BOXES" $SCAN_ARGS
        else
            echo "[fake_run] SCAN_MAIN=1 → 탐지 실제 코드(scan_main). 로봇이 작업공간을 훑습니다."
            # shellcheck disable=SC2086
            PKG=box_scan start_bg scan scan_main --ros-args $SCAN_ARGS
        fi
    else
        # shellcheck disable=SC2086
        start_bg scan  fake_scan  --ros-args -p box_count:="$BOX_COUNT" -p fake_delay_sec:="$DELAY" $SCAN_ARGS
    fi
    if [ "${JUDGE_MAIN:-0}" = 1 ]; then
        echo "[fake_run] JUDGE_MAIN=1 → 판별 실제 코드(judge_main). 실제 로봇 전용 (힘 센서·RG2 폭·테이블 기준 파일 필요)."
        # shellcheck disable=SC2086
        PKG=box_judge start_bg judge judge_main --ros-args $JUDGE_ARGS
    else
        # shellcheck disable=SC2086
        start_bg judge fake_judge --ros-args -p class_sequence:="$CLASSES" -p fake_delay_sec:="$DELAY" $JUDGE_ARGS
    fi
    if [ "${PLACE_MAIN:-0}" = 1 ]; then
        SIM_ARG=""
        if [ "${SIM:-0}" = 1 ]; then
            SIM_ARG="-p sim_contact:=true"
            echo "[fake_run] PLACE_MAIN=1 SIM=1 → 적재 실제 코드(place_main, 가상 접촉). 에뮬레이터 전용."
        else
            echo "[fake_run] PLACE_MAIN=1 → 적재 실제 코드(place_main). 로봇이 움직입니다. 작업공간을 비우세요."
        fi
        # shellcheck disable=SC2086
        PKG=box_place start_bg place place_main --ros-args $SIM_ARG $PLACE_ARGS
    else
        # shellcheck disable=SC2086
        start_bg place fake_place --ros-args -p fake_delay_sec:="$DELAY" $PLACE_ARGS
    fi
fi

# 서버 4개가 뜰 때까지 대기 (최대 40초). 그 사이 죽은 노드가 있으면 로그를 보여 주고 멈춤
# 확인은 ros2 CLI(데몬) 대신 각 노드 로그의 '준비' 줄로 한다 — 데몬이 걸려도 영향 없음, 바로 판단.
#   box_comm interface가 서버를 띄우면 남기는 줄: "[scan] Action 서버 /scan 준비", "[database] Service 서버 준비"
NAMES=(database scan judge place)
READY=('Service 서버 준비' 'Action 서버 /scan 준비' 'Action 서버 /judge 준비' 'Action 서버 /place 준비')
echo -n "[fake_run] 서버 준비 대기 "
all_ready=0
for _ in $(seq 1 80); do
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then       # 프로세스 자체로 확인 (그룹은 막 띄운 직후엔 아직 없을 수 있음)
            echo
            echo "[fake_run] ${NAMES[$i]} 노드가 시작하자마자 종료됐습니다. 로그 끝부분:" >&2
            tail -n 15 "$LOG_DIR/${NAMES[$i]}.log" | sed 's/^/  /' >&2
            exit 1
        fi
    done
    all_ready=1
    for i in "${!NAMES[@]}"; do
        grep -qF "${READY[$i]}" "$LOG_DIR/${NAMES[$i]}.log" 2>/dev/null || { all_ready=0; break; }
    done
    if [ "$all_ready" = 1 ]; then
        echo " 완료"
        break
    fi
    echo -n "."
    sleep 0.5
done
if [ "$all_ready" != 1 ]; then
    echo
    echo "[fake_run] 40초 안에 서버가 다 뜨지 않았습니다. 로그를 확인하세요: $LOG_DIR" >&2
    for i in "${!NAMES[@]}"; do
        grep -qF "${READY[$i]}" "$LOG_DIR/${NAMES[$i]}.log" 2>/dev/null || echo "  - ${NAMES[$i]}: 준비 로그 없음" >&2
    done
fi

# 같은 이름의 노드가 둘 이상이면 멈춤 (다른 터미널·다른 PC가 같은 ROS_DOMAIN_ID에서 띄운 경우 포함)
# → 관제가 엉뚱한 서버에 Goal을 보내 로봇이 예상과 다르게 움직이는 것을 막는다.
# (데몬이 걸려 5초 안에 답이 없으면 이 확인만 건너뛴다 — 시작을 막지 않음)
NODES="$(timeout 5 ros2 node list 2>/dev/null)"
if [ $? = 124 ]; then
    echo "[fake_run] ros2 데몬 응답 없음 → 중복 노드 확인 건너뜀 (계속 멈추면: pkill -9 -f _ros2_daemon; ros2 daemon start)" >&2
fi
DUPS="$(grep -E '^/(database|scan|judge|place|control)$' <<<"$NODES" | sort | uniq -d)"
if [ -n "$DUPS" ]; then
    echo "[fake_run] 같은 이름의 노드가 둘 이상 있습니다: $(echo "$DUPS" | tr '\n' ' ')" >&2
    echo "[fake_run] 다른 터미널의 노드를 끄거나(bash scripts/fake_stop.sh), 다른 PC와 ROS_DOMAIN_ID가 겹치는지 확인하세요." >&2
    echo "[fake_run] 지금 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" >&2
    exit 1
fi

# 관제 (앞에서 실행). 인자를 주면 그 인자만 쓴다 (auto_start는 RESIDENT로 정함).
if [ "${RESIDENT:-0}" = 1 ]; then
    AUTO_ARGS=(-p auto_start:=false)
else
    AUTO_ARGS=(-p auto_start:=true)
fi
if [ "$#" -gt 0 ]; then
    CONTROL_ARGS=(--ros-args "${AUTO_ARGS[@]}" "$@")
elif [ "${MOTION:-0}" = 1 ] || [ "${PLACE_MAIN:-0}" = 1 ] || [ "${JUDGE_MAIN:-0}" = 1 ] || [ "${SCAN_MAIN:-0}" = 1 ]; then
    CONTROL_ARGS=(--ros-args "${AUTO_ARGS[@]}" -p robot_monitor:=dsr -p start_delay_sec:=5.0)
else
    CONTROL_ARGS=(--ros-args "${AUTO_ARGS[@]}" -p robot_monitor:=fake -p start_delay_sec:=3.0)
fi
echo "[fake_run] 그리퍼: BOX_GRIPPER=$BOX_GRIPPER"
echo "[fake_run] 관제 실행: ros2 run box_control control ${CONTROL_ARGS[*]}"
echo
ros2 run box_control control "${CONTROL_ARGS[@]}"
