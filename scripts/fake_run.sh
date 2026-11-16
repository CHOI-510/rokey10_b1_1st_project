#!/usr/bin/env bash
# =====================================================================
# fake_run.sh — 로봇 없이 전체 흐름 시험 (터미널 하나)
#   fake 노드 4개(database, scan, judge, place)를 백그라운드로 띄우고
#   관제(control)는 앞에서 실행한다 (q / s 키 그대로 사용 가능).
#   관제가 끝나거나 Ctrl+C를 누르면 백그라운드 노드도 같이 끈다.
#
# 사용
#   ./scripts/fake_run.sh                         # 기본 (관제: robot_monitor=fake, 대기 3초)
#   ./scripts/fake_run.sh -p progress_timeout_sec:=5.0     # 관제 파라미터 추가 (기본값 대신 이걸 씀)
#   BOX_COUNT=6 CLASSES="A,A,B,C,D,D" ./scripts/fake_run.sh
#   DB_ARGS="-p estop_at:=2" ./scripts/fake_run.sh          # 2번째 박스 뒤 UI 비상정지 흉내
#   PLACE_ARGS="-p miscount_at:=2" ./scripts/fake_run.sh    # 재고 불일치 시험
#   MOTION=1 BOX_COUNT=2 ./scripts/fake_run.sh               # 로봇이 실제로 움직임 (아래 참고)
#
# MOTION=1 (에뮬레이터·실제 로봇, bringup 필요)
#   탐지·판별·적재를 fake 대신 test_*_motion 노드로 띄운다 → 로봇이 HOME에서 50mm씩 위아래로 움직임.
#   DB는 fake 그대로. 관제 기본값: robot_monitor=dsr, 대기 5초.
#   움직이는 중에 비상정지를 걸어 로봇이 멈추는지 확인하는 용도. 작업공간을 비우고 실행할 것.
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

BOX_COUNT="${BOX_COUNT:-4}"
CLASSES="${CLASSES:-A,B,B,D}"
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
RUNNING_PATTERN='(box_control/(fake_database|fake_scan|fake_judge|fake_place|test_scan_motion|test_judge_motion|test_place_motion|control)( |$))'
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
    grep -E "fake_db\]|SetJobStatus" "$LOG_DIR/database.log" 2>/dev/null | tail -n 5 | sed 's/^/  /'
    echo "[fake_run] 종료"
}
trap cleanup EXIT INT TERM

start_bg() {      # start_bg 이름 실행파일 [인자...]
    local name="$1"; shift
    # setsid: 노드마다 자기 프로세스 그룹 → 정리할 때 그룹째 신호를 보냄
    # </dev/null: 백그라운드 노드가 키 입력(q/s)을 가로채지 않게
    setsid ros2 run box_control "$@" >"$LOG_DIR/$name.log" 2>&1 </dev/null &
    local pid=$!
    disown "$pid" 2>/dev/null          # 끝날 때 'Killed' 같은 job 메시지 안 띄움
    PIDS+=("$pid")
    echo "[fake_run] $name 시작 (pid $pid, 로그 $LOG_DIR/$name.log)"
}

# shellcheck disable=SC2086
start_bg database fake_database --ros-args $DB_ARGS
if [ "${MOTION:-0}" = 1 ]; then
    echo "[fake_run] MOTION=1 → 로봇이 실제로 움직입니다 (test_*_motion). 작업공간을 비우세요."
    # shellcheck disable=SC2086
    start_bg scan  test_scan_motion  --ros-args -p robot_setup:=on -p box_count:="$BOX_COUNT" $SCAN_ARGS
    # shellcheck disable=SC2086
    start_bg judge test_judge_motion --ros-args -p robot_setup:=on $JUDGE_ARGS
    # shellcheck disable=SC2086
    start_bg place test_place_motion --ros-args -p robot_setup:=on $PLACE_ARGS
else
    # shellcheck disable=SC2086
    start_bg scan  fake_scan  --ros-args -p box_count:="$BOX_COUNT" -p fake_delay_sec:="$DELAY" $SCAN_ARGS
    # shellcheck disable=SC2086
    start_bg judge fake_judge --ros-args -p class_sequence:="$CLASSES" -p fake_delay_sec:="$DELAY" $JUDGE_ARGS
    # shellcheck disable=SC2086
    start_bg place fake_place --ros-args -p fake_delay_sec:="$DELAY" $PLACE_ARGS
fi

# 서버 4개가 뜰 때까지 대기 (최대 20초). 그 사이 죽은 노드가 있으면 로그를 보여 주고 멈춤
NAMES=(database scan judge place)
echo -n "[fake_run] 서버 준비 대기 "
for _ in $(seq 1 40); do
    for i in "${!PIDS[@]}"; do
        if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then       # 프로세스 자체로 확인 (그룹은 막 띄운 직후엔 아직 없을 수 있음)
            echo
            echo "[fake_run] ${NAMES[$i]} 노드가 시작하자마자 종료됐습니다. 로그 끝부분:" >&2
            tail -n 15 "$LOG_DIR/${NAMES[$i]}.log" | sed 's/^/  /' >&2
            exit 1
        fi
    done
    actions="$(ros2 action list 2>/dev/null)"
    services="$(ros2 service list 2>/dev/null)"
    if grep -q '^/scan$' <<<"$actions" && grep -q '^/judge$' <<<"$actions" \
        && grep -q '^/place$' <<<"$actions" && grep -q '^/get_job_data$' <<<"$services"; then
        echo " 완료"
        break
    fi
    echo -n "."
    sleep 0.5
done
if ! grep -q '^/get_job_data$' <<<"$services" || ! grep -q '^/place$' <<<"$actions"; then
    echo
    echo "[fake_run] 서버가 다 뜨지 않았습니다. 로그를 확인하세요: $LOG_DIR" >&2
fi

# 같은 이름의 노드가 둘 이상이면 멈춤 (다른 터미널·다른 PC가 같은 ROS_DOMAIN_ID에서 띄운 경우 포함)
# → 관제가 엉뚱한 서버에 Goal을 보내 로봇이 예상과 다르게 움직이는 것을 막는다.
DUPS="$(ros2 node list 2>/dev/null | grep -E '^/(database|scan|judge|place|control)$' | sort | uniq -d)"
if [ -n "$DUPS" ]; then
    echo "[fake_run] 같은 이름의 노드가 둘 이상 있습니다: $(echo "$DUPS" | tr '\n' ' ')" >&2
    echo "[fake_run] 다른 터미널의 노드를 끄거나(bash scripts/fake_stop.sh), 다른 PC와 ROS_DOMAIN_ID가 겹치는지 확인하세요." >&2
    echo "[fake_run] 지금 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" >&2
    exit 1
fi

# 관제 (앞에서 실행). 인자를 주면 그 인자만 쓴다.
if [ "$#" -gt 0 ]; then
    CONTROL_ARGS=(--ros-args "$@")
elif [ "${MOTION:-0}" = 1 ]; then
    CONTROL_ARGS=(--ros-args -p robot_monitor:=dsr -p start_delay_sec:=5.0)
else
    CONTROL_ARGS=(--ros-args -p robot_monitor:=fake -p start_delay_sec:=3.0)
fi
echo "[fake_run] 관제 실행: ros2 run box_control control ${CONTROL_ARGS[*]}"
echo
ros2 run box_control control "${CONTROL_ARGS[@]}"
