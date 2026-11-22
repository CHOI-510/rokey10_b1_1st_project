# UI ↔ FastAPI ↔ rosbridge ↔ ROS 2

```text
브라우저 my-next-app :3000
    │ HTTP /robot/start, /robot/cancel, /robot/stop, /robot/status
    ▼
Docker Desktop의 robot-fastapi :8000
    │ WebSocket ws://host.docker.internal:9090
    ▼
호스트 rosbridge_server (127.0.0.1:9090)
    ├─ call_service /job_command {command: start/cancel, source: ui}
    ├─ call_service /emergency_stop {source: ui}
    └─ subscribe /control_status (std_msgs/msg/String JSON)
           ▼
       control → scan / judge / place → 로봇
```

기본 `ROBOT_BACKEND=rosbridge`입니다. UI의 HTTP API는 유지하고 FastAPI–ROS 사이에 rosbridge를 사용합니다. 브라우저가 ROS에 직접 접근하지 않습니다. launch는 상주하며 시작 버튼은 작업 시작 명령을 보냅니다. 사전 점검·카운트다운·로봇 정지 처리는 기존 관제가 담당합니다. 브라우저를 닫아도 관제는 계속 실행됩니다.

**DB 기능은 HTTP 유지:** 시작 팝업의 재고 조회·설정은 기존 `box_database`의 `http://host.docker.internal:8765/inventory`를 사용합니다. DB에 저장된 작업 결과·이유·시각도 `/state`에서 보충합니다. 실시간 단계·관제 연결 여부는 rosbridge 토픽을 기준으로 하며, rosbridge가 끊겼을 때 HTTP 상태를 대신 사용해 연결된 것처럼 표시하지 않습니다.

## 설치와 실행

아래 명령은 워크스페이스를 `~/ros2_ws`에 둔 예시입니다. 자신의 설치 위치에 맞게 이 경로를 바꿔 사용하세요. `~`는 현재 사용자의 홈 디렉터리이므로 사용자 이름을 명시할 필요가 없습니다.

한 번 설치 (호스트 ROS 2):

```bash
sudo apt install ros-jazzy-rosbridge-suite
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select box_database box_bringup
source install/setup.bash
```

터미널 1 — 로봇 없는 fake 시험:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=179
export ROS_LOG_DIR=/tmp/robot-fastapi-rosbridge/ros_logs
export BOX_LOG_DIR=/tmp/robot-fastapi-rosbridge/box_logs
ros2 launch box_bringup system.launch.py fake:=true gui:=false db_store:=file start_delay:=15.0
```

`system.launch.py`는 기본적으로 rosbridge를 함께 실행합니다. WebSocket은 `127.0.0.1:9090`에서 수신합니다. 이미 rosbridge를 별도 실행했다면 `rosbridge:=false`를 사용하고 ROS_DOMAIN_ID를 동일하게 맞춥니다. 포트 변경은 `rosbridge_port:=9091`입니다.

터미널 2 — API (Docker Desktop):

```bash
cd ~/ros2_ws/src/robot-api
docker context use desktop-linux
docker --context desktop-linux compose up -d --build
curl http://localhost:8000/health
curl http://localhost:8000/robot/status
```

`compose.yaml`은 `robot-fastapi:latest` 이미지로 `robot-project1` API 컨테이너를 실행합니다. `robot-log-sync`는 로컬 JSONL 로그를 Supabase에 전송하는 별도 컨테이너입니다. Docker Desktop의 Images/Containers에서 확인할 수 있습니다.

```bash
docker --context desktop-linux compose logs --tail 30
docker --context desktop-linux compose stop
docker --context desktop-linux compose start
```

터미널 3 — UI:

```bash
cd ~/ros2_ws/src/my-next-app
# .env.local: NEXT_PUBLIC_ROBOT_API_URL=http://localhost:8000
npm run dev
```

http://localhost:3000/dashboard 에서 상태에 `(rosbridge)`가 표시되는지 확인하고 시작 → 재고 확인 → 시작을 누릅니다. fake 박스 처리 결과는 시작 팝업과 임시 DB 로그로 확인합니다. 기존 로그 요약은 예시이며, 재고 카드 등 Supabase 표시는 fake 파일 DB와 별도입니다.

## 실제 로봇 연결 및 실행

아래는 현재 프로젝트 경로 기준입니다. 다른 작업자는 자신의 워크스페이스 경로로 변경합니다.
fake 시험을 종료하고 실제 로봇·RG2 그리퍼의 전원과 네트워크, 작업 좌표를 확인한 뒤 진행합니다.
fake 시험용 `BOX_GRIPPER=fake`, 임시 `BOX_LOG_DIR` 등의 환경 변수가 남아 있지 않은 새 터미널을 사용합니다.

### 1. 호스트에서 ROS2 로봇 관제 실행

기존 관제를 종료한 뒤, 초기 재고 실측 코드가 포함된 **이 GitHub 프로젝트 소스**를 빌드합니다.
상위 워크스페이스의 구버전 `install/setup.bash`만 source하면 `inventory_check:=true`를
지정해도 구버전 관제에는 실측 호출 코드가 없어 점검하지 않을 수 있습니다.
아래 빌드는 GitHub 프로젝트 내부에 별도 build/install을 만들며 로컬 테스트용 소스는 수정하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
# 두산 드라이버와 기존 ROS 인터페이스 의존성
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project
colcon build --base-paths box_comm box_control box_place box_database box_bringup --packages-select box_comm box_control box_place box_database box_bringup
# 반드시 방금 빌드한 프로젝트의 overlay를 마지막에 source
source install/setup.bash
ros2 pkg prefix box_control
ros2 pkg prefix box_bringup
ros2 launch box_bringup system.launch.py mode:=real bringup:=true inventory_check:=true start_delay:=30.0 db_store:=file
```

`ros2 pkg prefix` 결과가 `rokey10_b1_1st_project/install/...` 아래인지 확인합니다.
새 터미널에서도 기존 워크스페이스 설정 다음에 이 프로젝트의 `install/setup.bash`를 source합니다.
정상적으로 실측하면 `초기 재고 점검 시작`, `시작 재고 점검 완료, DB 반영: ...` 로그가 출력됩니다.

이미 빌드한 뒤에는 기존 관제를 종료하고 아래 실행 스크립트를 사용하면 설치 경로를 혼동하지 않습니다.
이 스크립트는 네 패키지의 경로가 GitHub 프로젝트 설치본인지 확인하고, 불일치하면 실행을 중단합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project
bash scripts/run_real_robot.sh
```

경로만 확인하려면 `bash scripts/run_real_robot.sh --check`를 사용합니다. 이 옵션은 로봇을 실행하지 않습니다.

| 옵션 | 동작 |
| --- | --- |
| `mode:=real` | 실제 로봇 모드 사용 |
| `bringup:=true` | 로봇 드라이버도 함께 실행 |
| `inventory_check:=true` | 작업 시작 시 A~D 적재 수량을 실제로 점검해 시작 재고에 반영 |
| `start_delay:=30.0` | 시작 대기 시간을 30초로 설정 |
| `db_store:=file` | ROS2 작업 데이터·재고를 로컬 파일 저장소에서 읽고 저장 |

일반 사전 점검과 초기 재고 실측은 별도 과정입니다. `inventory_check:=false`이면 실제 적재존을 비워도 저장된 이전 재고를 유지합니다. 위 명령은 초기 재고 실측을 켭니다.
`db_store:=file`이어도 아래 로그 전송 컨테이너를 통해 Supabase의 `log`와 `inventory`를 갱신할 수 있습니다.
기본 원본 로그 경로는 `~/box_logs/*.jsonl`이고 로컬 재고 파일은 `~/box_logs/fake_db/state.json`입니다. `fake_db`라는 폴더명은 파일 저장소 이름이며 `mode:=real`의 로봇 동작을 fake 모드로 바꾸지 않습니다.

### 2. FastAPI와 로그 전송 실행

`my-next-app/.env.local`에 Supabase URL과 서버 전용 `SUPABASE_SERVICE_KEY`가 설정되어 있어야 합니다. 로그 기반 재고 갱신에는 Supabase 트리거와 관련 테이블도 필요합니다. 설정은 `../docs/LOG_INVENTORY_SYNC.md`를 참고합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project/robot-api
docker --context desktop-linux compose up -d --no-build robot-fastapi log-sync
curl http://localhost:8000/health
curl http://localhost:8000/robot/status
curl http://localhost:8000/robot/telemetry
```

이 명령은 이미 빌드된 `robot-fastapi:latest` 이미지를 사용합니다.
API 컨테이너는 `robot-project1`, 로그 전송 컨테이너는 `robot-log-sync`입니다.
`/robot/status`의 `robot_connected: true`, `/robot/telemetry`의 `connected: true`와 관절각·TCP 값 수신을 확인합니다.

### 3. 웹 UI 실행 및 작업 시작

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project/my-next-app
npm run dev
```

`http://localhost:3000/dashboard`에 접속해 API·로봇·DB 연결을 확인하고 **시작 → 재고 확인 → 시작**을 누릅니다.
사전 점검과 30초 대기 후 초기 재고 실측 및 작업을 진행합니다. 카운트다운의 **바로 시작**은 대기 시간을 생략하는 `start_now` 요청입니다.
UI 로봇 뷰의 **실시간** 모드는 실제 관절각·BASE TCP를 표시하며 **데모 미리보기**와 구분됩니다.

로그 전송 상태 확인:

```bash
docker --context desktop-linux logs --tail 30 -f robot-log-sync
```

## 의존성과 설정

설치 목록은 `requirements.txt`에서 관리합니다: FastAPI, Pydantic, Uvicorn, `websocket-client`(rosbridge 통신), HTTPX(API 테스트). 하위 의존성은 pip가 함께 설치합니다. API 이미지에 ROS나 rclpy는 설치하지 않고 호스트의 rosbridge가 ROS에 참여합니다.

| 환경 변수 | 호스트 직접 실행 기본값 | Desktop compose |
| --- | --- | --- |
| ROBOT_BACKEND | rosbridge | rosbridge |
| ROSBRIDGE_URL | ws://127.0.0.1:9090 | ws://host.docker.internal:9090 |
| ROSBRIDGE_TIMEOUT | 8초 | 8초 |
| BOX_API_URL | http://127.0.0.1:8765 | http://host.docker.internal:8765 |
| BOX_START_TOKEN | 미설정 | 설정 시 DB 노드와 동일하게 지정 |
| CORS_ORIGINS | localhost:3000, 127.0.0.1:3000의 HTTP 주소 | 동일 |

`ROBOT_BACKEND=box`는 이전 DB HTTP 명령 경로, `mock`은 UI 전용 모의 상태입니다. rosbridge 연결 실패 시 이 경로로 자동 전환하지 않습니다. 응답의 `mode=rosbridge`는 통신 방식을 뜻하며 실제 로봇 여부는 ROS launch의 `fake`/`mode`에 달려 있습니다.

## 포트 충돌과 fake/real 중복 실행

`Unable to start server: [Errno 98] Address already in use`는 이미 실행 중인 rosbridge가 같은 9090 포트를 사용한다는 뜻입니다. `system.launch.py`에 rosbridge가 포함되어 있으므로 보통 별도로 실행하지 않습니다.

- fake 시험을 실행한 터미널에서 Ctrl+C로 전체 launch를 종료한 후 실제 로봇용 launch를 실행합니다. rosbridge만 끄면 fake 관제·DB가 남아 상태와 서비스가 섞일 수 있습니다.
- 같은 ROS 도메인에서 fake와 real 관제를 동시에 실행하지 않습니다. 포트만 바꾸는 것으로 관제 중복 문제가 해결되지는 않습니다.
- 별도 rosbridge만 사용 중이고 기존 관제·DB가 없는 경우에만 `rosbridge:=false`로 시스템을 실행합니다.

```bash
ss -ltnp '( sport = :9090 or sport = :8765 )'
```

launch는 8765(DB HTTP, `BOX_DB_PORT`로 변경 가능)와 9090(rosbridge) 포트를 먼저 검사합니다. 사용 중이면 이유를 출력하고 로봇 노드를 시작하기 전에 중단합니다. 기존 서버를 자동 종료하거나 임의의 서버에 연결하지 않습니다.

## 연결·실패 처리

`/control_status`를 상시 구독하고 연결이 끊기면 구독만 재연결합니다. 메시지가 10초 이상 없으면 상태를 알 수 없음으로 표시하고 시작을 비활성화합니다. 각 명령은 별도 WebSocket을 사용하므로 정지가 다른 명령의 응답 대기에 막히지 않습니다. 명령은 자동 재시도하지 않습니다. 서비스 응답은 요청 ID로 식별하며, 관제 거절은 HTTP 409, rosbridge 서비스 실패·응답 불명은 503으로 반환합니다. 명령 접수 직후 작업 중으로 바꾸지 않고 실제 토픽 상태를 기다립니다.

## 검증

```bash
cd ~/ros2_ws/src/robot-api
docker --context desktop-linux run --rm --network none \
  --mount type=bind,source="$PWD/tests",target=/code/tests,readonly \
  robot-fastapi:latest python -m unittest discover -s tests -v

cd ~/ros2_ws/src/my-next-app
npm run lint
npx tsc --noEmit
```

2026-11-20: rosbridge 코드의 API·프로토콜 테스트 16개 및 UI lint·TypeScript 검사를 통과했습니다. fake 시험 기록은 `/tmp/robot-fastapi-rosbridge/box_logs/`에 저장합니다. 실제 로봇은 현장 준비 전이므로 시험하지 않습니다.

프로토콜 근거: https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md


브라우저에서 rosbridge 경로로 확인한 결과:

- 11:17:18 시작 요청 → 11:18:05 fake 4개 처리 완료, A=1/B=2/C=0/D=1/U=0, 불일치 0건.
- 11:19:33 취소 → READY, 작업 시작 안 함.
- 11:20:26 정지 → STOPPED, `EMERGENCY STOP — by ui (시작 전)`.

API·프로토콜 테스트 16개, DB 상태 테스트 3개, UI lint·TypeScript 검사를 통과했습니다. 현재 UI·API·fake ROS·rosbridge는 실행 중이며 마지막 결과는 STOPPED입니다.

## 실측 로봇 자세 API

`GET /robot/telemetry`는 별도의 읽기 전용 rosbridge 소켓에서 두산 서비스를 조회한 마지막 샘플을 즉시 반환합니다.

- `${ROBOT_SERVICE_NS}/aux_control/get_current_posj`: 6개 관절각, degree
- `${ROBOT_SERVICE_NS}/aux_control/get_current_posx` (`ref: 0`): BASE TCP XYZ(mm), ABC(degree)
- 기본 namespace `/dsr01/dsr_controller2`, 기본 조회 주기 `ROBOT_TELEMETRY_PERIOD=0.1`초. 두 조회는 순차 호출하므로 완전히 동시인 샘플은 아닙니다.
- 관제 상태가 fresh이고 `robot_monitor=dsr`, `robot_hardware_mode=real`, `robot_connected=true`일 때만 조회합니다. fake/mock/virtual 데이터를 실측으로 표시하지 않습니다.
- 응답: `connected`, `joints_deg`, `tcp_mm`, `orientation_deg`, `received_at`, `age_ms`, `frame=BASE`, `message`. 아직 샘플이 없으면 좌표는 null입니다.
- 실패/연결 끊김/2초 이상 오래된 데이터는 `connected=false`, 마지막 성공 샘플은 유지합니다. 명령 연결과 분리되어 시작/비상정지를 기다리게 하지 않습니다.
- 웹 로봇 뷰는 100ms 간격으로 조회합니다. 그리퍼 개폐·물품 보유 여부는 이 서비스에서 제공하지 않으므로 추정하지 않습니다. 화면의 로봇 외형은 URDF 기반 단순 형상입니다.

적용하려면 이 디렉터리의 변경 코드로 FastAPI 이미지를 다시 빌드하고 컨테이너를 교체해야 합니다. ROS2 관제/scan/judge/place 알고리즘 재빌드는 필요 없습니다. rosbridge가 실행 중인 ROS 환경에서 `dsr_msgs2` 서비스 타입을 찾을 수 있어야 하며, 해당 조회 서비스가 허용되어 있어야 합니다. 브라우저는 기존 `NEXT_PUBLIC_ROBOT_API_URL`을 그대로 사용합니다.

### 동작 중 로봇 뷰 수신

로봇 뷰의 실측 관절각은 `/dsr01/joint_states` (`sensor_msgs/JointState`)를 별도
WebSocket으로 구독한다. joint_1~joint_6 이름으로 매핑하고 radian을 degree로 변환하며
그리퍼 관절은 제외한다. source stamp가 오래되거나 중복된 메시지는 새 데이터로 처리하지 않는다.
이 토픽은 긴 이동 서비스 실행 중에도 갱신되므로 TCP 조회 타임아웃과 독립적으로 팔 자세를 표시한다.
TCP는 기존 BASE 조회 서비스를 사용하고 `tcp_connected`, `tcp_age_ms`, `tcp_received_at`을 따로 제공한다.
TCP가 지연되면 UI에 지연을 표시하고 실제 TCP 마커/수치를 숨기며, 관절각은 계속 갱신한다.
관절 구조로 그리는 단순 형상은 그대로 유지하고 TCP를 추정 실측값으로 대신 표시하지 않는다.

## 현재 DB 스키마로 Supabase 모드 실행

GitHub 프로젝트 overlay를 source한 같은 터미널에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/ws_cobot_pjt/ws_dsr/src/rokey10_b1_1st_project/install/local_setup.bash
ros2 launch box_bringup system.launch.py mode:=real bringup:=true inventory_check:=true start_delay:=30.0 db_store:=supabase
```

현재 DB에 맞춘 조회/저장 계약은 `../docs/SUPABASE_DATABASE_MODE.md`를 참조한다.
DB 노드는 서버 `.env.local`의 URL·서비스 키를 읽고 Python 표준 HTTP로 통신한다.
별도 `state` 테이블이나 supabase Python SDK는 필요 없다.
`product.len`은 가로·세로·높이(mm), `weight`는 g이며 NULL 무게는 미측정 0으로 전달한다.
분류는 `inventory.loc_id`와 `inventory.product_name → product.name`으로 연결한다.

## 비상정지 후 UI 복구

UI 복구는 `POST /robot/recover` → `/job_command {command: recover, source: ui}`로 전달한다.
관제가 READY이고 진행 중인 Action·시작 요청이 없으며 move_stop 반복 전송 시간이 끝나야 한다.
관제는 두산 `GetRobotState`로 실제 상태를 읽고 SAFE_OFF(3)일 때만
`/dsr01/dsr_controller2/system/set_robot_control {robot_control: 3}`을 한 번 전송한다.
성공 응답 뒤에도 실제 STANDBY(1)를 다시 확인한 뒤 소프트웨어 정지를 해제한다.
이미 STANDBY면 하드웨어 리셋 명령 없이 소프트웨어 정지만 해제한다.
다른 보호 정지·하드웨어 비상정지 상태는 자동 해제하지 않고 원인을 반환한다.
복구 중 새 비상정지를 누르면 새 요청을 유지하며 복구 완료로 처리하지 않는다.
복구는 새 작업을 자동 시작하거나 이전 작업을 이어서 실행하지 않는다. 시작 버튼으로 새 작업을 요청한다.

관제/통신 코드 변경은 box_comm, box_control, box_database 빌드와 관제 재시작이 필요하다.
API 이미지도 수정본으로 교체해야 한다. 실제 SAFE_OFF 해제는 UI 복구를 눌렀을 때만 요청한다.
