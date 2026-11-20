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

`compose.yaml`은 `robot-fastapi:latest`를 빌드하고 같은 이름의 컨테이너를 실행합니다. Docker Desktop의 Images/Containers에서 확인할 수 있습니다. 소스 변경 시 위 명령으로 다시 빌드·교체합니다.

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

## 실제 로봇 실행

fake 시험을 종료하고, 현장에서 작업공간·로봇·그리퍼·좌표가 준비된 후 별도 터미널에서 실행합니다. fake 시험용 환경 변수는 해당 터미널에 적용하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch box_bringup system.launch.py mode:=real
```

기본 시작 대기는 600초입니다. 웹에서는 대기 생략 명령 `start_now`를 보내지 않습니다. 실제 로봇 동작은 fake 시험으로 검증되지 않습니다. 웹 정지는 소프트웨어 요청이며 현장 펜던트 비상정지를 대체하지 않습니다.

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
