# 소형 박스 자동 분류 및 재고 관리 시스템

두산 협동로봇 **M0609 + OnRobot RG2** 그리퍼만으로 소형 박스의 크기(W·D·H)와 무게를 측정해 A/B/C/D로 분류하고,
분류 결과에 따라 적재하면서 재고를 관리하는 ROS 2 자동화 시스템입니다.
카메라·비전·추가 센서 없이 그리퍼 개구 폭과 로봇 토크만으로 측정합니다.

> ROKEY 10기 B조 1팀 · 협동-1 프로젝트 (ROS 2 Jazzy)

| 문서 | 내용 |
| --- | --- |
| `README.md` | 시스템 소개, 구조, 실행 방법 (이 문서) |
| [`docs/CONTRIBUTING.md`](docs/CONTRIBUTING.md) | 팀 작업 규칙 — 처음 세팅, 파트별 구현 약속, Git 사용법 |
| [`docs/PROGRESS.md`](docs/PROGRESS.md) | 진행 현황 — 파트별 상태, 결정 사항, 남은 이슈 |

---

## 1. 시스템 개요

```
작업공간 스캔 → 물품 감지·픽업 → 측정존 이동 → W·D·H·무게 측정 → A~D 판별 → 분류별 적재 → 재고 반영
   (탐지)                                       (판별)                       (적재)         (DB·UI)
```

- **관제(control)** 가 전체 순서를 지휘하고, 각 파트는 관제의 명령을 받아 자기 동작만 수행합니다.
- 분류 기준값(물품 A~D의 크기·무게)과 재고는 DB에 있고, UI는 DB 값만 표시합니다.

<!-- TODO: 시연 영상 링크 -->

---

## 2. 시스템 구조

```
                 Service                         Action
  database  ◀──────────────▶  control  ◀──────────────▶  scan   (탐지)
  (DB·UI)    GetJobData        (관제)    /scan            judge  (판별)
             UpdateInventory             /judge           place  (적재)
             SetJobStatus                /place
                 Topic
            ◀───────────────
             /event_log  (로그, 관제 → DB 보내기만)
                 Service
            ───────────────▶
             /emergency_stop (UI 비상정지 버튼, DB → 관제)

  control ──▶ 두산 dsr_controller2: motion/move_stop (정지), system/get_robot_state (상태 감시)
```

- 관제 ↔ 탐지/판별/적재: **Action** (로봇이 움직여 오래 걸리고, 진행·결과가 필요)
- 관제 ↔ DB: 응답이 필요하면 **Service** (작업 데이터, 재고 반영, 상태 기록), 알리기만 하면 **Topic** (이벤트 로그 `/event_log`)
- 파트끼리는 직접 통신하지 않고, 파트는 DB와 UI를 모릅니다. 파트 로그도 Feedback으로 관제에 보내면 관제가 `/event_log`로 넘깁니다.
- 비상정지만 방향이 반대입니다(DB → 관제 `/emergency_stop`). 로봇 정지와 상태 감시는 관제가 두산 서비스를 직접 씁니다.
- 실행되는 노드는 5개입니다: `/control`, `/scan`, `/judge`, `/place`, `/database`

<!-- TODO: 노드 구조도, 통신 시퀀스 그림 -->

### 코드 구조: 통신과 동작의 분리

각 노드는 **통신 계층**(`box_comm`)과 **작업 코드**(각 파트 패키지)로 나뉩니다.

- `box_comm`: Action/Service 서버·클라이언트, ROS 메시지 ↔ 파이썬 값 변환, 계약 규칙(Result 보장, 등록된 `error_code`만 허용, `box_id` 중복 무시 등)을 코드로 강제
- 각 파트: 약속된 함수 `run()` 하나만 구현. ROS 메시지 타입을 몰라도 됨

인터페이스가 바뀌어도 `box_comm`만 수정하면 되고, 파트 코드는 로봇 동작에만 집중합니다.

---

## 3. 패키지 구성

```
box_project/
├─ .githooks/        관제 전용 파일 commit 방지 훅 (팀원은 git config core.hooksPath .githooks)
├─ .github/          GitHub Actions: 관제 전용 파일 변경 감시
├─ box_interfaces/   메시지 정의 (.msg .srv .action)
├─ box_comm/         공용 통신 라이브러리 (노드 아님)
│  ├─ common.py       공통 값: 에러 코드, stage, TCP·툴, 고정 좌표, PartError
│  ├─ robot.py        기본 TCP·툴 확인·적용 (파트 Goal 시작 전 자동 실행)
│  └─ control_interface.py, scan_interface.py, judge_interface.py,
│     place_interface.py, database_interface.py
├─ box_control/      관제: control.py (순서) + preflight.py (시작 전 점검)
│                    + 테스트용 fake_*.py 4개, test_scan_motion.py, robot_test.py
├─ box_scan/         탐지: scan_template.py (관제 관리 참고용) + 담당 메인 코드
├─ box_judge/        판별: judge_template.py + 담당 메인 코드
├─ box_place/        적재: place_template.py + 담당 메인 코드
├─ box_database/     DB·UI: database_template.py + 담당 메인 코드
└─ docs/             CONTRIBUTING.md, PROGRESS.md
```

| 패키지 | 역할 | 노드 | 실행 |
| --- | --- | --- | --- |
| `box_interfaces` | 메시지 정의 | - | - |
| `box_comm` | 공용 통신 코드 | - | import 전용 |
| `box_control` | 관제 | `/control` | `ros2 run box_control control` |
| `box_scan` | 탐지 | `/scan` | `ros2 run box_scan scan_template` |
| `box_judge` | 판별 | `/judge` | `ros2 run box_judge judge_template` |
| `box_place` | 적재 | `/place` | `ros2 run box_place place_template` |
| `box_database` | DB·UI | `/database` | `ros2 run box_database database_template` |

---

## 4. 인터페이스 (`box_interfaces` v0.4)

### Action (관제 → 파트)

| 이름 | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `/scan` | `poses` (시작명령) | `success`, `found`, `error_code` | `stage`, `log_level`, `log_msg` |
| `/judge` | `specs: ClassSpec[]`, `poses` | `success`, `result_class`, `error_code` | `stage`, `log_level`, `log_msg` |
| `/place` | `result_class`, `poses` | `success`, `count`(측정 수량), `error_code` | `stage`, `log_level`, `log_msg` |

모든 Goal의 `poses`(`FixedPoses`)는 관제가 DB에서 받은 고정 좌표(HOME, 측정존, 적재 구역 1단 x·y)입니다. 파트는 이 값으로 움직입니다.

`/place`의 `count`는 적재가 **실제로 놓은 위치(단·층)로 잰** `result_class` 수량입니다(v0.4). 단·층·z는 적재 내부 정보라 Result에 넣지 않고 적재 로그에만 남깁니다. 아래 "적재 규칙" 참고.

`/scan`의 `found=False`는 "더 이상 물품 없음"이며 실패가 아닙니다(`success=True`).

### Feedback stage

각 파트는 단계가 바뀔 때마다 Feedback `stage`를 **반드시** 보냅니다. 관제는 마지막 stage와 받은 시각으로 진행 상황을 추적하고, 중단될 때 어느 단계에서 멈췄는지 기록합니다. 목록은 `box_comm/common.py`의 `STAGES`에 있으며, 목록에 없는 값을 보내면 경고 로그가 남습니다.

| Action | stage 순서 |
| --- | --- |
| `/scan` | `space_scan` → `picking` → `moving` → `placing` |
| `/judge` | `measuring_length1` → `measuring_length2` → `measuring_length3` → `measuring_weight` → (`measuring_materiality`, 옵션) → `judging` |
| `/place` | `picking` → `moving` → `placing` |

Feedback의 `log_level`·`log_msg`는 파트가 `report.log('메시지', 'WARN')`으로 채웁니다. 이 칸이 채워진 Feedback만 관제가 `/event_log`로 넘기고, `report(stage)`로 진행만 알린 Feedback은 로그로 올라가지 않습니다.

모든 Action은 맨 앞에 `robot_setup`(기본 TCP·툴 확인, `box_comm`이 자동으로 보냄)이 먼저 옵니다.
판별의 길이 측정은 물품 방향에 따라 가로·세로·높이가 달라질 수 있어 번호로 구분합니다.

### Service (관제 → DB)

| 이름 | Request | Response | 호출 시점 |
| --- | --- | --- | --- |
| `/get_job_data` | 없음 | `success`, `specs`, `inventory`, `poses`, `message` | 작업 시작 시 |
| `/update_inventory` | `box_id`, `inventory` | `success`, `message` | 적재완료마다 (같은 `box_id`는 한 번만 반영) |
| `/set_job_status` | `status`, `reason` | `success` | 작업중, 작업완료, 오류중단·정지 시 |

### Service (DB → 관제)

| 이름 | Request | Response | 호출 시점 |
| --- | --- | --- | --- |
| `/emergency_stop` | `source` (`ui`, `cli` …) | `success`, `message` | UI 비상정지 버튼. DB는 `self.node.request_emergency_stop('ui')` 한 줄로 호출 |

### Topic (관제 → DB)

| 이름 | 메시지 | 내용 |
| --- | --- | --- |
| `/event_log` | `LogEvent` | 이벤트 로그. 관제가 보내기만 함(응답 없음, 누락 가능). QoS reliable + transient_local(최근 100개) |

`code`(`box_comm/common.py` `LOG_CODES`): `PART_LOG`(파트 `report.log`), `JOB_START`, `BOX_DONE`, `COUNT_MISMATCH`, `JOB_ERROR`, `JOB_STOPPED`, `JOB_CANCELED`, `JOB_DONE`
확인: `ros2 topic echo /event_log` · 시연 기록: `ros2 bag record /event_log`

### 메시지

| 메시지 | 필드 |
| --- | --- |
| `Inventory` | `a`, `b`, `c`, `d`: int32 — 분류별 재고 수 |
| `ClassSpec` | `class_id`: string, `width_mm`, `depth_mm`, `height_mm`, `weight_g`: float32 |
| `FixedPoses` | `available`: bool, `home_j`: float64[6] (deg), `measure_zone`: float64[3] (x, y, z mm), `place_zones`: `PlaceZone[]` — 고정 좌표. **기준은 DB**. `GetJobData`로 받아 모든 Action Goal에 실어 보냄 |
| `LogEvent` | `stamp`, `job_id`, `source`(control/scan/judge/place), `stage`, `level`(INFO/WARN/ERROR), `code`, `box_id`, `class_id`: string, `expected`, `measured`: int32 (-1 = 해당 없음), `message`: string |
| `PlaceZone` | `class_id`: string, `x`, `y`: float64 (mm) — 클래스별 적재 구역의 **1단** 위치 (v0.4) |

### 공통 값

| 항목 | 값 |
| --- | --- |
| `status` | `BEFORE`, `RUNNING`, `DONE`, `ERROR`, `STOPPED` (`common.JOB_STATUS`) |
| `reason` | `ERROR`·`STOPPED`일 때 UI에 보일 이유 (`common.ERROR_TEXT`, 예: `Fail to pick`, `Zone full (B)`, `EMERGENCY STOP`). UI 표시: `{status} : {reason}` |
| `result_class` | `A`, `B`, `C`, `D` |
| `error_code` | 성공 `""` / 실패 `NOT_FOUND`, `GRASP_FAILED`, `OVER_FORCE`, `MEASURE_FAILED`, `NO_SLOT`, `PLACE_FAILED`, `ROBOT_SETUP_FAILED` |
| 좌표·단위 | 로봇 base 기준, mm |

### 적재 규칙 (v0.4, `box_comm/common.py` `STACK`)

- 구역 하나에 같은 클래스만 쌓습니다(A 물품 → A 구역).
- **1단(첫 자리)** 에 1층부터 **3층**까지 쌓고, 다 차면 **다음 자리(2단)** 에 다시 1층부터 쌓습니다. 구역당 **2자리** (`STACK` = 3층 × 2자리, 최대 6개), 넘으면 `NO_SLOT`.
- 다음 자리 위치는 적재 설정 `box_place/config/place_config.json`의 `slot_dx_mm`(현재 **−x 방향 80mm**, y는 같음)로 정합니다.
- `STACK`과 적재 설정(`max_layers`, `slot_count`)은 같아야 합니다. 다르면 `place_main`이 시작하지 않습니다.
- 적재는 **매번 1단부터** 박스를 쥔 채 위에서 내려가, 더 내려가지 않는 높이(z)로 층을 판단합니다. 5층 높이에서 닿으면 그 단은 꽉 찬 것이므로 놓지 않고 다음 단으로 갑니다.
- 놓은 위치로 수량을 계산합니다: `수량 = (단-1) × 4 + 층` (예: 2단 2층 → 6개). 앞 단은 모두 찼다고 봅니다.
- 관제·DB는 **1단 x·y와 수량만** 알고, 단 간격·z·박스 높이는 적재 파트만 가집니다.
- UI 적재 이미지는 수량에서 바로 그립니다: `stack_layout(5)` → `[3, 2]` (1자리 3층, 2자리 2층).

---

## 5. 관제 흐름

```
BEFORE        GetJobData로 분류 기준(specs), 재고, 고정 좌표(poses, DB 기준) 수신
PREFLIGHT     시작 전 점검 (분류 기준, DB 고정 좌표 유무·형식, 적재 구역 A~D) → FAIL이면 시작 안 함
STANDBY       시작 대기 카운트다운 (작업공간 안전 확인, q: 취소 / s: 바로 시작)
RUNNING       SetJobStatus(RUNNING)
  ┌─ SCANNING    /scan Goal ──▶ found=False(물품 없음)면 관제가 작업완료로 판단
  │  SCAN_DONE
  │  JUDGING     /judge Goal(specs)
  │  JUDGE_DONE  result_class 수신
  │  PLACING     /place Goal(result_class, poses)
  │  PLACE_DONE  측정 수량(count) 수신 → 기대값(+1)과 비교·기록 → UpdateInventory(box_id, 측정값 반영 재고)
  └─ 반복
DONE          SetJobStatus(DONE)

작업 중 오류  → ERROR   SetJobStatus(ERROR, "Fail to pick")       공정 오류 — 로그 확인 후 재시작
로봇 정지     → STOPPED SetJobStatus(STOPPED, "EMERGENCY STOP")  로봇이 멈춤 — 현장 확인 후 복구
```

| 상태 | 한글 | DB 기록 |
| --- | --- | :-: |
| `BEFORE` | 작업전 | ✓ |
| `PREFLIGHT` | 사전점검 | |
| `STANDBY` | 시작대기 | |
| `RUNNING` | 작업중 | ✓ |
| `SCANNING` / `SCAN_DONE` | 스캔중 / 스캔완료 | |
| `JUDGING` / `JUDGE_DONE` | 판별중 / 판별완료 | |
| `PLACING` / `PLACE_DONE` | 적재중 / 적재완료 | |
| `DONE` | 작업완료 | ✓ |
| `ERROR` | 오류중단 (공정 오류, 관제가 판단해서 멈춤) | ✓ |
| `STOPPED` | 정지 (로봇이 물리적으로 멈춤 — 비상정지, 안전 정지) | ✓ |

- **중단은 두 가지로 통일합니다.**
  - **시작 전 중단** (`RUNNING` 전): 작업 데이터 수신 실패, 사전 점검 FAIL, 카운트다운 중 `q` → 모두 같은 메시지 `작업을 시작하지 않습니다 [단계]: 이유`로 끝나고, 로봇은 움직이지 않으며 DB는 `BEFORE` 그대로입니다.
  - **작업 중 중단** (`RUNNING` 이후, `abort_job()`): 에러 코드로 `ERROR`/`STOPPED`를 정하고 DB에 `status`와 `reason`을 기록합니다(예: `ERROR : Fail to pick`). 로그에는 `단계에서 중단: error_code | 마지막 stage`가 남습니다. **로봇에 정지 명령을 보내지는 않습니다** (정지 기능은 이슈 #1·#3·#12에서 추가).
  - `STOPPED`가 되는 코드(`common.STOP_CODES`: `EMERGENCY_STOP`, `ROBOT_SAFETY_STOP`, `ROBOT_EMERGENCY_STOP`, `ROBOT_DISCONNECTED`)는 정의만 되어 있고, 감지 경로는 정지 기능 단계에서 연결합니다.
- `-ING` 상태는 Goal을 보내기 직전, `_DONE` 상태는 Result를 받은 직후에 바뀝니다.
- 단계 상태는 관제 내부에만 있고, DB에는 `BEFORE` / `RUNNING` / `DONE` / `ERROR` / `STOPPED`와 재고만 기록합니다.
- 상태 값은 영어(`State` Enum)이고, 로그에는 `===== SCANNING (스캔중) =====`처럼 한글을 함께 표시합니다.
- 스캔은 물품 유무(`found`)만 알리고, 작업완료 여부는 관제가 판단합니다.
- **재고 검증 (v0.4)**: 관제는 적재가 잰 수량(`count`)을 기대값(적재 전 재고의 `result_class` 수량 + 1)과 비교합니다. DB에는 **항상 측정값**을 반영하고(실제 적재 상태가 기준), 다르면 `재고 불일치` 경고 로그(기대·측정·차이)를 남긴 뒤 작업을 계속하며, 작업이 끝나면 불일치 건수를 요약해서 출력합니다. 다른 클래스 수량은 그대로 둡니다.
- **중단 기록**: 오류로 중단되면 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 로그로 남깁니다. (예: `SCANNING (스캔중) 단계에서 중단: GRASP_FAILED | /scan 마지막 stage="picking" (4.2초 전)`)
- `box_id`는 관제가 `<작업 시작 시각>_<순번>` 형식으로 붙입니다. (예: `20261114_130512_001`)
  관제를 다시 실행해도 이전 작업과 겹치지 않으므로, DB 노드를 켜 둔 채 관제만 재실행해도 됩니다.

### 사전 점검 (PREFLIGHT) — `box_control/preflight.py`

| 점검 | FAIL (시작 안 함) | WARN (진행) |
| --- | --- | --- |
| 분류 기준 | A~D 중 빠진 것, 0 이하 값 | |
| 고정 좌표 | DB에 좌표 없음 → "DB에 고정 좌표가 없습니다. DB에 HOME·측정존 좌표를 설정한 뒤 실행해 주세요.", 형식 오류 | 측정존이 (0, 0, 0) |
| 적재 구역 | A~D 중 1단 x·y가 없는 구역 | 구역 좌표가 (0, 0) |

고정 좌표(HOME, 측정존, 적재 구역 1단 x·y)는 **DB가 기준**입니다(v0.3, 적재 구역은 v0.4). 관제는 `GetJobData`로 받은 DB 값으로 자기 좌표를 **무조건 덮어쓰고**(비교·중단 없음, 로그만 남김), 그 값을 `/scan`, `/judge`, `/place` Goal에 매번 실어 보내고, 각 파트는 그 값(`self.poses`)으로 움직입니다. `box_comm/common.py`의 `FIXED_POSES`는 **기본값**으로, Goal에 좌표가 없을 때(팀원이 혼자 `send_goal`로 테스트할 때)만 쓰입니다.
점검을 추가할 때는 `preflight.py`에 함수를 만들고 `CHECKS`에 넣습니다.

---

## 6. 로봇 설정 (TCP·툴)

TCP와 툴은 로봇 컨트롤러에 **하나만** 걸리는 전역 설정입니다. 탐지·판별·적재가 같은 로봇을 번갈아 쓰므로, 앞 파트가 바꾼 설정이 남거나 (특히 가상 모드 에뮬레이터에서) TCP가 등록돼 있지 않으면 **플랜지 기준으로 움직여 TCP 오프셋(208.4mm)만큼 더 내려가 바닥에 부딪힙니다.**

그래서 `box_comm/robot.py`가 **각 파트가 Goal을 받으면 `run()` 직전에** 기본 TCP·툴을 확인하고 적용합니다. 관제와 함께 돌리든 팀원이 혼자 테스트하든 똑같이 동작하고, 팀원 코드에는 아무것도 넣지 않아도 됩니다.

| 항목 | 값 (`common.py`) |
| --- | --- |
| TCP | `GripperDA_v1`, 오프셋 `[0, 0, 208.4, 0, 0, 0]` |
| 툴 | `Tool Weight`, 1.530 kg, 무게중심 `[13.52, 79.74, 0.12]` mm |

동작 (TCP·툴 각각)
1. 현재 이름 조회 → 기본값이면 끝
2. 다르면 적용 → 다시 조회해서 확인
3. 그래도 다르면 (미등록으로 판단) 수동 모드 → 등록 → 적용 → 자동 모드 → 확인
4. 최종 확인 실패 → `run()`을 부르지 않고 `ROBOT_SETUP_FAILED`로 Result

파트 노드 파라미터 `robot_setup`

| 값 | 동작 | 쓰는 때 |
| --- | --- | --- |
| `auto` (기본) | 두산 서비스가 있으면 확인·적용, 없으면 경고 후 건너뜀 | 개발, fake 테스트 |
| `on` | 두산 서비스가 없거나 확인 실패면 Goal 실패 | **실제 로봇, 시연** |
| `off` | 하지 않음 | 특수한 경우 |

```bash
ros2 run box_scan scan_template --ros-args -p robot_setup:=on
```

파트는 자기 `run()` 안에서 필요하면 다른 TCP로 바꿔도 됩니다. 다음 Goal에서 기본값으로 다시 맞춰집니다.

---

## 7. 빌드와 실행

### 요구 환경

- Ubuntu 24.04, ROS 2 Jazzy
- 두산 로봇 패키지(`doosan-robot2`)가 빌드된 워크스페이스 — 이 저장소는 그 워크스페이스의 `src` 안에 둡니다.

### 빌드

```bash
cd ~/ws/src
git clone https://github.com/CHOI-510/rokey10_b1_1st_project.git box_project
cd ~/ws
colcon build --symlink-install --packages-up-to box_control box_scan box_judge box_place box_database
source install/setup.bash
```

### 실행

서버 4개를 먼저 띄우고 관제를 마지막에 실행합니다.

```bash
ros2 run box_database database_template
ros2 run box_scan scan_template
ros2 run box_judge judge_template
ros2 run box_place place_template
ros2 run box_control control
```

관제는 실행되면 **시작 대기(STANDBY)** 카운트다운을 띄웁니다. 로봇이 움직이기 전에 작업공간에 사람이나 파손 위험이 있는 물품이 없는지 확인하는 시간입니다.

```
  ⚠ 10초 후에 작업을 시작합니다.
  ⚠ 작업공간 안에 사람이나 파손 위험이 있는 물품이 없는지 확인하세요.
  중단하려면 q, 바로 시작하려면 s 를 누르세요. 10초 동안 입력이 없으면 자동으로 시작합니다.
  시작까지 7초 ...  (q: 중단 / s: 바로 시작)
```

| 키 | 동작 |
| --- | --- |
| `q` | 작업 시작 취소 (DB 상태는 `BEFORE` 유지, 관제 종료) |
| `s` | 바로 시작 |
| 입력 없음 | 시간이 지나면 자동 시작 |

- 대기 시간은 `start_delay_sec` 파라미터로 바꿉니다. 기본 10초(테스트용), 실제 운용은 10분을 상정합니다.
  `ros2 run box_control control --ros-args -p start_delay_sec:=600.0`
- `0`으로 주면 대기 없이 바로 시작합니다.
- 관제가 터미널 입력을 받을 수 없는 환경(launch 등)에서는 `q` 없이 대기 후 시작합니다. 비상정지 서비스는 대기 중에도 동작합니다(시작 안 함).

### 적재 실제 로봇 실행 (`place_main`)

적재 실제 코드는 `box_place/box_place/place_main.py`, 설정은 `box_place/config/place_config.json`입니다 (빌드하면 `share/box_place/config/`에 설치).

```bash
ros2 run box_place place_main                                        # 기본 설정 파일
ros2 run box_place place_main --ros-args -p show_force:=true         # 하강 중 힘·Z 출력
ros2 run box_place place_main --ros-args -p clear_pending:=true      # 멈춘 뒤 확인 완료 (아래)
ros2 run box_place place_main --ros-args -p sim_contact:=true        # 에뮬레이터 전용 (아래)
```

- 지원 클래스는 설정에 있는 것만입니다 (현재 **A/B/C**). D가 오면 `PLACE_FAILED` (`D: place_config.json에 설정이 없습니다`).
- Goal의 측정존이 `(0,0,0)`이면 움직이지 않고 실패합니다 → DB에 실제 측정존·구역 좌표가 있어야 합니다.
- 정지: 모든 이동 전과 힘 제어 하강 중에 관제 정지 요청을 확인합니다 → `CANCELED`로 빠져나오며 힘·순응제어 해제 + `move_stop`. 하강 중에는 1초마다 keepalive를 보내 진행 감시가 오인하지 않습니다.
- 측정존 z는 **박스를 쥐는 TCP 높이**입니다. 현재 측정존 `[300, -160, 30]` (z 30 = 적재 1층 첫 접촉 Z, 측정존 바닥이 적재 구역 바닥과 같은 높이라는 가정).
- **`sim_contact` (에뮬레이터 전용)**: 에뮬레이터는 힘 센서 값이 없어 '닿을 때까지 하강'이 끝나지 않습니다. `true`면 힘 제어 대신 가상으로 쌓인 높이까지 `movel`로 내려가 닿았다고 칩니다(층 판단·만재 시 다음 자리·수량 계산은 원래 코드). 가상 적재 수는 노드가 켜질 때 0, 상태 파일은 `~/.ros/place_sim_state.json`으로 분리. **실제 로봇에서는 켜지 않습니다.**
- **멈춘 뒤 복구**: 작업 중 멈추면 상태 파일(`~/.ros/place_real_v04_state.json`)에 미완료 작업이 남아 다음 적재를 거부합니다. 그리퍼(물건을 쥐고 있는지)와 적재 상태를 확인하고 `-p clear_pending:=true`로 한 번 실행하세요.

### 정지 기능 (비상정지 · 로봇 정지 감지 · 진행 감시)

관제의 `abort_job()`은 작업을 끝내는 처리이고, 로봇을 멈추는 것은 아래 정지 기능이 합니다.

| 계기 | 언제 | 관제 동작 | DB 기록 |
| --- | --- | --- | --- |
| `/emergency_stop` (UI 버튼, 터미널) | 언제든 (시작 전이면 시작 안 함) | 두산 `move_stop` 반복 + 진행 중인 파트 Goal cancel | `STOPPED : EMERGENCY STOP — by ui` |
| 로봇 상태가 STANDBY·MOVING이 아님 (안전 정지, 펜던트 비상정지 등) | 작업 중 (`robot_monitor`) | 같음 | `STOPPED : Robot safety stop — robot_state=SAFE_STOP` 등 |
| 로봇 상태 확인 3초 연속 실패, `robot_disconnection` | 작업 중 (`robot_monitor=dsr`) | 같음 | `STOPPED : Robot disconnected` |
| 파트 Feedback이 `progress_timeout_sec` 동안 없음 | 작업 중 | 같음 | `ERROR : No progress (judge) — …` |

- 파트는 cancel을 받으면 다음 `report(stage)`·`report.check()`에서 `CANCELED`로 빠져나옵니다. `move_stop` 뒤에도 파트가 다음 동작을 보낼 수 있어 관제가 3초 동안 `move_stop`을 반복합니다(`common.SAFETY`).
- 두산 알람(`error` 토픽)은 이벤트 로그(`ROBOT_ALARM`)로 남고, 로봇 정지 이유에 덧붙습니다.
- 복구(로봇 안전 정지 해제)는 아직 수동입니다(펜던트). 해제 후 관제를 다시 실행합니다.

| 관제 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `robot_monitor` | `off`(기본) / `dsr` / `fake` | `dsr`: 실제 로봇·에뮬레이터에서 `get_robot_state`를 0.5초마다 확인. `off`면 사전 점검에서 WARN |
| `fake_robot_state` | int (기본 1 = STANDBY) | `robot_monitor:=fake`일 때 로봇 상태 흉내. 실행 중에 `ros2 param set`으로 바꿈 |
| `progress_timeout_sec` | float (기본 0 = 끔) | 파트 Feedback 감시 시간. 파트 동작 중 가장 긴 구간보다 넉넉하게 (예: 60) |

```bash
# 실제 로봇·에뮬레이터
ros2 run box_control control --ros-args -p robot_monitor:=dsr -p progress_timeout_sec:=60.0

# 비상정지 버튼 대신 터미널에서
ros2 service call /emergency_stop box_interfaces/srv/EmergencyStop "{source: 'cli'}"

# 로봇 안전 정지 흉내 (로봇 없이, robot_monitor:=fake)
ros2 run box_control control --ros-args -p robot_monitor:=fake -p start_delay_sec:=3.0
ros2 param set /control fake_robot_state 5      # 5 = SAFE_STOP, 6 = EMERGENCY_STOP, 1 = STANDBY
```

| 환경 | 시험할 수 있는 것 |
| --- | --- |
| 로봇 없이 (fake 노드) | 비상정지 서비스·`fake_database estop_at`, `robot_monitor:=fake` 상태 변경, 진행 감시 |
| 에뮬레이터 (`mode:=virtual`) | 위 + `robot_monitor:=dsr`로 실제 상태 읽기, `move_stop` 동작. 충돌·토크 안전 정지는 거의 안 생김 |
| 실제 로봇 | 위 + 펜던트 비상정지 버튼 → `STOPPED : Pendant emergency stop`, 충돌 감지 안전 정지 |

### 로봇 없이 실행 (fake)

`box_control`의 fake 노드로 전체 흐름을 로봇 없이 확인할 수 있습니다.

**터미널 하나로 (추천)** — fake 노드 4개는 백그라운드, 관제는 앞에서 실행. 관제가 끝나거나 Ctrl+C면 모두 정리됩니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/box_project
./scripts/fake_run.sh                                        # 기본: robot_monitor=fake, 대기 3초
BOX_COUNT=6 CLASSES="A,A,B,C,D,D" ./scripts/fake_run.sh
DB_ARGS="-p estop_at:=2" ./scripts/fake_run.sh               # 2번째 박스 뒤 UI 비상정지 흉내
PLACE_ARGS="-p miscount_at:=2" ./scripts/fake_run.sh         # 재고 불일치
./scripts/fake_run.sh -p robot_monitor:=fake -p progress_timeout_sec:=5.0   # 관제 파라미터 직접 지정
MOTION=1 BOX_COUNT=2 ./scripts/fake_run.sh                   # 탐지·판별·적재를 test_*_motion으로 (로봇이 위아래로 움직임)
PLACE_MAIN=1 SIM=1 BOX_COUNT=5 CLASSES="A,A,A,A,B" ./scripts/fake_run.sh   # 적재만 place_main, 에뮬레이터 (가상 접촉)
PLACE_MAIN=1 BOX_COUNT=1 CLASSES="A" ./scripts/fake_run.sh   # 적재만 place_main, 실제 로봇 (실제 구역 좌표 필요)
./scripts/fake_stop.sh                                       # 남은 노드 강제 정리
```

fake 노드 출력은 `/tmp/box_fake_logs/*.log`에 남습니다 (`tail -f /tmp/box_fake_logs/database.log`).

- `MOTION=1`, `PLACE_MAIN=1`은 bringup(에뮬레이터 또는 실제 로봇)이 필요하고, 관제 기본값이 `robot_monitor=dsr`, 대기 5초로 바뀝니다.
- `PLACE_MAIN=1`: 탐지·판별은 fake(움직이지 않음), 적재만 `box_place place_main`. `CLASSES` 기본값 `A,B,C`(D 미지원). `SIM=1`이면 `-p sim_contact:=true`.
- `PLACE_MAIN=1`로 멈췄다가 다시 돌릴 때는 미완료 기록부터 지웁니다: 에뮬레이터 `rm -f ~/.ros/place_sim_state.json`, 실제 로봇은 그리퍼·적재 상태 확인 후 `PLACE_ARGS="-p clear_pending:=true"`.

**터미널을 따로 띄울 때**

```bash
ros2 run box_control fake_database
ros2 run box_control fake_scan --ros-args -p box_count:=4 -p fake_delay_sec:=0.3
ros2 run box_control fake_judge --ros-args -p class_sequence:="A,B,B,D"
ros2 run box_control fake_place
ros2 run box_control control --ros-args -p start_delay_sec:=3.0     # 테스트할 때는 대기를 짧게
```

| 노드 | 파라미터 |
| --- | --- |
| `fake_scan` | `box_count` (박스 개수, 기본 3), `fake_delay_sec` |
| `fake_judge` | `class_sequence` (`"A,B,B,D"`처럼 주면 그 순서로 판별, 비우면 무작위), `fake_delay_sec` |
| `fake_place` | `fake_delay_sec`, `fake_start` (A~D 구역에 처음부터 쌓인 개수, 기본 `[0,0,0,0]`), `miscount_at` (N번째 적재에서 수량을 일부러 +1 틀리게 측정 — 관제 불일치 처리 테스트용, 기본 0=끔) |
| `fake_database` | `pose_offset` (고정 좌표에 값을 더해서 반환 → DB 값이 Goal로 전달되는지 확인), `poses_available` (False면 좌표 없음 → 사전 점검 FAIL), `start_inventory`, `estop_at` (N번째 박스 반영 직후 UI 비상정지 버튼을 누른 것처럼 동작, 기본 0=끔) |

fake와 실제 노드는 같은 Action/Service 이름을 쓰므로 **동시에 띄우지 않습니다.**
파트 하나만 실제로 붙여 보려면 그 파트만 실제 노드로, 나머지는 fake로 띄웁니다.

### 로봇 연동 점검 (관제 테스트 도구)

두산 bringup(가상 또는 실제)을 띄운 상태에서 TCP·툴 자동 확인과 로봇 동작 구조를 점검합니다.

| 도구 | 내용 |
| --- | --- |
| `test_scan_motion` | `scan_interface` + 실제 두산 동작(`connect_dsr`, `movej`/`movel`)이 들어간 `/scan` 노드. Goal마다 HOME → 위로 50mm → 아래로 → HOME. **팀원이 실제 동작을 넣을 때 참고할 예시 코드** |
| `test_judge_motion` · `test_place_motion` | 같은 구조의 `/judge`·`/place` 노드. 판별은 x ±30mm, 적재는 y ±30mm로 움직여 눈으로 구분. 판별 결과는 A→B→C→D 순서, 적재는 클래스별 카운터 +1로 단·층 반환 |
| `robot_test` | `/scan` Goal을 반복해서 보내며 TCP·툴 적용을 점검하고 PASS/FAIL 표 출력. `-p reapply_test:=false`면 컨트롤러 설정을 바꾸는 3번 항목을 건너뜀 |

```bash
# 로봇을 움직이지 않고 TCP·툴만 점검
ros2 run box_scan scan_template --ros-args -p robot_setup:=on -p fake_delay_sec:=0.3 &
sleep 3; ros2 run box_control robot_test --ros-args -p reapply_test:=false; pkill -f box_scan

# 실제 동작 포함 (로봇이 움직임 — 작업공간 비우고 펜던트 속도 낮게)
ros2 run box_control test_scan_motion --ros-args -p robot_setup:=on &
sleep 3; ros2 run box_control robot_test --ros-args -p reapply_test:=false; pkill -f test_scan_motion
```

**세 파트 동시 연결 시험** — 탐지·판별·적재가 각자 두산에 연결해 둔 채 관제가 순서대로 움직이게 합니다 (실제 통합과 같은 구조).

```bash
ros2 run box_database database_template &
ros2 run box_control test_scan_motion  --ros-args -p robot_setup:=on -p box_count:=2 &
ros2 run box_control test_judge_motion --ros-args -p robot_setup:=on &
ros2 run box_control test_place_motion --ros-args -p robot_setup:=on &
sleep 5; ros2 run box_control control --ros-args -p start_delay_sec:=5.0
pkill -f database_template; pkill -f _motion
```

`test_scan_motion`은 `box_count`를 꼭 지정합니다 (0이면 계속 “물품 있음”이라 관제가 끝나지 않음). 박스 2개 → `DONE`이면 통과.

검증 결과 (11/14): 가상·실제 로봇 모두 통과. HOME에서 TCP 기준 z=214.9, 플랜지 기준 z=423.3 → 차이 208.4 (TCP 오프셋과 일치)

---

## 8. 팀

| 역할 | 담당 | 패키지 |
| --- | --- | --- |
| 관제·통합 (팀장) | 최은정 | `box_interfaces`, `box_comm`, `box_control` |
| 탐지 | 김창균 | `box_scan` |
| 판별 | 송현석 | `box_judge` |
| 적재 | 정수현 | `box_place` |
| DB·UI | 이승재 | `box_database` |
