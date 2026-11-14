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
```

- 관제 ↔ 탐지/판별/적재: **Action** (로봇이 움직여 오래 걸리고, 진행·결과가 필요)
- 관제 ↔ DB: **Service** (짧은 요청·응답)
- 토픽은 쓰지 않습니다. 파트끼리는 직접 통신하지 않고, 파트는 DB와 UI를 모릅니다.
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
├─ box_interfaces/   메시지 정의 (.msg .srv .action)
├─ box_comm/         공용 통신 라이브러리 (노드 아님)
│  ├─ common.py       공통 값: 에러 코드, stage, TCP·툴, 고정 좌표, PartError
│  ├─ robot.py        기본 TCP·툴 확인·적용 (파트 Goal 시작 전 자동 실행)
│  └─ control_interface.py, scan_interface.py, judge_interface.py,
│     place_interface.py, database_interface.py
├─ box_control/      관제: control.py (순서) + preflight.py (시작 전 점검)
│                    + 테스트용 fake_*.py 4개, test_scan_motion.py, robot_test.py
├─ box_scan/         탐지: scan.py
├─ box_judge/        판별: judge.py
├─ box_place/        적재: place.py
├─ box_database/     DB·UI: database.py
└─ docs/             CONTRIBUTING.md, PROGRESS.md
```

| 패키지 | 역할 | 노드 | 실행 |
| --- | --- | --- | --- |
| `box_interfaces` | 메시지 정의 | - | - |
| `box_comm` | 공용 통신 코드 | - | import 전용 |
| `box_control` | 관제 | `/control` | `ros2 run box_control control` |
| `box_scan` | 탐지 | `/scan` | `ros2 run box_scan scan` |
| `box_judge` | 판별 | `/judge` | `ros2 run box_judge judge` |
| `box_place` | 적재 | `/place` | `ros2 run box_place place` |
| `box_database` | DB·UI | `/database` | `ros2 run box_database database` |

---

## 4. 인터페이스 (`box_interfaces` v0.3)

### Action (관제 → 파트)

| 이름 | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `/scan` | 없음 (시작명령) | `success`, `found`, `error_code` | `stage` |
| `/judge` | `specs: ClassSpec[]` | `success`, `result_class`, `error_code` | `stage` |
| `/place` | `result_class`, `inventory` | `success`, `inventory`(적재 후), `error_code` | `stage` |

`/scan`의 `found=False`는 "더 이상 물품 없음"이며 실패가 아닙니다(`success=True`).

### Feedback stage

각 파트는 단계가 바뀔 때마다 Feedback `stage`를 **반드시** 보냅니다. 관제는 마지막 stage와 받은 시각으로 진행 상황을 추적하고, 중단될 때 어느 단계에서 멈췄는지 기록합니다. 목록은 `box_comm/common.py`의 `STAGES`에 있으며, 목록에 없는 값을 보내면 경고 로그가 남습니다.

| Action | stage 순서 |
| --- | --- |
| `/scan` | `space_scan` → `picking` → `moving` → `placing` |
| `/judge` | `measuring_length1` → `measuring_length2` → `measuring_length3` → `measuring_weight` → (`measuring_materiality`, 옵션) → `judging` |
| `/place` | `picking` → `moving` → `placing` |

모든 Action은 맨 앞에 `robot_setup`(기본 TCP·툴 확인, `box_comm`이 자동으로 보냄)이 먼저 옵니다.
판별의 길이 측정은 물품 방향에 따라 가로·세로·높이가 달라질 수 있어 번호로 구분합니다.

### Service (관제 → DB)

| 이름 | Request | Response | 호출 시점 |
| --- | --- | --- | --- |
| `/get_job_data` | 없음 | `success`, `specs`, `inventory`, `poses`, `message` | 작업 시작 시 |
| `/update_inventory` | `box_id`, `inventory` | `success`, `message` | 적재완료마다 (같은 `box_id`는 한 번만 반영) |
| `/set_job_status` | `status` | `success` | 작업중, 작업완료 시 |

### 메시지

| 메시지 | 필드 |
| --- | --- |
| `Inventory` | `a`, `b`, `c`, `d`: int32 — 분류별 재고 수 |
| `ClassSpec` | `class_id`: string, `width_mm`, `depth_mm`, `height_mm`, `weight_g`: float32 |
| `FixedPoses` | `available`: bool, `home_j`: float64[6] (deg), `measure_zone`: float64[3] (x, y, z mm) — DB에 저장된 고정 좌표 사본 |

### 공통 값

| 항목 | 값 |
| --- | --- |
| `status` | `BEFORE`, `RUNNING`, `DONE` |
| `result_class` | `A`, `B`, `C`, `D` |
| `error_code` | 성공 `""` / 실패 `NOT_FOUND`, `GRASP_FAILED`, `OVER_FORCE`, `MEASURE_FAILED`, `NO_SLOT`, `PLACE_FAILED`, `ROBOT_SETUP_FAILED` |
| 좌표·단위 | 로봇 base 기준, mm |

---

## 5. 관제 흐름

```
BEFORE        GetJobData로 분류 기준(specs), 재고, 고정 좌표 사본(poses) 수신
PREFLIGHT     시작 전 점검 (분류 기준, 고정 좌표 common.py vs DB) → FAIL이면 시작 안 함
STANDBY       시작 대기 카운트다운 (작업공간 안전 확인, q: 취소 / s: 바로 시작)
RUNNING       SetJobStatus(RUNNING)
  ┌─ SCANNING    /scan Goal ──▶ found=False(물품 없음)면 관제가 작업완료로 판단
  │  SCAN_DONE
  │  JUDGING     /judge Goal(specs)
  │  JUDGE_DONE  result_class 수신
  │  PLACING     /place Goal(result_class, inventory)
  │  PLACE_DONE  적재 후 inventory 수신 → UpdateInventory(box_id, inventory)
  └─ 반복
DONE          SetJobStatus(DONE)
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

- **중단은 두 가지로 통일합니다.**
  - **시작 전 중단** (`RUNNING` 전): 작업 데이터 수신 실패, 사전 점검 FAIL, 카운트다운 중 `q` → 모두 같은 메시지 `작업을 시작하지 않습니다 [단계]: 이유`로 끝나고, 로봇은 움직이지 않으며 DB는 `BEFORE` 그대로입니다.
  - **작업 중 중단** (`RUNNING` 이후): 파트 오류 등 → `단계에서 중단: error_code | 마지막 stage` 로그
- `-ING` 상태는 Goal을 보내기 직전, `_DONE` 상태는 Result를 받은 직후에 바뀝니다.
- 단계 상태는 관제 내부에만 있고, DB에는 `BEFORE` / `RUNNING` / `DONE`과 재고만 기록합니다.
- 상태 값은 영어(`State` Enum)이고, 로그에는 `===== SCANNING (스캔중) =====`처럼 한글을 함께 표시합니다.
- 스캔은 물품 유무(`found`)만 알리고, 작업완료 여부는 관제가 판단합니다.
- **재고 검증**: 관제는 적재 Result의 재고를 기대값(적재 전 재고에서 `result_class` 칸만 +1)과 비교합니다. 다르면 기대값으로 DB에 반영하고 `재고 카운팅 오류` 경고 로그를 남긴 뒤 작업을 계속하며, 작업이 끝나면 불일치 건수를 요약해서 출력합니다.
- **중단 기록**: 오류로 중단되면 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 로그로 남깁니다. (예: `SCANNING (스캔중) 단계에서 중단: GRASP_FAILED | /scan 마지막 stage="picking" (4.2초 전)`)
- `box_id`는 관제가 `<작업 시작 시각>_<순번>` 형식으로 붙입니다. (예: `20261114_130512_001`)
  관제를 다시 실행해도 이전 작업과 겹치지 않으므로, DB 노드를 켜 둔 채 관제만 재실행해도 됩니다.

### 사전 점검 (PREFLIGHT) — `box_control/preflight.py`

| 점검 | FAIL (시작 안 함) | WARN (진행) |
| --- | --- | --- |
| 분류 기준 | A~D 중 빠진 것, 0 이하 값 | |
| 고정 좌표 | `common.py` 원본과 DB 사본이 다름 → "측정존 설정값이 DB와 다릅니다. 재설정 후 실행해 주세요." | DB에 사본 없음, `common.py` 값 미설정 |

고정 좌표는 `box_comm/common.py`의 `FIXED_POSES`가 **원본**(로봇은 항상 이 값으로 움직임)이고, DB는 값 보호와 UI 표시를 위한 **사본**입니다. 둘을 비교해서 어느 한쪽이 실수로 바뀌어도 로봇이 움직이기 전에 잡습니다.
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
ros2 run box_scan scan --ros-args -p robot_setup:=on
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
ros2 run box_database database
ros2 run box_scan scan
ros2 run box_judge judge
ros2 run box_place place
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
- 관제가 터미널 입력을 받을 수 없는 환경(launch 등)에서는 취소 기능 없이 대기 후 시작합니다.

### 로봇 없이 실행 (fake)

`box_control`의 fake 노드로 전체 흐름을 로봇 없이 확인할 수 있습니다.

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
| `fake_place` | `fake_delay_sec`, `miscount_at` (N번째 적재에서 일부러 틀린 재고 반환 — 관제 재고 검증 테스트용, 기본 0=끔) |
| `fake_database` | `pose_offset` (고정 좌표에 값을 더해 불일치 테스트), `poses_available` (False면 사본 없음) |

fake와 실제 노드는 같은 Action/Service 이름을 쓰므로 **동시에 띄우지 않습니다.**
파트 하나만 실제로 붙여 보려면 그 파트만 실제 노드로, 나머지는 fake로 띄웁니다.

### 로봇 연동 점검 (관제 테스트 도구)

두산 bringup(가상 또는 실제)을 띄운 상태에서 TCP·툴 자동 확인과 로봇 동작 구조를 점검합니다.

| 도구 | 내용 |
| --- | --- |
| `test_scan_motion` | `scan_interface` + 실제 두산 동작(`connect_dsr`, `movej`/`movel`)이 들어간 `/scan` 노드. Goal마다 HOME → 위로 50mm → 아래로 → HOME. **팀원이 실제 동작을 넣을 때 참고할 예시 코드** |
| `robot_test` | `/scan` Goal을 반복해서 보내며 TCP·툴 적용을 점검하고 PASS/FAIL 표 출력. `-p reapply_test:=false`면 컨트롤러 설정을 바꾸는 3번 항목을 건너뜀 |

```bash
# 로봇을 움직이지 않고 TCP·툴만 점검
ros2 run box_scan scan --ros-args -p robot_setup:=on -p fake_delay_sec:=0.3 &
sleep 3; ros2 run box_control robot_test --ros-args -p reapply_test:=false; pkill -f box_scan

# 실제 동작 포함 (로봇이 움직임 — 작업공간 비우고 펜던트 속도 낮게)
ros2 run box_control test_scan_motion --ros-args -p robot_setup:=on &
sleep 3; ros2 run box_control robot_test --ros-args -p reapply_test:=false; pkill -f test_scan_motion
```

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
