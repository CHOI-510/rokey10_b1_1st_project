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
- 분류 기준값(물품 A~D의 크기·무게)은 DB에서 읽습니다. 재고는 시작 시 로봇이 실측하고, 작업 중 관제 메모리에서 갱신한 뒤 작업 종료·중단 때 최종값을 DB에 저장합니다. 웹 UI 연동은 아직 진행 전입니다.

<!-- TODO: 시연 영상 링크 -->

---

## 2. 시스템 구조

```
                 Service                         Action
  database  ◀──────────────▶  control  ◀──────────────▶  scan   (탐지)
  (DB)       GetJobData        (관제)    /inventory_check (실측)
             UpdateInventory             /scan            judge  (판별)
             SetJobStatus                /judge           place  (적재)
                                          /place
                 Topic
            ◀───────────────
             /event_log  (로그, 관제 → DB 보내기만)
                 Service
            ───────────────▶
             /emergency_stop (비상정지 버튼, DB → 관제)
             /control_status (관제 → 관리자 GUI, 재고·단계·남은 시간)
             /job_command    (관리자 GUI → 관제, 시작·취소·바로 시작)

  control ──▶ 두산 dsr_controller2: motion/move_stop (정지), system/get_robot_state (상태 감시)
```

- 관제 ↔ 탐지/판별/적재: **Action** (로봇이 움직여 오래 걸리고, 진행·결과가 필요)
- 관제 ↔ DB: 응답이 필요하면 **Service** (작업 데이터, 재고 반영, 상태 기록), 알리기만 하면 **Topic** (이벤트 로그 `/event_log`)
- 파트끼리는 직접 통신하지 않고, 파트는 DB와 UI를 모릅니다. 파트 로그도 Feedback으로 관제에 보내면 관제가 `/event_log`로 넘깁니다.
- 비상정지만 방향이 반대입니다(DB → 관제 `/emergency_stop`). 로봇 정지와 상태 감시는 관제가 두산 서비스를 직접 씁니다.
- 실물 구성 노드는 `/control`, `/inventory_check`, `/scan`, `/judge`, `/place`, `/database` (+ 관리자 화면 `/control_monitor`). 가상·fake 모드에서는 일부 파트가 fake Action 서버로 대체됩니다.

<!-- TODO: 노드 구조도, 통신 시퀀스 그림 -->

### 코드 구조: 통신과 동작의 분리

각 노드는 **통신 계층**(`box_comm`)과 **작업 코드**(각 파트 패키지)로 나뉩니다.

- `box_comm`: Action/Service 서버·클라이언트, ROS 메시지 ↔ 파이썬 값 변환, 계약 규칙(Result 보장, 등록된 `error_code`만 허용, `box_id` 중복 무시 등)을 코드로 강제
- 각 파트: 약속된 알고리즘 메서드 하나만 구현 (`scan(poses, report)` / `judge(specs, poses, report)` / `place(result_class, poses, report)`). 로봇은 관제가 넣어 주는 `robot`(`box_comm/dsr_io.py`)으로만 움직이고, ROS 메시지 타입을 몰라도 됨
- 세 파트 Action 서버 흐름(Goal 하나만, 좌표 넣기, TCP·툴 확인, 실패 마무리)은 `box_comm/part_server.py` 한 곳에 있음 (11/18 통합)

인터페이스가 바뀌어도 `box_comm`만 수정하면 되고, 파트 코드는 로봇 동작에만 집중합니다.

---

## 3. 패키지 구성

```
box_project/
├─ .githooks/        관제 전용 파일 commit 방지 훅 (팀원은 git config core.hooksPath .githooks)
├─ .github/          GitHub Actions: 관제 전용 파일 변경 감시
├─ box_interfaces/   메시지 정의 (.msg .srv .action)
├─ box_comm/         공용 통신 라이브러리 (노드 아님)
│  ├─ common.py       공통 값: 에러 코드, stage, TCP·툴, 고정 좌표, 그리퍼·sync 집기 설정, PartError
│  ├─ convert.py      메시지 ↔ 파이썬 값 변환 (관제·판별·DB 공용, 11/18)
│  ├─ robot.py        기본 TCP·툴 확인·적용 (파트 Goal 시작 전 자동 실행)
│  ├─ dsr_io.py       파트·관제가 쓰는 로봇 명령 (명령 전 정지 확인, 진행 알림)
│  ├─ gripper.py      RG2 그리퍼 Modbus (세 파트 공용, 11/17) + 잡힘 판정
│  ├─ sync_grip.py    닫으면서 핑거 끝 높이 유지 집기 (판별·적재·탐지 공용, 11/18)
│  ├─ algorithm.py    파트 알고리즘 실행 어댑터 (로봇 주입, 실패 시 정지)
│  ├─ part_server.py  파트 Action 서버 공통 틀 (11/18)
│  └─ control_interface.py, database_interface.py,
│     scan_interface.py, judge_interface.py, place_interface.py (파트별 입력·결과 검사만)
├─ box_control/      관제: control.py (순서) + preflight.py (시작 전 점검)
│                    + recovery.py (관제 복구 동작: 측정존 비우기 등, 관제가 직접 로봇을 움직임)
│                    + monitor_gui.py (관리자 모니터 화면, 11/18 — 관제 실행 시 같이 뜸)
│                    + 테스트용 fake_scan/judge/place.py, test_scan/judge/place_motion.py, robot_test.py
├─ box_scan/         탐지: scan_template.py + scan_algorithm.py·scan_config.py·scan_main.py (담당)
│                    + scan_original.py (담당 원본, 보관만 — 실행 노드 아님) + test/test_scan_flow.py
├─ box_judge/        판별: judge_template.py + judge_algorithm.py·judge_main.py·rg2_sync_grip.py (담당) + test/test_judge_grip.py
├─ box_place/        적재·시작 재고 점검: place_template.py + place_algorithm.py·place_main.py·inventory_check_algorithm.py (담당) + test/test_place_pick.py
├─ box_database/     DB 노드: database_main.py (+ database_work/store/config), 약속 예시 database_template.py
├─ box_bringup/      전체 실행 launch: system.launch.py (전체), robot.launch.py (두산 bringup, 그리퍼 드라이버 끔), config/params.yaml
└─ docs/             CONTRIBUTING.md, PROGRESS.md
```

| 패키지 | 역할 | 노드 | 실행 |
| --- | --- | --- | --- |
| `box_interfaces` | 메시지 정의 | - | - |
| `box_comm` | 공용 통신 코드 | - | import 전용 |
| `box_control` | 관제 | `/control` (+ `/monitor_gui`) | `ros2 run box_control control` (모니터 GUI 창이 같이 뜸, 끄기 `-p gui:=false`) |
| `box_scan` | 탐지 | `/scan` | `ros2 run box_scan scan_main` (참고용 `scan_template`) |
| `box_judge` | 판별 | `/judge` | `ros2 run box_judge judge_main` (참고용 `judge_template`) |
| `box_place` | 적재·실물 재고 점검 | `/place`, `/inventory_check` | `ros2 run box_place place_main`, `ros2 run box_place inventory_check` |
| `box_database` | ROS DB 노드 (웹 UI 미연동) | `/database` | `ros2 run box_database database_main` (Supabase 키 없으면 파일 모드) |
| `box_bringup` | 전체 실행 | (위 전부 + 두산 bringup) | `ros2 launch box_bringup system.launch.py mode:=real` |

---

## 4. 인터페이스 (`box_interfaces` v0.7)

### Action (관제 → 파트)

| 이름 | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `/scan` | `poses` (시작명령) | `success`, `found`, `error_code` | `stage`, `log_level`, `log_msg` |
| `/judge` | `specs: ClassSpec[]`, `poses` | `success`, `result_class`, `error_code` | `stage`, `log_level`, `log_msg` |
| `/place` | `result_class`, `poses`, `inventory`(현재 관제 스냅샷, v0.7) | `success`, `count`(측정 수량), `placed_class`(실제로 놓은 곳, 구역 만재면 `U` — v0.6), `error_code` | `stage`, `log_level`, `log_msg` |
| `/inventory_check` | `result_class`, `poses`, `inventory`(현재 관제 스냅샷, v0.7) | `success`, `count`(구역 실측 수량), `error_code` | `stage`, `log_level`, `log_msg` |

모든 Goal의 `poses`(`FixedPoses`)는 관제가 DB에서 받은 고정 좌표(HOME, 측정존, 적재 구역 1단 x·y)입니다. 파트는 이 값으로 움직입니다.

`/place`와 `/inventory_check` Goal은 호출 시점의 관제 재고 스냅샷을 받습니다. 파트 알고리즘에서는 `self.inventory`로 읽을 수 있습니다. 결과 수량은 관제가 메모리 재고에 반영하고, 초기 실측 완료와 작업 종료·중단 시점에만 DB에 저장합니다. `/place`의 `count`는 적재가 **실제로 놓은 위치(단·층)로 잰** `result_class` 수량입니다(v0.4). 단·층·z는 적재 내부 정보라 Result에 넣지 않고 적재 로그에만 남깁니다. 아래 "적재 규칙" 참고.

`/scan`의 `found=False`는 "더 이상 물품 없음"이며 실패가 아닙니다(`success=True`).

**미분류 `U` (v0.5)**: 매 작업의 U 재고는 0부터 시작합니다. 판별 결과가 `U`이거나 클래스 구역이 만재라 U 위치로 보낸 경우 관제가 U 수량을 집계합니다. 적재는 미분류 투하 위치(`place_config.json`의 `unclassified`)에 놓고 `count = 0`을 돌려줍니다. 재고는 메모리와 `/control_status`에서 즉시 갱신하고, DB에는 초기 실측 완료 및 작업 종료·중단 시에만 저장합니다. 중단 시 저장된 값은 마지막 관제 확인 스냅샷이며, 물리 상태가 확정되지 않은 중단은 `reason`과 `/control_status.inventory_needs_recheck`로 재측정 필요를 표시합니다. 미분류 위치는 DB 고정 좌표가 아니라 적재 설정에 있습니다.

### Feedback stage

각 파트는 단계가 바뀔 때마다 Feedback `stage`를 **반드시** 보냅니다. 관제는 마지막 stage와 받은 시각으로 진행 상황을 추적하고, 중단될 때 어느 단계에서 멈췄는지 기록합니다. 목록은 `box_comm/common.py`의 `STAGES`에 있으며, 목록에 없는 값을 보내면 경고 로그가 남습니다.

| Action | stage 순서 |
| --- | --- |
| `/inventory_check` | `moving` → `placing` |
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
| `/update_inventory` | `box_id`, `inventory` | `success`, `message` | 시작 실측 후 1회, 작업 종료·중단 시 최종 스냅샷 1회 (박스별 저장 없음) |
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
| `Inventory` | `a`, `b`, `c`, `d`: int32 — 분류별 재고 수 (적재 측정값), `u`: int32 — 미분류 수 (관제 집계, v0.5) |
| `ClassSpec` | `class_id`: string, `width_mm`, `depth_mm`, `height_mm`, `weight_g`: float32 |
| `FixedPoses` | `available`: bool, `home_j`: float64[6] (deg), `measure_zone`: float64[3] (x, y, z mm), `place_zones`: `PlaceZone[]` — 고정 좌표. **기준은 DB**. `GetJobData`로 받아 모든 Action Goal에 실어 보냄 |
| `LogEvent` | `stamp`, `job_id`, `source`(control/scan/judge/place), `stage`, `level`(INFO/WARN/ERROR), `code`, `box_id`, `class_id`: string, `expected`, `measured`: int32 (-1 = 해당 없음), `message`: string |
| `PlaceZone` | `class_id`: string, `x`, `y`: float64 (mm) — 클래스별 적재 구역의 **1단** 위치 (v0.4) |

### 공통 값

| 항목 | 값 |
| --- | --- |
| `status` | `BEFORE`, `RUNNING`, `DONE`, `ERROR`, `STOPPED` (`common.JOB_STATUS`) |
| `reason` | `ERROR`·`STOPPED`일 때 UI에 보일 이유 (`common.ERROR_TEXT`, 예: `Fail to pick`, `Zone full (B)`, `EMERGENCY STOP`). UI 표시: `{status} : {reason}` |
| `result_class` | `A`, `B`, `C`, `D`, `U`(미분류, v0.5) |
| `error_code` | 성공 `""` / 실패 `NOT_FOUND`, `GRASP_FAILED`, `OVER_FORCE`, `MEASURE_FAILED`, `NO_SLOT`, `PLACE_FAILED`, `ROBOT_SETUP_FAILED` |
| 좌표·단위 | 로봇 base 기준, mm |

### 적재 규칙 (v0.4, `box_comm/common.py` `STACK`)

- 구역 하나에 같은 클래스만 쌓습니다(A 물품 → A 구역).
- **1단(첫 자리)** 에 1층부터 **3층**까지 쌓고, 다 차면 **다음 자리(2단)** 에 다시 1층부터 쌓습니다. 구역당 **2자리** (`STACK` = 3층 × 2자리, 최대 6개). 꽉 차면 그 박스는 **미분류 투하 위치로** 보내고 작업은 계속합니다 (11/18: 재고 그 구역 = 6, U +1, 로그 `ZONE_FULL`. 미분류 투하가 꺼져 있을 때만 예전처럼 쥔 채 `NO_SLOT`).
- 다음 자리 위치는 적재 설정 `box_place/config/place_config.json`의 `slot_dx_mm`(현재 **−x 방향 80mm**, y는 같음)로 정합니다.
- 적재 규칙(층·자리 수)은 `STACK` 하나가 기준입니다. 적재 설정(`place_config.json`)에는 적지 않고 `place_algorithm`이 `STACK`을 그대로 씁니다. 바꿀 때는 관제와 합의하고, 자리별 첫 접촉 Z를 다시 재야 합니다.
- 적재는 **매번 1단부터** 박스를 쥔 채 위에서 내려가, 더 내려가지 않는 높이(z)로 층을 판단합니다. 4층 높이(3층 위)에서 닿으면 그 단은 꽉 찬 것이므로 놓지 않고 다음 단으로 갑니다.
- 놓은 위치로 수량을 계산합니다: `수량 = (단-1) × 3 + 층` (예: 2단 2층 → 5개). 앞 단은 모두 찼다고 봅니다.
- 관제·DB는 **1단 x·y와 수량만** 알고, 단 간격·z·박스 높이는 적재 파트만 가집니다.
- UI 적재 이미지는 수량에서 바로 그립니다: `stack_layout(5)` → `[3, 2]` (1자리 3층, 2자리 2층).

---

## 5. 관제 흐름

```
READY         (상주 모드, 기본) 시작 명령 대기 — UI 시작 버튼(DB 노드 → /job_command) / 관리자 GUI / 터미널 s
              대기 중에도 로봇 상태를 0.5초마다 읽어 /control_status로 발행. 작업이 끝나면 여기로 돌아옴
BEFORE        GetJobData로 분류 기준(specs), 고정 좌표(poses, DB 기준) 수신
PREFLIGHT     시작 전 점검 (분류 기준, 좌표, 적재 구역, 로봇 상태, 그리퍼, 측정존 비우기) → FAIL이면 시작 안 함
STANDBY       시작 대기 카운트다운 (기본 10분. UI 취소·q: 취소 / 관리자 GUI·s: 바로 시작)
RUNNING       SetJobStatus(RUNNING)
START_SETTING 대기 종료/바로 시작 후 그리퍼 열기 → HOME → 닫기
INVENTORY_CHECK (옵션) 시작 재고 실측 (`inventory_check:=false`면 생략)
  ┌─ SCANNING    /scan Goal ──▶ found=False(물품 없음)면 관제가 작업완료로 판단
  │  SCAN_DONE
  │  JUDGING     /judge Goal(specs)
  │  JUDGE_DONE  result_class 수신
  │  PLACING     /place Goal(result_class, poses)
  │  PLACE_DONE  측정 수량(count) 수신 → 기대값(+1)과 비교·기록 → 메모리 재고 및 `/control_status` 갱신
  └─ 반복
DONE          최종 재고 스냅샷 저장 → SetJobStatus(DONE) → READY

집기 실패     → GRASP_FAILED면 (판별·적재는 CLEARING: 관제가 측정존 비움) 공간 스캔부터 다시, 연속 3번 넘으면 ERROR
작업 중 오류  → ERROR   SetJobStatus(ERROR, "Fail to pick")       공정 오류 — 로그 확인 후 재시작
로봇 정지     → STOPPED SetJobStatus(STOPPED, "EMERGENCY STOP")  로봇이 멈춤 — 현장 확인 후 복구
관제 코드 오류 → ERROR  "Control error — …" (로봇 정지 + 파트 cancel, 관제는 죽지 않고 READY로, 11/18)
작업 중단     → ERROR/STOPPED일 때 실측 시작값과 작업 중 메모리 재고를 최종 스냅샷으로 저장
```

재고 DB 기록은 시작 실측 결과 1회와 작업 종료·중단 시점 1회입니다. 적재마다 DB를 쓰지 않습니다. 작업 중 관리자 GUI는 관제가 발행하는 현재 재고를 보여 줍니다.

| 상태 | 한글 | DB 기록 |
| --- | --- | :-: |
| `READY` | 대기 (상주 모드, 시작 명령 기다림) | |
| `BEFORE` | 작업전 | ✓ |
| `PREFLIGHT` | 사전점검 | |
| `STANDBY` | 시작대기 | |
| `RUNNING` | 작업중 | ✓ |
| `INVENTORY_CHECK` | 시작 재고 실측 | |
| `SCANNING` / `SCAN_DONE` | 스캔중 / 스캔완료 | |
| `JUDGING` / `JUDGE_DONE` | 판별중 / 판별완료 | |
| `PLACING` / `PLACE_DONE` | 적재중 / 적재완료 | |
| `CLEARING` | 측정존비우기 (집기 실패 뒤 관제가 직접 로봇을 움직임) | |
| `DONE` | 작업완료 | ✓ |
| `ERROR` | 오류중단 (공정 오류, 관제가 판단해서 멈춤) | ✓ |
| `STOPPED` | 정지 (로봇이 물리적으로 멈춤 — 비상정지, 안전 정지) | ✓ |

- **중단은 두 가지로 통일합니다.**
  - **시작 전 중단** (`RUNNING` 전, `cancel_start()`): 작업 데이터 수신 실패, 사전 점검 FAIL, 카운트다운 중 UI 취소·`q` → `작업을 시작하지 않습니다 [단계]: 이유`. 로봇은 움직이지 않고 DB는 `status = BEFORE`, `reason = '시작 안 함: …'`(시작 전 비상정지면 `STOPPED … (시작 전)`). 상주 모드면 READY로 돌아갑니다.
  - **작업 중 중단** (`RUNNING` 이후, `abort_job()`): 에러 코드로 `ERROR`/`STOPPED`를 정하고 DB에 `status`와 `reason`을 기록합니다(예: `ERROR : Fail to pick`). 로그에는 `단계에서 중단: error_code | 마지막 stage`가 남습니다. `abort_job()` 자체는 로봇을 멈추지 않고, 로봇 정지(`move_stop`)·파트 cancel은 아래 '정지 기능'의 `request_stop()`이 합니다.
  - `STOPPED`가 되는 코드는 `common.STOP_CODES`(`EMERGENCY_STOP`, `ROBOT_SAFETY_STOP`, `ROBOT_EMERGENCY_STOP`, `ROBOT_DISCONNECTED`, `ROBOT_NOT_READY`), 나머지는 모두 `ERROR`.
- `-ING` 상태는 Goal을 보내기 직전, `_DONE` 상태는 Result를 받은 직후에 바뀝니다.
- 단계 상태는 관제 내부에만 있고, DB에는 `BEFORE` / `RUNNING` / `DONE` / `ERROR` / `STOPPED`와 재고만 기록합니다.
- 상태 값은 영어(`State` Enum)이고, 로그에는 `===== SCANNING (스캔중) =====`처럼 한글을 함께 표시합니다.
- 스캔은 물품 유무(`found`)만 알리고, 작업완료 여부는 관제가 판단합니다.
- **재고 검증 (v0.4)**: 관제는 적재가 잰 수량(`count`)을 기대값(적재 전 재고의 `result_class` 수량 + 1)과 비교합니다. 차이가 나면 `재고 불일치` 경고를 남기고 실측값으로 관제 메모리를 갱신합니다. DB는 시작 실측 완료 후 초기 스냅샷과 작업 종료·중단 시 최종 스냅샷만 받으며, 박스별 중간 재고 쓰기는 하지 않습니다.
  - 예외 (11/18): 적재가 잰 수량이 1 ~ 구역 정원(6) 밖이면 `place_interface`가 **작업을 실패시키지 않고** `count = 0`(수량 확인 불가)으로 보냅니다(박스는 이미 놓였으므로). 관제는 0을 받으면 **반영하지 않고** 이전 값을 유지하며 `COUNT_MISMATCH`에 '적재 현황 확인 불가'를 남깁니다. 같은 클래스를 다음에 적재할 때 측정값으로 바로잡힙니다.
- `DONE` 상태 기록이 실패하면 에러 로그를 남깁니다 (UI에는 `RUNNING`으로 남아 있을 수 있음).
- **미분류 (v0.5)**: 새 작업의 `U`는 0부터 시작합니다. `result_class`가 `U`이거나 적재 구역 만재로 U 위치에 보냈으면 비교 없이 관제 메모리의 U를 +1 합니다 (적재는 `count = 0`). 최종 DB 스냅샷은 종료·중단 때 저장합니다.
- **집기 실패 (11/17)**: 파트가 `GRIP_RETRY`(3)번 다시 집어 보고 포기하면 `GRASP_FAILED`로 Goal을 끝냅니다(그리퍼 열고 안전 높이). 관제는 작업을 끝내지 않고, 판별·적재였으면 **`CLEARING`(측정존비우기)** 상태에서 관제가 직접 로봇을 움직여 측정존 박스를 스캔 공간으로 밀어내고 비었는지 확인합니다(`box_control/recovery.py`, 관제 복구 동작 모음 — 나중에 예외 처리 동작도 여기에). 그 뒤 `BOX_RETRY` 경고를 남기고 **공간 스캔부터 다시** 합니다(재고·박스 번호 그대로). 연속 `RECOVER_LIMIT`(3)번을 넘으면 `ERROR : Too many pick failures`, 측정존을 못 비우면 `ERROR : Measure zone not cleared`. 박스 하나를 적재까지 끝내면 연속 횟수는 0. 밀어내기 경로(`common.RECOVERY push_path`)는 아직 미정 → **11/18부터 비우기를 fake로 넘기고(WARN 로그 `ZONE_CLEAR`, 대기 없음) 바로 재스캔**합니다. 스캔하는 동안 사람이 측정존 박스를 치웁니다. 관제 파라미터 `clear_zone`: `auto`(기본: `robot_monitor=dsr`면 실제 로봇, 아니면 `fake_ok`) / `robot` / `fake_ok` / `fake_fail`.
- **중단 기록**: 오류로 중단되면 해당 파트의 마지막 Feedback stage와 경과 시간을 함께 로그로 남깁니다. (예: `SCANNING (스캔중) 단계에서 중단: GRASP_FAILED | /scan 마지막 stage="picking" (4.2초 전)`)
- `box_id`는 관제가 `<작업 시작 시각>_<순번>` 형식으로 붙입니다. (예: `20261114_130512_001`)
  관제를 다시 실행해도 이전 작업과 겹치지 않으므로, DB 노드를 켜 둔 채 관제만 재실행해도 됩니다.

### 사전 점검 (PREFLIGHT) 및 시작 설정 — `box_control/preflight.py`, `control.py`

| 점검 | FAIL (시작 안 함) | WARN (진행) |
| --- | --- | --- |
| 분류 기준 | A~D 중 빠진 것, 0 이하 값 | |
| 고정 좌표 | DB에 좌표 없음 → "DB에 고정 좌표가 없습니다. DB에 HOME·측정존 좌표를 설정한 뒤 실행해 주세요.", 형식 오류 | 측정존이 (0, 0, 0) |
| 적재 구역 | A~D 중 1단 x·y가 없는 구역 | 구역 좌표가 (0, 0) |
| 로봇 상태 | bringup 없음, STANDBY·MOVING이 아님 | `robot_monitor=off` |
| 그리퍼 | RG2 Modbus 접속 실패(OnRobot 드라이버가 잡고 있음 등), 안전 비트 켜짐 | `robot_monitor`≠dsr (확인 안 함). `BOX_GRIPPER=fake`면 OK. 그리퍼가 물체를 쥐고 있는지는 시작 거부 사유가 아닙니다. |
| 측정존 비우기 (11/17) | | 밀어내기 경로 미정 |

고정 좌표(HOME, 측정존, 적재 구역 1단 x·y)는 **DB가 기준**입니다(v0.3, 적재 구역은 v0.4). 관제는 `GetJobData`로 받은 DB 값으로 자기 좌표를 **무조건 덮어쓰고**(비교·중단 없음, 로그만 남김), 그 값을 `/scan`, `/judge`, `/place` Goal에 매번 실어 보내고, 각 파트는 그 값(작업 함수의 `poses` 인자, 예전 방식은 `self.poses`)으로 움직입니다. `box_comm/common.py`의 `FIXED_POSES`는 **기본값**으로, Goal에 좌표가 없을 때(팀원이 혼자 `send_goal`로 테스트할 때)만 쓰입니다.
WARN·FAIL 항목과 통과/실패 요약은 이벤트 로그(`code = PREFLIGHT`)로도 보내서 웹·관리자 GUI 로그 창에 보입니다 (11/18).
점검을 추가할 때는 `preflight.py`에 함수를 만들고 `CHECKS`에 넣습니다.

사전 점검은 로봇을 움직이지 않습니다. 시작 확인 후 10분 카운트다운이 끝나거나 관리자가 바로 시작을 누르면 `START_SETTING`에서 그리퍼를 최대로 열고 HOME으로 이동한 뒤 닫습니다. 그 다음 `inventory_check:=true`(기본값)이면 재고 실측을, `false`이면 재고 실측을 건너뛰고 스캔을 실행합니다. 즉 `inventory_check:=false`는 시작 시 그리퍼 설정을 건너뛰는 옵션이 아닙니다.

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
   - 모드 전환·수동 모드 적용은 응답의 `success`까지 확인해서, 실패하면 어느 단계인지 메시지에 남깁니다 (등록 실패는 이미 등록된 경우라 무시)
   - 중간에 실패해도 자동 모드 복귀는 항상 시도합니다. 복귀까지 실패하면 두 원인을 함께 남깁니다 (`… / 자동 모드 복귀 실패 — 펜던트에서 자동 모드로 바꿔 주세요`)
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

### 실행 — 한 번에 (launch, 11/19)

```bash
ros2 launch box_bringup system.launch.py mode:=real        # 실제 로봇 (예전 sodreal + 노드 5개 + GUI)
ros2 launch box_bringup system.launch.py mode:=virtual     # 에뮬레이터 (예전 sodvir). 판별은 fake, 탐지·적재는 가상 박스·가상 접촉
ros2 launch box_bringup system.launch.py fake:=true db_store:=file start_delay:=10.0   # 로봇 없이 전부 fake
ros2 launch box_bringup system.launch.py mode:=real bringup:=false   # bringup은 따로 띄운 경우 (sodreal 터미널)
ros2 launch box_bringup system.launch.py mode:=real inventory_check:=false  # 시작 설정 후 초기 재고 실측을 건너뛰고 스캔 (DB 재고 유지)
```

| mode | 로봇 bringup | 탐지 | 판별 | 적재 | 그리퍼 |
| --- | --- | --- | --- | --- | --- |
| `real` | 실제 (192.168.1.100) | `scan_main` | `judge_main` | `place_main` | Modbus |
| `virtual` | 에뮬레이터 (127.0.0.1) | `scan_main` 가상 박스 | `fake_judge` | `place_main` 가상 접촉, 재고 점검도 가상 경로 이동 | 가짜 |
| `fake:=true` | 없음 | `fake_scan` | `fake_judge` | `fake_place`, 재고 점검도 이동 없음 | 가짜 |

- 로봇 bringup은 `box_bringup/launch/robot.launch.py`가 띄웁니다. 수업 패키지 `m0609_rg2_bringup`과 같은 노드인데 **OnRobot 그리퍼 드라이버를 끄고**(`gripper_driver:=false` 기본 — 파트가 Modbus로 직접 쓰므로, 이제 `pkill` 필요 없음) **RViz를 끔**(`rviz:=true`로 켬). 원본 패키지는 고치지 않았습니다.
- 그 밖의 인자: `start_delay`(기본 600), `db_store`(auto/file/supabase), `gui`, `sim_boxes`, `box_count`·`classes`(fake). 고정 파라미터는 `box_bringup/config/params.yaml`.
- `scan_mode` launch 인자는 없습니다. `mode:=virtual`은 가상 박스 힘/높이 모델을 쓰는 `scan_main`을 실행하고, `fake:=true`는 이동 없는 `fake_scan`을 실행합니다.
- launch로 띄우면 **터미널 키(s/q)는 안 들어갑니다.** 관제는 READY에서 기다리고, 시작·취소·바로 시작은 관리자 GUI(또는 웹 UI) 버튼으로. 전체 종료는 launch 터미널에서 Ctrl+C 한 번.
- launch의 순서는 '켜는 순서'일 뿐 준비 완료를 기다리지 않습니다. 파트는 첫 로봇 명령 때 두산 서비스를 기다리고, 관제는 시작 명령을 받으면 사전 점검에서 로봇·그리퍼를 확인하므로 따로 기다릴 필요가 없습니다.
- 로봇 없는 흐름 시험은 지금처럼 `./scripts/fake_run.sh`도 그대로 씁니다 (재고 0으로 시작, 시험 옵션이 더 많음).

### 실행 — 노드를 하나씩 (개발·단독 시험)

서버 4개를 먼저 띄우고 관제를 마지막에 실행합니다.

```bash
ros2 run box_database database_main --ros-args -p store:=file   # Supabase 대신 ~/box_logs/fake_db/*.json
ros2 run box_scan scan_main
ros2 run box_judge judge_main
ros2 run box_place place_main
ros2 run box_control control                                     # 상주 모드: READY에서 시작 명령 대기
```

관제는 **상주 모드**(기본)로 READY에서 시작 명령(UI 시작 버튼 / 관리자 GUI / 터미널 `s`)을 기다리고, 작업이 끝나면 다시 READY로 돌아옵니다. READY에서 `q`는 관제 종료. 예전처럼 바로 한 번 실행하고 끝내려면 `-p auto_start:=true`.

시작 명령을 받으면 사전 점검 뒤 **시작 대기(STANDBY)** 카운트다운을 띄웁니다. GUI 시작 버튼은 “10분 후 작업이 시작됩니다. 작업공간 내 사람 또는 파손 위험이 있는 물품이 없는지 확인해주세요.” 확인창을 띄우며, 확인 시점부터 이 대기가 시작됩니다. 이 시간 동안 로봇은 움직이지 않습니다. 카운트다운 종료 또는 바로 시작 뒤 `START_SETTING`에서 그리퍼를 열고 HOME으로 이동해 닫은 다음, 재고 실측(활성화된 경우)과 스캔을 시작합니다.

```
  ⚠ 10초 후에 작업을 시작합니다.
  ⚠ 작업공간 안에 사람이나 파손 위험이 있는 물품이 없는지 확인하세요.
  중단하려면 q, 바로 시작하려면 s 를 누르세요. 10초 동안 입력이 없으면 자동으로 시작합니다.
  시작까지 7초 ...  (q: 중단 / s: 바로 시작)
```

| 키 | 동작 |
| --- | --- |
| `q` (또는 UI·GUI 시작 취소) | 작업 시작 취소 (DB `status = BEFORE`, `reason = '시작 안 함: …'`, 상주 모드면 READY로) |
| `s` (또는 관리자 GUI 바로 시작) | 바로 시작 (현장 전용, 웹 UI에는 없음. 로그 `START_NOW`) |
| 입력 없음 | 시간이 지나면 자동 시작 |

- 대기 시간은 `start_delay_sec` 파라미터로 바꿉니다. 기본: 상주 모드 600초(10분), `auto_start` 10초.
  `ros2 run box_control control --ros-args -p start_delay_sec:=10.0`   # 시험할 때 짧게
- `0`으로 주면 대기 없이 바로 시작합니다.
- 관제가 터미널 입력을 받을 수 없는 환경(launch 등)에서는 `q` 없이 대기 후 시작합니다. 비상정지 서비스는 대기 중에도 동작합니다(시작 안 함).

### 모니터 GUI (관리자 화면, 11/18) — `box_control/monitor_gui.py`

관제를 실행하면 **모니터 GUI 창이 별도 프로세스로 같이 뜹니다**(GUI가 죽어도 관제는 계속). 관제가 끝나도 창은 결과 확인용으로 남고, 직접 닫습니다. DB·웹 UI 연동 전까지 관제 PC에서 상황을 한눈에 보는 관리자·개발 확인용이며, 작업 흐름에는 끼어들지 않습니다(보기 + 비상정지 버튼).

| 보는 것 | 출처 |
| --- | --- |
| 관제 단계, job_id, 처리 박스 수, 재고 A~D·U, 불일치·재스캔 횟수, 중단 이유, 시작 대기 남은 시간 | `/control_status` (관제가 발행, `std_msgs/String` JSON, 마지막 1개 유지 → GUI를 늦게 켜도 받음). 웹 UI 연동 전까지 관리자 GUI에서 확인 |
| 이벤트 로그 (관제 이벤트 + 파트 `report.log`) | `/event_log` |
| 파트별 Goal 상태(실행중·성공·실패·취소), 진행 stage, 마지막 소식 후 경과(30초 넘으면 주황) | `/scan`·`/judge`·`/place` Action status·feedback |
| 로봇 상태 + 알람 | 관제가 살아 있으면 `/control_status`의 로봇 상태(관제가 0.5초마다 읽은 값), 관제가 없을 때만 직접 `get_robot_state` 1초 주기 (11/18). 알람은 `/dsr01/error` |
| 연결 상태 | `/emergency_stop`(관제), `/get_job_data`(DB), 파트 Action 서버, 로봇 서비스 |

- **비상정지 버튼**: 확인창 [정지]/[취소] → `/emergency_stop`(source=`monitor_gui`).
- **시작 / 시작 취소 / 바로 시작**: `/job_command`(source=`monitor_gui`). 시작 확인창에서 안전 문구를 확인하고 확인을 누르면 10분 대기가 시작되며 GUI에 남은 시간이 표시됩니다. 바로 시작(카운트다운 생략)은 현장 전용이라 관리자 GUI·터미널에만 있습니다. 대기 중에는 로봇이 움직이지 않으며 실제 시작 직전에 그리퍼 열기→HOME→닫기를 수행합니다. 수동 재고 설정은 제거했습니다.
- 끄기: `ros2 run box_control control --ros-args -p gui:=false`. 화면(`DISPLAY`)이 없으면 자동으로 안 띄웁니다. 관제를 `robot_monitor:=off`/`fake`로 실행하면 GUI도 로봇 상태 조회를 끕니다.
- GUI만 따로: `ros2 run box_control monitor_gui` (`-p robot_poll:=false`로 로봇 조회 끔).
- 필요: `sudo apt install python3-tk`
- `/control_status`는 모니터 GUI와 DB 노드가 같이 받습니다. DB 노드의 Supabase 저장 경로는 준비돼 있지만, `my-next-app/`과의 실제 웹 UI 연결은 아직 진행 전입니다 (`docs/DB_UI_CONTRACT.md`).

### 탐지 실제 코드 (`scan_main`, 11/17)

탐지 담당 원본(`m0609_scan_final.py`, 실로봇에서 스캔·인식·픽업·이동 확인)을 팀 구조로 옮긴 것입니다. 알고리즘 `box_scan/box_scan/scan_algorithm.py`(`ScanAlgorithm`), 설정·보정표 `scan_config.py`, 진입점 `scan_main.py`. 원본은 `scan_original.py`로 보관만 합니다 (11/18 실행 노드에서 뺌. 그리퍼 DO 방식이라 비교용, 필요하면 `python3 box_scan/box_scan/scan_original.py`).

```bash
ros2 run box_scan scan_main
ros2 run box_scan scan_main --ros-args -p csv_dir:=~/ROkEY_협동1/04_실험데이터     # 탐색 Fy CSV 기록 (보정표 재측정용)
ros2 run box_scan scan_main --ros-args -p sim_boxes:="420,100,50;510,-20,60"     # 에뮬레이터 전용 (아래)
python3 box_scan/test/test_scan_flow.py                                          # ROS 없이 흐름 시험 (저장소 루트에서)
```

- **Goal 한 번 = 물품 1개.** HOME → 그리퍼 닫기(핑거가 탐침) → X 270~570 / 30mm 간격 11선을 Y 265↔−115 지그재그로 훑고, 이어 X 방향 확인 탐색을 합니다. 각 경로의 Fy 보정표와 비교해 접촉점을 찾습니다. 탐색이 끝나면 접촉점 앞 부채꼴 최대 5점을 Force 하강해 윗면 높이를 탐지하고, 충돌 정지점에서도 1회 집기를 시도한 뒤 최초 탐지점에서 **윗면 − 15mm** 높이를 그리퍼 0° → 90° → 45°로 최대 3번 집습니다. 측정존 위에서 내려놓고 공통 WAYPOINT로 이동한 뒤 `True`를 반환합니다.
- **이어서 탐색**: 노드가 켜져 있는 동안 마지막 접촉 선·위치를 기억하고, 다음 Goal은 그 접촉 위치에서 이어서 훑습니다. 스캔 선 사이 이동은 탐색 높이에서 수행하며 매번 Z=160으로 올라가지 않습니다. 전체 탐색을 마치면 `False`(물품 없음)를 보내고 WAYPOINT에서 끝납니다. 노드를 다시 켜면 처음부터.
- **집기 실패**: 3번 다 못 잡으면 그리퍼를 열고 높이 100mm 이상에서 `GRASP_FAILED` → 관제가 공간 스캔부터 다시(같은 접촉점 −50mm부터 이어서).
- **스캔공간 막힘**: 4점 모두 유효 높이가 없으면 Y−50으로 돌아가 계속 훑는데, 같은 자리(같은 선, Y 30mm 안)에서 3번이면 `스캔공간 막힘` ERROR 로그 + `SCAN_FAILED`(임시, 처리 방식 팀 결정 대기 — `scan_algorithm.py` `_on_scan_blocked`).
- 측정존은 Goal의 `measure_zone`을 씁니다(**z = 측정존 바닥 Z**). 예상 접촉 높이(`z + 집은 높이 − Z0`) 15mm 위부터 10mm/s로 내려가며 Fz +5N이면 멈추고 놓습니다. 하한은 바닥 + 2mm.
- 집는 높이(11/18): 원본의 '윗면 높이의 50%' 대신 **윗면 − 15mm**(`GRIP_DEPTH_FROM_TOP_MM`). 박스가 겹쳐 있어도 맨 위 박스만 잡고, 가장 얇은 박스(20mm)도 바닥에 닿지 않습니다. 높이 탐지 하한도 `Z0 + 17mm`(15 + 여유 2)로 낮춤.
- 설정은 전부 `scan_config.py` (원본 값 그대로, 집기 각도 `GRIP_RETRY_ANGLES`, 막힘 횟수 `HEIGHT_MISS_LIMIT` 등). 로봇 연결·TCP·HOME·측정존은 관제(box_comm)·DB 값을 씁니다.
- **`sim_boxes` (에뮬레이터 전용)**: 에뮬레이터는 힘 센서가 0이라 그대로 돌리면 보정표 때문에 시작하자마자 가짜 접촉이 납니다. 가상 박스 `"x,y,높이;..."`를 주면 탐색 Fy를 보정표 값 + 가상 접촉으로 흉내 내고, 높이 탐지·집기 접근·내려놓기는 가상 높이까지 `movel`. 중심 탐지는 간략화된 가상 윗면 판정 영역을 사용하고, 집은 박스는 목록에서 빠집니다. 이 형상은 경로 확인용 근사이며 **실제 로봇에서는 켜지 않습니다.**
- `robot`에 추가한 명령(11/17): `amovel`(절대 비동기 직선, 회전 속도 지정), `movejx`(TCP 목표 관절 보간), `motion_state()`(check_motion).

### 적재 실제 로봇 실행 (`place_main`)

적재 실제 코드는 알고리즘 `box_place/box_place/place_algorithm.py`(적재 담당) + 실행 진입점 `place_main.py`, 설정은 `box_place/config/place_config.json`입니다 (빌드하면 `share/box_place/config/`에 설치). 로봇 명령·정지 확인은 `box_comm/dsr_io.py`, 실패 시 정지·결과 변환은 `box_comm/place_interface.py`(관제)가 맡습니다 — 구조는 CONTRIBUTING '적재 실제 코드 구조' 참고.

```bash
ros2 run box_place place_main                                        # 기본 설정 파일
ros2 run box_place place_main --ros-args -p show_force:=true         # 하강 중 힘·Z 출력
ros2 run box_place place_main --ros-args -p clear_pending:=true      # 미완료 기록을 시작할 때 지움 (보통은 필요 없음, 아래)
ros2 run box_place place_main --ros-args -p sim_contact:=true        # 에뮬레이터 전용 (아래)
```

- 지원 클래스는 설정에 있는 것만입니다 (현재 **A/B/C/D**, D는 11/18 실물 확인. 구역 위치는 다시 정할 예정). 설정에 없는 클래스가 오면 `PLACE_FAILED`.
- **미분류 `U`**: `place_config.json`의 `unclassified`(`enabled`, `release_pose` [x, y, z, rx, ry, rz], `approach_z`) 위치로 가서 힘 제어 없이 놓고 0을 돌려줍니다. 현재 `[793, -113, 186, 171, -117, -92]`, 접근 높이 186.
- Goal의 측정존이 `(0,0,0)`이면 움직이지 않고 실패합니다 → DB에 실제 측정존·구역 좌표가 있어야 합니다.
- 정지: `dsr_io`가 모든 로봇 명령 전과 힘 제어 하강 중(`force_z`)에 관제 정지 요청을 확인합니다 → `CANCELED`로 빠져나오고 `place_interface`가 힘·순응제어 해제 + `move_stop`. 하강 중에는 1초마다 keepalive를 보내 진행 감시가 오인하지 않습니다.
- **공정 인계 WAYPOINT / 이동 고도**: `common.TRAVEL_Z = 160 mm`, `WAYPOINT['z_mm']`도 이를 사용합니다. WAYPOINT XY는 DB의 측정존 중앙에서 가져오므로 예를 들어 `[300, -160, 160]`입니다. 스캔은 각 탐색 선 사이에서 이 고도로 올라가지 않고, 탐색 완료 뒤에만 WAYPOINT로 이동합니다. 판별은 WAYPOINT에서 시작해 측정·정렬 후 WAYPOINT로 돌아오고, 적재는 여기서 시작해 완료 후 HOME으로 갑니다. U 투하와 HOME 같은 별도 목적지는 예외입니다.
- `TRAVEL_Z` 이상에서 XY만 이동할 때 `TRAVEL_SPEED_MM_S = 100 mm/s`를 씁니다. 수직 상승·하강, 접촉 동작, 스캔 선 사이 이동에는 적용하지 않습니다. 따라서 이 값은 공정 전반의 속도 설정이 아니라 안전 고도에서의 수평 이송 속도입니다.
- **측정존 z = 측정존 바닥 Z** (DB 기준 **`[300, -160, 30.9]`**). 판별 높이 접촉 기준에는 별도 실측 보정 `h_floor_offset = -1.16 mm`를 적용하지만, DB 바닥 좌표와 sync 집기 기준은 바꾸지 않습니다.
- **측정존 집기 = sync 집기** (11/18 실물 시험 후 변경, `box_comm/sync_grip.py` = 판별 `rg2_sync_grip` 그대로, **판별과 같은 설정** `common.SYNC_GRIP`): RG2는 벌릴수록 핑거 끝이 올라갑니다(완전 열림 ↔ 닫힘 약 39mm, TCP 208.4는 닫힌 핑거 끝). 예전처럼 TCP를 바닥 + 5에 두고 닫으면 A 박스 60mm 면에서 핑거 끝이 바닥 위 약 17mm → 20mm 박스의 위 3mm만 물어 들다가 놓쳤습니다. 이제는 **100mm로 열고** 핑거 끝이 **바닥 + 2.6mm**에 오게 내려가서, 닫는 동안 핑거 끝이 내려가는 만큼 TCP를 올립니다. 10N으로 닫고 40N으로 다시 조입니다. 판별이 측정존 중심에 정해진 위상으로 놓아 주므로 하강 중 얹힘 감시는 하지 않습니다. 못 잡으면 같은 자리에서 3번.
- 집은 TCP 높이가 폭마다 달라서(TCP − 박스 바닥, A 60mm 면 약 −8mm) 집을 때마다 재고, 층 판단·첫 접촉 Z 학습은 **TCP Z − 그 값(= 박스 바닥)** 으로 계산합니다 → 기존 `first_contact_z_mm`(30.19 등)과 상태 파일을 그대로 씁니다. 로그: `집기 성공 (1/3번째, 폭 60.2 mm, TCP−바닥 -15.2 mm)`.
- 시험(ROS 없이): `python3 box_place/test/test_place_pick.py`. 가짜 그리퍼는 일정 속도로 닫힌다고 가정해 핑거 끝이 목표보다 2~3mm 낮게 계산됩니다(판별이 실물에서 같은 방식으로 문제없었으므로 모델이 보수적이라고 봄). **실물에서 바닥을 긁으면 `common.SYNC_GRIP` `tip_above_floor_mm` 하나만 올리면** 판별·적재가 같이 바뀝니다.
- **`sim_contact` (에뮬레이터 전용)**: 에뮬레이터는 힘 센서 값이 없어 '닿을 때까지 하강'이 끝나지 않습니다. `true`면 힘 제어 대신 가상으로 쌓인 높이까지 `movel`로 내려가 닿았다고 칩니다(층 판단·만재 시 다음 자리·수량 계산은 원래 코드). 가상 적재 수는 노드가 켜질 때 0, 상태 파일은 `~/.ros/place_sim_state.json`으로 분리. **실제 로봇에서는 켜지 않습니다.**
- **멈춘 뒤 복구 (11/18 자동)**: 작업 중 멈추면 상태 파일(`~/.ros/place_real_v04_state.json`, 에뮬레이터 `place_sim_state.json`)에 미완료 기록이 남습니다. 다음 Goal에서 **그리퍼를 읽어 비어 있으면 기록을 지우고 진행**(WARN 로그), 무언가를 쥐고 있으면 `PLACE_FAILED`(사람이 빼면 다음 Goal에서 풀림). 적재는 매번 1자리부터 실제로 닿아 보고 수량을 재므로 재고가 틀어지지 않습니다. **상태 파일을 지우지 마세요** — 자리별 첫 접촉 Z 기준까지 사라집니다. 관제 사전 점검도 그리퍼가 무언가를 쥐고 있으면 시작하지 않습니다.

### 그리퍼 (RG2 Modbus, 11/17)

세 파트 모두 그리퍼를 **Modbus TCP**(`box_comm/gripper.py`, Compute Box `192.168.1.1:502`)로 씁니다. 예전 디지털 출력(DO 1·2) 방식은 쓰지 않습니다(`robot`이 준비할 때 DO 1·2를 끔).
- 폭과 잡힘 여부를 읽을 수 있어 **집기 실패 감지**와 판별 폭 측정에 씁니다. 설정은 `common.GRIPPER`(열 폭 100mm, 여는 힘 20N, 잡는 힘 20N, 빈손 기준 패드 사이 간격 5mm, 폭 보정 = 판별 실측값).
- **잡힘 판정 (11/18, `gripper.check_grip`/`holding`, 세 파트 공용)**: 잡힘 비트만 믿지 않습니다. ① 패드 사이 간격이 **박스 범위 17~80mm** 안(`box_width_range_mm`) — 밖이면 한쪽 핑거만 걸려 멈춘 것 등 (**탐지·적재만**. 판별은 범위 밖 물체도 재서 U로 판정해야 하므로 안 봄) ② 들어 올린 뒤·이송 중 폭이 닫았을 때보다 **3mm 넘게 변하면** 놓친 것(`hold_width_tol_mm`) — 한쪽 핑거가 걸려 멈췄던 경우 들어 올리면 풀려서 계속 닫힘. DO 시절 탐지는 WebLogic 폭 조건(닫기 목표 20mm, `Width ≤ 20 AND Grip` → DI)으로 비슷한 확인을 했습니다.
- **쥐는 힘 40N** (`GRIPPER grip_force_n`, 11/18 20 → 40): 탐지는 바로 40N, 적재는 sync(10N)로 닫은 뒤 40N으로 조임. 판별은 폭 측정 보정이 10N 기준이라 10N.
- **집은 뒤 들어 올리기 (11/18)**: 40N이 다 걸리기 전에 들면 박스가 핑거 사이로 빠져서, 탐지·적재는 다 조인 뒤 `GRIPPER settle_after_grip_s`(0.5초) 기다리고 처음 `lift_slow_mm`(15mm)은 `lift_slow_vel`(20mm/s)로 천천히 듭니다.
- **폭 레지스터 부호 (11/18 수정)**: 패드 사이 폭(레지스터 275)은 부호 있는 값입니다. 부호 없이 읽으면 그리퍼가 닫힌 채 접속할 때 패드 두께 합이 −6543mm가 됐습니다(판별 실물, 판별 담당 제보). `gripper.py`에서 부호 있게 읽고, 그래도 패드 합이 0~30mm 밖이면 오프셋 × 2로 씁니다.
- **'닫으면서 핑거 끝 높이 유지'(sync 집기)** 는 `box_comm/sync_grip.py` + `common.SYNC_GRIP`(판별 폭 측정·적재 측정존 집기 공용). 탐지는 고정 높이(윗면 − 15mm)로 닫음 — 넓은 면을 잡으면 실제 무는 깊이가 얕아짐(이슈 25).
- **다시 집기** (`GRIP_RETRY` 3): 탐지는 각도 0/45/90°, 판별·적재는 같은 자리·같은 자세로. 판별 가로 하강에서 핑거가 박스에 얹히면 같은 자리로는 다시 해도 같으므로 바로 `GRASP_FAILED`. 최종 실패: 탐지 → 관제가 바로 재스캔, 판별·적재 → 관제가 측정존 비우기 → 재스캔. **밀어내기 경로(`RECOVERY push_path`)가 아직 없어서 비우기는 fake**(WARN 로그만, 대기 없음) — 스캔하는 동안 사람이 측정존을 치웁니다.
- Compute Box는 **동시 접속 1개**입니다. `robot`(DsrIO)이 그리퍼 명령을 처음 쓸 때 접속하고 Goal이 끝나면 끊어서, 탐지·판별·적재가 번갈아 씁니다.
- **실제 로봇**: `m0609_rg2_bringup mode:=real`로 띄우면 OnRobot 드라이버(`OnRobotRGControllerServer`)가 먼저 접속을 잡습니다 → `ros2 node list | grep -i onrobot`으로 확인하고 `pkill -f OnRobotRGControllerServer`로 끕니다(RViz 손가락 표시만 멈춤). 사전 점검이 접속을 확인해 막혀 있으면 FAIL.
- **에뮬레이터·fake**: 환경변수 `BOX_GRIPPER=fake` → 가짜 그리퍼(닫으면 잡힌 것으로). `BOX_FAKE_GRIP_FAILS=N`이면 처음 N번 닫기가 빈손. `fake_run.sh`는 기본 fake(`PLACE_MAIN=1`·`SCAN_MAIN=1`을 SIM 없이 돌리거나 `JUDGE_MAIN=1`일 때만 modbus).
- 처음 한 번: `sudo apt install python3-pymodbus`

### 정지 기능 (비상정지 · 로봇 정지 감지 · 진행 감시)

관제의 `abort_job()`은 작업을 끝내는 처리이고, 로봇을 멈추는 것은 아래 정지 기능이 합니다.

| 계기 | 언제 | 관제 동작 | DB 기록 |
| --- | --- | --- | --- |
| `/emergency_stop` (UI 버튼, 터미널) | 언제든 (시작 전이면 시작 안 함) | 두산 `move_stop` 반복 + 진행 중인 파트 Goal cancel | `STOPPED : EMERGENCY STOP — by ui` |
| 로봇 상태가 STANDBY·MOVING이 아님 (안전 정지, 펜던트 비상정지 등) | 작업 중 (`robot_monitor`) | 같음 | `STOPPED : Robot safety stop — robot_state=SAFE_STOP` 등 |
| 로봇 상태 확인 3초 연속 실패, `robot_disconnection` | 작업 중 (`robot_monitor=dsr`) | 같음 | `STOPPED : Robot disconnected` |
| 실행 중인 파트 Action 서버가 3초 넘게 사라짐 (노드 종료·충돌, 11/18) | 작업 중 (항상) | 같음 | `ERROR : Part node stopped (place) — …` |
| 파트 Feedback이 `progress_timeout_sec` 동안 없음 (살아 있는데 멈춤) | 작업 중 (켰을 때) | 같음 | `ERROR : No progress (judge) — …` |
| 관제 코드 자체의 예외 (11/18) | 언제든 | 같음 + 관제는 READY로 | `ERROR : Control error — KeyError: …` |

- 파트는 cancel을 받으면 다음 `report(stage)`·`report.check()`에서 `CANCELED`로 빠져나옵니다. `move_stop` 뒤에도 파트가 다음 동작을 보낼 수 있어 관제가 3초 동안 `move_stop`을 반복합니다(`common.SAFETY`).
- 두산 알람(`error` 토픽)은 이벤트 로그(`ROBOT_ALARM`)로 남고, 로봇 정지 이유에 덧붙습니다.
- 대기·카운트다운 중 터미널 키 입력에서 예상 못 한 오류가 나면 관제를 멈추지 않고 `주의!` 로그(`CONTROL_WARN`)만 남긴 뒤 키 입력 없이 계속합니다 (11/18). 멈추려면 UI·관리자 GUI 비상정지나 펜던트.
- 복구(로봇 안전 정지 해제)는 아직 수동입니다(펜던트). 해제 후 다시 시작합니다(상주 모드면 관제를 다시 켤 필요 없음 — 사전 점검이 로봇 상태·그리퍼를 확인).
- 파트 노드가 정상 종료(Ctrl+C)되면 바로, 강제 종료(`kill -9`, 충돌)되면 DDS가 알아차리는 데 십수 초 걸릴 수 있습니다.

| 관제 파라미터 | 값 | 설명 |
| --- | --- | --- |
| `robot_monitor` | `dsr`(기본) / `fake` / `off` | `dsr`: 실제 로봇·에뮬레이터에서 `get_robot_state`를 0.5초마다 확인. bringup이 없거나 `dsr_msgs2`가 없으면 사전 점검 FAIL(시작 안 함). `off`는 로봇 없이 테스트할 때만 (사전 점검 WARN, 정지 감지 없음) |
| `fake_robot_state` | int (기본 1 = STANDBY) | `robot_monitor:=fake`일 때 로봇 상태 흉내. 실행 중에 `ros2 param set`으로 바꿈 |
| `progress_timeout_sec` | float (기본 0 = 끔) | 파트 Feedback 감시 시간 ('살아 있는데 멈춤'). 파트 동작 중 가장 긴 구간보다 넉넉하게 (예: 60). 노드가 죽는 경우는 이 값 없이도 `PART_DIED`로 잡힘 |

```bash
# 실제 로봇·에뮬레이터 (robot_monitor는 기본값 dsr)
ros2 run box_control control --ros-args -p progress_timeout_sec:=60.0

# 로봇 없이 fake 파트로 (bringup 없음)
ros2 run box_control control --ros-args -p robot_monitor:=off

# 비상정지 버튼 대신 터미널에서
ros2 service call /emergency_stop box_interfaces/srv/EmergencyStop "{source: 'cli'}"

# 로봇 안전 정지 흉내 (로봇 없이, robot_monitor:=fake)
ros2 run box_control control --ros-args -p robot_monitor:=fake -p start_delay_sec:=3.0
ros2 param set /control fake_robot_state 5      # 5 = SAFE_STOP, 6 = EMERGENCY_STOP, 1 = STANDBY
```

| 환경 | 시험할 수 있는 것 |
| --- | --- |
| 로봇 없이 (fake 노드) | 비상정지 서비스·DB 노드 `estop_at`, `robot_monitor:=fake` 상태 변경, 진행 감시, fake 파트를 Ctrl+C로 끄기 → `PART_DIED` |
| 에뮬레이터 (`mode:=virtual`) | 위 + `robot_monitor:=dsr`로 실제 상태 읽기, `move_stop` 동작. 충돌·토크 안전 정지는 거의 안 생김 |
| 실제 로봇 | 위 + 펜던트 비상정지 버튼 → `STOPPED : Pendant emergency stop`, 충돌 감지 안전 정지 |

### 로봇 없이 실행 (fake)

`box_control`의 fake 노드로 전체 흐름을 로봇 없이 확인할 수 있습니다.

**터미널 하나로 (추천)** — DB 노드(`database_main` 파일 모드), fake 시작 재고 점검 및 fake 파트 3개는 백그라운드, 관제는 앞에서 실행. 관제가 끝나거나 Ctrl+C면 모두 정리됩니다. 시작 재고는 fake 점검 기본값 A~D=0을 사용합니다. (11/18: `fake_database`는 없어지고 DB는 항상 실제 코드)

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/box_project
./scripts/fake_run.sh                                        # 기본: robot_monitor=fake, 대기 3초
BOX_COUNT=6 CLASSES="A,A,B,C,D,D" ./scripts/fake_run.sh
DB_ARGS="-p estop_at:=2" ./scripts/fake_run.sh               # 2번째 박스 뒤 UI 비상정지 흉내
INVENTORY_ARGS="-p fake_inventory_counts:=[0,3,0,0]" PLACE_ARGS="-p fake_start:=[0,3,0,0]" ./scripts/fake_run.sh  # 초기 재고 B=3
RESIDENT=1 ./scripts/fake_run.sh                             # 상주 모드 (curl -s -X POST localhost:8765/start)
PLACE_ARGS="-p miscount_at:=2" ./scripts/fake_run.sh         # 재고 불일치
./scripts/fake_run.sh -p robot_monitor:=fake -p progress_timeout_sec:=5.0   # 관제 파라미터 직접 지정
MOTION=1 BOX_COUNT=2 ./scripts/fake_run.sh                   # 탐지·판별·적재를 test_*_motion으로 (로봇이 위아래로 움직임)
PLACE_MAIN=1 SIM=1 BOX_COUNT=5 CLASSES="A,A,A,A,B" ./scripts/fake_run.sh   # 적재만 place_main, 에뮬레이터 (가상 접촉)
PLACE_MAIN=1 BOX_COUNT=1 CLASSES="A" ./scripts/fake_run.sh   # 적재만 place_main, 실제 로봇 (실제 구역 좌표 필요)
SCAN_MAIN=1 SIM=1 ./scripts/fake_run.sh                      # 탐지만 scan_main, 에뮬레이터 (가상 박스 2개)
SCAN_MAIN=1 SIM=1 SIM_BOXES="420,100,10" ./scripts/fake_run.sh   # 낮은 박스(17mm 미만) → 높이 3회 실패 → 스캔공간 막힘
BOX_FAKE_GRIP_FAILS=4 SCAN_MAIN=1 SIM=1 ./scripts/fake_run.sh    # 탐지 집기 3회 실패 → 재스캔 → 성공
SCAN_MAIN=1 SIM=1 PLACE_MAIN=1 CLASSES="A,B" ./scripts/fake_run.sh   # 탐지·적재 실제 코드, 판별만 fake (에뮬레이터)
SCAN_MAIN=1 BOX_COUNT=1 ./scripts/fake_run.sh                # 탐지 scan_main, 실제 로봇 (작업공간에 박스, SIM 없이)
./scripts/fake_stop.sh                                       # 남은 노드 강제 정리
```

fake 노드 출력은 `/tmp/box_fake_logs/*.log`에 남습니다 (`tail -f /tmp/box_fake_logs/database.log`).

- `MOTION=1`, `PLACE_MAIN=1`, `SCAN_MAIN=1`, `JUDGE_MAIN=1`은 bringup(에뮬레이터 또는 실제 로봇)이 필요하고, 관제 기본값이 `robot_monitor=dsr`, 대기 5초로 바뀝니다.
- `SCAN_MAIN=1`: 탐지를 `box_scan scan_main`으로. `SIM=1`이면 `-p sim_boxes:="$SIM_BOXES"`(기본 `420,100,50;510,-20,60`). 탐지 Goal은 박스가 없을 때 11선을 다 훑어 1~2분 걸립니다. 판별 실제 코드(`judge_main`)는 시뮬 모드가 없어 에뮬레이터에서는 fake로 둡니다.
- `PLACE_MAIN=1`: 탐지·판별은 fake(움직이지 않음), 적재만 `box_place place_main`. `CLASSES` 기본값 `A,B,C`(D 미지원). `SIM=1`이면 `-p sim_contact:=true`.
- `PLACE_MAIN=1`로 멈췄다가 다시 돌릴 때 상태 파일을 지울 필요 없습니다 (11/18: 그리퍼가 비어 있으면 미완료 기록을 자동으로 지움).

**터미널을 따로 띄울 때**

```bash
ros2 run box_database database_main --ros-args -p store:=file
ros2 run box_control fake_inventory_check
ros2 run box_control fake_scan --ros-args -p box_count:=4 -p fake_delay_sec:=0.3
ros2 run box_control fake_judge --ros-args -p class_sequence:="A,B,B,D"
ros2 run box_control fake_place
ros2 run box_control control --ros-args -p start_delay_sec:=3.0     # 테스트할 때는 대기를 짧게
```

| 노드 | 파라미터 |
| --- | --- |
| `fake_scan` | `box_count` (박스 개수, 기본 3), `fake_delay_sec` |
| `fake_judge` | `class_sequence` (`"A,B,B,D"`처럼 주면 그 순서로 판별, 비우면 무작위), `fake_delay_sec` |
| `fake_inventory_check` | `fake_inventory_counts` (초기 A~D 재고, 기본 `[0,0,0,0]`), `fake_inventory_delay_sec`, `fake_move_robot` (기본 false, virtual launch에서만 true) |
| `fake_place` | `U`는 0 반환, `fake_delay_sec`, `fake_start` (A~D 구역에 처음부터 쌓인 개수, 기본 `[0,0,0,0]`), `miscount_at` (N번째 적재에서 수량을 일부러 +1 틀리게 측정 — 관제 불일치 처리 테스트용, 기본 0=끔) |
| `database_main` (파일 모드 시험용) | `estop_at` (N번째 `BOX_DONE` 뒤 UI 비상정지 흉내, 기본 0). Supabase에서는 무시. 시작 재고는 `fake_inventory_check`가 돌려주는 실측값으로 정합니다. 좌표 시험은 `~/box_logs/fake_db/loc.json`을 직접 고침(측정존 행을 지우면 사전 점검 FAIL) |

fake와 실제 노드는 같은 Action/Service 이름을 쓰므로 **동시에 띄우지 않습니다.**
파트 하나만 실제로 붙여 보려면 그 파트만 실제 노드로, 나머지는 fake로 띄웁니다.

### 로봇 연동 점검 (관제 테스트 도구)

두산 bringup(가상 또는 실제)을 띄운 상태에서 TCP·툴 자동 확인과 로봇 동작 구조를 점검합니다.

| 도구 | 내용 |
| --- | --- |
| `test_scan_motion` | 실제 파트 코드와 같은 알고리즘 방식(`start_algorithm`, `robot` = `DsrIO`)의 `/scan` 노드. Goal마다 HOME → 위로 50mm → 아래로 → HOME (11/18: DSR_ROBOT2 직접 호출 → `robot` 명령으로 바꿈, 정지 확인 자동) |
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
ros2 run box_database database_main --ros-args -p store:=file &
ros2 run box_control test_scan_motion  --ros-args -p robot_setup:=on -p box_count:=2 &
ros2 run box_control test_judge_motion --ros-args -p robot_setup:=on &
ros2 run box_control test_place_motion --ros-args -p robot_setup:=on &
sleep 5; ros2 run box_control control --ros-args -p auto_start:=true -p start_delay_sec:=5.0
pkill -f database_main; pkill -f _motion
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
