# box_project — 소형 박스 자동 분류 및 재고 관리 시스템

두산 M0609 + OnRobot RG2로 소형 박스의 크기와 무게를 측정해 A/B/C/D로 분류하고, 재고를 관리하는 ROS 2 시스템입니다.
관제(control)를 중심으로 한 허브 구조이고, 메시지 정의는 `box_interfaces`(v0.2)를 따릅니다.

---

## 1. 한눈에 보기

```
                 Service                         Action
  database  ◀──────────────▶  control  ◀──────────────▶  scan   (탐지)
  (DB·UI)    GetJobData        (관제)    /scan            judge  (판별)
             UpdateInventory             /judge           place  (적재)
             SetJobStatus                /place
```

- 관제 ↔ 탐지/판별/적재: **Action**. 관제 ↔ DB: **Service**. 토픽은 쓰지 않습니다.
- 파트끼리는 직접 통신하지 않습니다. 파트는 DB와 UI를 모릅니다. UI는 DB 값만 표시합니다.
- **실행되는 노드는 5개**입니다: `/control`, `/scan`, `/judge`, `/place`, `/database`
- 통신 코드는 공용 패키지 `box_comm`에 모여 있고, 각 파트는 이를 import해서 씁니다.
  각 파트 패키지에는 **자기 작업 파일 하나만** 있습니다.

---

## 2. 저장소 구성

```
ws/src/box_project/                 ← git 저장소
├─ README.md
├─ box_interfaces/                  메시지 정의 (.msg .srv .action)        [관제 관리]
├─ box_comm/                        공용 통신 라이브러리 (노드 아님)        [관제 관리]
│  └─ box_comm/
│     ├─ control_interface.py       관제용: Action 클라이언트 3, Service 클라이언트 3
│     ├─ scan_interface.py          Action 서버 /scan
│     ├─ judge_interface.py         Action 서버 /judge
│     ├─ place_interface.py         Action 서버 /place
│     └─ database_interface.py      Service 서버 3개
├─ box_control/                     관제                                   [관제]
│  └─ box_control/
│     ├─ control.py                 관제 시퀀스·상태 (정상 경로)
│     ├─ fake_scan.py               ┐
│     ├─ fake_judge.py              │ 관제 테스트 전용 가짜 노드
│     ├─ fake_place.py              │ (로봇 없이 관제를 돌려 볼 때)
│     └─ fake_database.py           ┘
├─ box_scan/box_scan/scan.py        탐지 작업 (템플릿)                     [탐지 담당]
├─ box_judge/box_judge/judge.py     판별 작업 (템플릿)                     [판별 담당]
├─ box_place/box_place/place.py     적재 작업 (템플릿)                     [적재 담당]
└─ box_database/box_database/database.py  DB 작업 (템플릿)                 [DB 담당]
```

각 패키지에는 이 밖에 `package.xml`, `setup.py`, `setup.cfg`, `resource/`가 있습니다.

| 패키지 | 역할 | 노드 | 실행 명령 |
| --- | --- | --- | --- |
| `box_interfaces` | 메시지 정의 | - | - |
| `box_comm` | 공용 통신 코드 | - | - (import 전용) |
| `box_control` | 관제 | `/control` | `ros2 run box_control control` |
| `box_scan` | 탐지 | `/scan` | `ros2 run box_scan scan` |
| `box_judge` | 판별 | `/judge` | `ros2 run box_judge judge` |
| `box_place` | 적재 | `/place` | `ros2 run box_place place` |
| `box_database` | DB | `/database` | `ros2 run box_database database` |

> 템플릿(`scan.py`, `judge.py`, `place.py`, `database.py`)은 지금 fake로 동작합니다.
> `[FAKE]` 표시된 부분을 실제 동작으로 바꿔 가면 됩니다.

---

## 3. 담당자가 지킬 약속

작업 파일에서는 **클래스 하나와 약속된 함수만** 만들면 됩니다. 통신은 `box_comm`이 처리하므로 ROS 메시지 타입을 몰라도 됩니다.

### 탐지 — `box_scan/scan.py`

```python
from box_comm.scan_interface import start, PartError

class ScanWork:
    def __init__(self, node): ...
    def run(self, report) -> bool:
        # 스캔 → 물품 감지 → 픽업 → 측정존에 두기 → 홈 복귀
        # 물품을 측정존에 두었으면 True, 작업공간에 물품이 없으면 False

def main():
    start(ScanWork)
```

### 판별 — `box_judge/judge.py`

```python
def run(self, specs, report) -> str:
    # specs = [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
    #           'height_mm': 40.0, 'weight_g': 100.0}, ...]
    # 측정 후 'A' / 'B' / 'C' / 'D' 중 하나를 반환
```

### 적재 — `box_place/place.py`

```python
def run(self, result_class, inventory, report) -> dict:
    # result_class = 'B'
    # inventory    = {'A': 1, 'B': 0, 'C': 2, 'D': 0}   (적재 전)
    # 적재 후 inventory를 같은 형식으로 반환
    # 슬롯 배정·좌표는 이 파일 안에서만 다룸
```

### DB — `box_database/database.py`

```python
def get_job_data(self) -> tuple:           # (specs, inventory) 반환
def update_inventory(self, box_id, inventory) -> None:
def set_job_status(self, status) -> None:  # 'BEFORE' / 'RUNNING' / 'DONE'
```

### 공통 규칙

- **실패하면 `PartError`를 raise합니다.** `box_comm`이 Result(`success=False`, `error_code`)로 바꿔 관제에 보냅니다.
  ```python
  raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
  ```
- **등록된 `error_code`만 쓸 수 있습니다.** 등록되지 않은 코드는 실행 중 에러가 납니다.

  | 코드 | 뜻 |
  | --- | --- |
  | `NOT_FOUND` | 물품을 찾지 못함 |
  | `GRASP_FAILED` | 집기 실패 |
  | `OVER_FORCE` | 과도한 힘 감지 |
  | `MEASURE_FAILED` | 측정 실패 |
  | `NO_SLOT` | 적재할 칸 없음 |
  | `PLACE_FAILED` | 적재 실패 |

- **예상하지 못한 예외도 괜찮습니다.** `box_comm`이 잡아서 파트별 기본 코드로 Result를 보냅니다. 어떤 경우에도 Result 없이 끝나지 않습니다.
- `report('단계명')`은 진행 상황 알림(Feedback)입니다. 쓰지 않아도 됩니다.
- 로봇 동작과 홈 복귀가 **끝난 뒤에** 반환합니다.
- 좌표는 로봇 base 기준, 단위는 mm입니다.
- DB에서 같은 `box_id` 중복 무시와 `status` 값 검사는 `box_comm`이 이미 처리합니다.
- 작업 파일은 자유롭게 나눠도 됩니다. 모션·측정 함수를 같은 패키지 안의 다른 모듈로 분리해서 import해도 됩니다.

---

## 4. 통신 계약 요약 (`box_interfaces` v0.2)

### Action (관제 → 파트)

| 이름 | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `/scan` | 없음 (시작명령) | `success`, `found`, `error_code` | `stage` |
| `/judge` | `specs: ClassSpec[]` | `success`, `result_class`, `error_code` | `stage` |
| `/place` | `result_class`, `inventory` | `success`, `inventory`(적재 후), `error_code` | `stage` |

- `/scan`에서 `found=False`는 실패가 아닙니다. "더 이상 물품 없음"이며 `success=True`입니다.

### Service (관제 → DB)

| 이름 | Request | Response | 호출 시점 |
| --- | --- | --- | --- |
| `/get_job_data` | 없음 | `success`, `specs`, `inventory`, `message` | 작업 시작 시 |
| `/update_inventory` | `box_id`, `inventory` | `success`, `message` | 적재완료마다 |
| `/set_job_status` | `status` | `success` | 작업중, 작업완료 시 |

---

## 5. 관제 흐름 (`box_control/control.py`)

```
작업전   GetJobData로 specs, inventory 수신
작업중   SetJobStatus(RUNNING)
  ┌─ 스캔중   /scan Goal ──▶ found=False면 루프 종료
  │  스캔완료
  │  판별중   /judge Goal(specs)
  │  판별완료  result_class 수신
  │  적재중   /place Goal(result_class, inventory)
  │  적재완료  inventory 수신 → UpdateInventory(box_id, inventory)
  └─ 반복
작업완료  SetJobStatus(DONE)
```

- "-중" 상태는 Goal을 보내기 직전, "-완료" 상태는 Result를 받은 직후에 바뀝니다.
- 단계 상태는 관제 내부에만 있고, DB에는 `BEFORE` / `RUNNING` / `DONE`과 재고만 기록합니다.
- `box_id`는 관제가 `box_001`, `box_002`처럼 순번으로 붙입니다.
- 관제 통신 함수는 모두 `(값, err)` 형태로 반환합니다. `err`가 `None`이면 성공입니다.
  관제가 내부적으로 만드는 코드: `SERVER_DOWN`, `REJECTED`, `TIMEOUT`, `NO_ERROR_CODE`, `DB_FAILED`

---

## 6. 처음 받기와 빌드

`~/ws`는 두산 로봇 패키지(`doosan-robot2` 등)가 있는 워크스페이스입니다. (예: `~/ws_cobot_pjt/ws_dsr`)
탐지·판별·적재 코드가 `DSR_ROBOT2`를 쓰므로 반드시 같은 워크스페이스의 `src` 안에 받습니다.

```bash
# GitHub 로그인 (처음 한 번)
sudo apt install gh
gh auth login                       # GitHub.com → HTTPS → Yes → Login with a web browser
git config --global user.name "이름"
git config --global user.email "GitHub 가입 이메일"

# 받기
cd ~/ws/src
git clone <저장소 주소> box_project

# 빌드 (두산 패키지는 다시 빌드하지 않음)
cd ~/ws
colcon build --symlink-install --packages-up-to box_control box_scan box_judge box_place box_database
source install/setup.bash
ros2 pkg list | grep box_           # 7개가 나오면 성공
```

`--symlink-install`을 쓰면 파이썬 파일을 고쳤을 때 다시 빌드하지 않아도 바로 반영됩니다.
빌드 중 `pytest-repeat` 관련 `UserWarning`은 시스템 경고이므로 무시해도 됩니다.

---

## 7. 실행

### 전체 실행

서버 4개를 먼저 띄우고 **관제를 마지막에** 실행합니다. 터미널을 각각 하나씩 씁니다.

```bash
ros2 run box_database database
ros2 run box_scan scan              # fake 템플릿: --ros-args -p box_count:=5
ros2 run box_judge judge
ros2 run box_place place
ros2 run box_control control
```

fake 템플릿의 동작 시간은 `--ros-args -p fake_delay_sec:=0.3`으로 줄일 수 있습니다.

### 파트 단독 테스트

관제 없이 자기 노드만 띄워서 Goal/Request를 직접 보내 봅니다. Result가 돌아오면 통신은 정상입니다.

```bash
# 탐지
ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback

# 판별
ros2 action send_goal /judge box_interfaces/action/Judge \
  "{specs: [{class_id: 'A', width_mm: 40.0, depth_mm: 40.0, height_mm: 40.0, weight_g: 100.0},
            {class_id: 'B', width_mm: 60.0, depth_mm: 40.0, height_mm: 30.0, weight_g: 150.0}]}" --feedback

# 적재
ros2 action send_goal /place box_interfaces/action/Place \
  "{result_class: 'B', inventory: {a: 1, b: 0, c: 2, d: 0}}" --feedback

# DB
ros2 service call /get_job_data box_interfaces/srv/GetJobData "{}"
ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}"
ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \
  "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0}}"
```

---

## 8. 관제 테스트용 fake 노드 (관제 전용)

`box_control` 안의 `fake_*` 노드는 **팀원 코드가 실제 로봇 코드로 바뀐 뒤에도** 로봇 없이 관제를 테스트하기 위한 것입니다.
`box_comm`을 그대로 쓰므로 관제 입장에서는 실제 노드와 똑같이 응답합니다. 팀원은 쓰지 않아도 됩니다.

```bash
ros2 run box_control fake_database
ros2 run box_control fake_scan --ros-args -p box_count:=4 -p fake_delay_sec:=0.3
ros2 run box_control fake_judge --ros-args -p class_sequence:="A,B,B,D"
ros2 run box_control fake_place
ros2 run box_control control
```

| 노드 | 파라미터 |
| --- | --- |
| `fake_scan` | `box_count` (박스 개수, 기본 3), `fake_delay_sec` |
| `fake_judge` | `class_sequence` (예: `"A,B,B,D"`를 주면 그 순서대로 판별, 비우면 무작위), `fake_delay_sec` |
| `fake_place` | `fake_delay_sec` |
| `fake_database` | 없음. 상태와 재고가 바뀔 때마다 로그로 출력 |

> 같은 이름의 Action/Service가 두 개 뜨면 안 되므로, fake와 실제 노드는 **동시에 띄우지 않습니다.**
> 예: 탐지만 실제 코드로 테스트하려면 `box_scan scan` + `fake_judge` + `fake_place` + `fake_database` + `control`.

---

## 9. 두산 로봇 연동 시 주의

- `box_comm`이 만든 노드(`self.node`)와 **로봇 제어용 노드를 분리**합니다. `DSR_ROBOT2` 함수는 등록된 노드를 내부에서 spin하기 때문에, 같은 노드를 쓰면 충돌할 수 있습니다.
- 예시는 `box_scan/scan.py`의 `DR_init` 주석을 참고합니다. 노드 이름만 파트별로 다르게(`scan_dsr`, `judge_dsr`, `place_dsr`) 합니다.
- 탐지·판별·적재 세 프로세스가 같은 로봇 한 대에 연결되므로, 처음 연동할 때 동시 연결에 문제가 없는지 확인합니다.

---

## 10. Git 사용 규칙

모두 `main` 브랜치 하나에서 작업합니다. 각자 고치는 폴더가 완전히 나뉘어 있어서 충돌이 거의 생기지 않습니다.

### 수정 권한

| 누가 | 고치는 폴더 |
| --- | --- |
| 관제 | `box_interfaces/`, `box_comm/`, `box_control/`, `README.md` |
| 탐지 | `box_scan/` |
| 판별 | `box_judge/` |
| 적재 | `box_place/` |
| DB | `box_database/` |

- `box_interfaces/`, `box_comm/`은 **관제만 수정**합니다. 필드 추가, 새 `error_code`, 이름 변경이 필요하면 관제에게 요청합니다.
- **다른 사람의 폴더와 공용 파일은 건드리지 않습니다.**

### 매일 작업 루틴

```bash
cd ~/ws/src/box_project

git pull                                 # ① 작업 시작 전: 최신 받기

# ... 자기 폴더에서 작업 ...

git add box_scan/                        # ② 자기 폴더만 담기 (git add . 금지)
git commit -m "scan: 외력 감지 로직 추가"  # ③ 무엇을 했는지 적어서 저장
git pull                                 # ④ 올리기 직전에 한 번 더 받기
git push                                 # ⑤ 업로드
```

### 관제가 공용 패키지를 고쳤다고 알렸을 때

```bash
cd ~/ws/src/box_project && git pull
cd ~/ws
colcon build --symlink-install     # box_interfaces가 바뀌었거나 파일이 추가됐을 때 필수
source install/setup.bash
```

- `box_comm`의 파이썬 내용만 바뀐 경우, `--symlink-install`로 빌드해 두었다면 `git pull`만 해도 반영됩니다.
- `run()`의 입력/출력 형식이 바뀌는 변경은 관제가 미리 알리며, 이때는 해당 파트도 코드를 수정해야 합니다.

### 자주 만나는 상황

| 화면 | 할 일 |
| --- | --- |
| `! [rejected] ... (fetch first)` | 다른 사람이 먼저 올림 → `git pull` 후 다시 `git push` |
| pull 후 편집기 화면이 열림 | 자동 합치기 메모 → vim이면 `:wq` Enter, nano면 `Ctrl+X` |
| `CONFLICT` | 손대지 말고 관제에게 연락 |
| `Authentication failed` | `gh auth login` 다시 하기 |
| 내 상태 확인 | `git status` |

---

## 11. 아직 구현하지 않은 것 (관제 TODO)

| 항목 | 넣을 위치 |
| --- | --- |
| 시작 버튼 연동 (지금은 실행하면 바로 시작) | `control.py` 시퀀스 앞부분 |
| 상태 확인·환경 세팅, 측정존 비우기 (WBS 0-2, 0-3) | `control.py` 시퀀스 앞부분 |
| `error_code`별 처리 (재시도, 건너뛰기, 정지) | `control.py`의 `stop()` |
| Action별 타임아웃, cancel, 정지 버튼 | `box_comm/control_interface.py`의 `_call_action` |
| `ERROR` 상태 (SetJobStatus 값 추가) | `box_interfaces` + `box_comm` |
