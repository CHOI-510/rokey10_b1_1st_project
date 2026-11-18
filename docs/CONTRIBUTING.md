# 팀 작업 규칙

이 문서는 팀원이 **처음 세팅하고, 자기 파트를 구현하고, Git으로 올리는 방법**을 정리합니다.
시스템 전체 구조는 [`README.md`](../README.md), 진행 상황은 [`PROGRESS.md`](PROGRESS.md)를 봅니다.

---

## 1. 처음 세팅 (한 번만)

### 1-1. GitHub 로그인

```bash
sudo apt install gh
gh auth login
```

질문이 나오면 방향키로 고르고 Enter를 누릅니다.

1. `What account do you want to log into?` → **GitHub.com**
2. `What is your preferred protocol?` → **HTTPS**
3. `Authenticate Git with your GitHub credentials?` → **Y**
4. `How would you like to authenticate?` → **Login with a web browser**
5. 화면의 8자리 코드를 브라우저에 입력하고 **Authorize**

```bash
git config --global user.name "GitHub 아이디"
git config --global user.email "GitHub 가입 이메일"
```

### 1-2. 저장소 받기와 빌드

`~/ws`는 **두산 로봇 패키지가 있는 워크스페이스**입니다. (예: `~/ws_cobot_pjt/ws_dsr`)
탐지·판별·적재 코드가 `DSR_ROBOT2`를 쓰므로 반드시 같은 워크스페이스의 `src` 안에 받습니다.

```bash
cd ~/ws/src
git clone https://github.com/CHOI-510/rokey10_b1_1st_project.git box_project

cd box_project
git config core.hooksPath .githooks   # 관제 전용 파일 보호 훅 켜기 (팀원 필수, 한 번만)

cd ~/ws
colcon build --symlink-install --packages-up-to box_control box_scan box_judge box_place box_database
source install/setup.bash
ros2 pkg list | grep box_       # 7개가 나오면 성공
```

- 폴더 이름은 모두 `box_project`로 맞춥니다. (`clone` 뒤에 `box_project`를 꼭 붙이기)
- `core.hooksPath` 설정은 관제 전용 파일(`box_comm/`, `box_interfaces/`, `*_template.py`)을 실수로 commit하지 않게 막아 줍니다 (3-1 참고).
- `--symlink-install`로 빌드하면 파이썬 파일을 고쳐도 다시 빌드하지 않아도 됩니다.
- 빌드 중 `pytest-repeat` 관련 `UserWarning`은 시스템 경고이므로 무시합니다.

### 1-3. 동작 확인 (fake)

터미널 5개를 열고, 각 터미널에서 먼저 `source ~/ws/install/setup.bash`를 합니다.

```bash
ros2 run box_control fake_database
ros2 run box_control fake_scan --ros-args -p box_count:=3 -p fake_delay_sec:=0.3
ros2 run box_control fake_judge
ros2 run box_control fake_place
ros2 run box_control control          # 마지막에 실행 (모니터 GUI 창이 같이 뜸, 끄기 -p gui:=false)
```

관제는 10초 카운트다운 뒤에 시작합니다. 기다리기 싫으면 `s`를 누르면 바로 시작합니다.
관제 터미널에 `===== DONE (작업완료) =====`가 나오면 환경 세팅이 끝난 것입니다.

---

## 2. 내 파트 구현하기

### 2-1. 템플릿은 참고용, 메인 코드는 내가 만든다

| 담당 | 참고용 템플릿 (관제 관리, **수정 금지**) | 실행 |
| --- | --- | --- |
| 탐지 | `box_scan/box_scan/scan_template.py` | `ros2 run box_scan scan_template` |
| 판별 | `box_judge/box_judge/judge_template.py` | `ros2 run box_judge judge_template` |
| 적재 | `box_place/box_place/place_template.py` | `ros2 run box_place place_template` |
| DB·UI | `box_database/box_database/database_template.py` | `ros2 run box_database database_template` |

- 통신(Action/Service 서버, Result 보내기)은 `box_comm`이 처리합니다. **신경 쓰지 않아도 됩니다.**
- 템플릿은 fake로 바로 동작합니다. **템플릿은 고치지 않습니다.** 관제가 인터페이스·공통 규칙을 바꾸면 템플릿을 업데이트하고 공지합니다 (3-4 참고).
- 메인 코드는 템플릿을 복사해서 **알고리즘 파일 + 실행 진입점** 두 개로 만듭니다 (11/17, 적재와 같은 방식). 아래는 탐지 예입니다.

```bash
cd ~/ws/src/box_project/box_scan
cp box_scan/scan_template.py box_scan/scan_algorithm.py      # ScanAlgorithm — 내 로직 (main()은 지워도 됨)
cat > box_scan/scan_main.py <<'PY'
from box_comm.scan_interface import start_algorithm
from box_scan.scan_algorithm import ScanAlgorithm

def main():
    start_algorithm(ScanAlgorithm)
PY
# setup.py의 console_scripts에 한 줄 추가:
#     'scan_main = box_scan.scan_main:main',
cd ~/ws && colcon build --symlink-install --packages-select box_scan && source install/setup.bash
ros2 run box_scan scan_main
```

| 파트 | 알고리즘 파일 (담당자) | 진입점 | 실제 예시 |
| --- | --- | --- | --- |
| 탐지 | `box_scan/scan_algorithm.py` (`ScanAlgorithm`) + 설정 `scan_config.py` | `scan_main.py` | 통합 (에뮬레이터 검증, 실물 확인 전) |
| 판별 | `box_judge/judge_algorithm.py` (`JudgeAlgorithm`) | `judge_main.py` | — |
| 적재 | `box_place/place_algorithm.py` (`PlaceAlgorithm`) | `place_main.py` | 완성 (실제 로봇 검증) |

- 메인 코드의 `[FAKE]` 부분을 실제 동작으로 바꿔 갑니다.
- **템플릿 노드와 내 노드를 동시에 켜지 마세요.** 서버 이름(`/scan` 등)이 같아서 관제가 엉뚱한 결과를 받습니다.
- 모션·측정 함수가 길어지면 **같은 패키지 안에 파일을 추가**해서 import해도 됩니다.
  예: `box_scan/box_scan/motion.py` → `scan_algorithm.py`에서 `from box_scan.motion import ...`

### 2-2. 지켜야 할 약속: 알고리즘 클래스의 입력과 출력

세 파트 모두 같은 모양입니다. 생성자는 `(node, robot)`, 작업 함수는 `(..., poses, report)`.
`robot`은 `box_comm`이 만들어 넣어 주는 로봇 객체(`box_comm/dsr_io.py`, 아래 2-3-1), `poses`는 Goal로 받은 DB 고정 좌표입니다.

**탐지 — `scan_algorithm.py`**

```python
from box_comm.scan_interface import PartError

class ScanAlgorithm:
    def __init__(self, node, robot): ...
    def scan(self, poses, report) -> bool:
        # 스캔 → 물품 감지 → 픽업 → 측정존에 두기 → 홈 복귀
        # 물품을 측정존에 두었으면 True, 작업공간에 물품이 없으면 False
```

**판별 — `judge_algorithm.py`**

```python
class JudgeAlgorithm:
    def __init__(self, node, robot): ...
    def judge(self, specs, poses, report) -> str:
        # specs = [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
        #           'height_mm': 40.0, 'weight_g': 100.0}, ...]
        # 측정 후 'A' / 'B' / 'C' / 'D' 중 하나를 반환
        # 어느 기준에도 맞지 않으면 'U' (미분류, v0.5)
```

**적재 — `place_algorithm.py`**

```python
class PlaceAlgorithm:
    def __init__(self, node, robot): ...
    def place(self, result_class, poses, report) -> int:       # v0.4
        # result_class = 'B'
        # 매번 1단부터 박스를 쥔 채 내려가며 놓을 자리를 찾고(최상층 위 높이에서 닿으면 다음 단),
        # 놓은 위치로 잰 수량을 반환: count_from_position(단, 층)  예: 2단 2층 → 5
        # 단·층·z는 관제에 보내지 않음 (자기 로그로만 남김)
        # 구역 1단 x, y는 poses['place_zones']['B'] (DB 값)
        # result_class = 'U' (미분류, v0.5): 미분류 투하 위치에 놓기만 하고 0을 반환 (수량은 관제가 셈)
```

> 예전 방식(`class ScanWork: run(self, report)` + `start(ScanWork)`, 좌표는 `self.poses`)도 아직 동작합니다.
> `box_control`의 fake·`test_*_motion` 노드가 이 방식입니다. 새로 짜는 메인 코드는 위 알고리즘 방식으로 합니다.

**DB — `database.py`**

```python
def get_job_data(self) -> tuple:           # (specs, inventory) 반환
def update_inventory(self, box_id, inventory) -> None:
def set_job_status(self, status, reason) -> None:
    # status: 'BEFORE' / 'RUNNING' / 'DONE' / 'ERROR' / 'STOPPED'
    # reason: ERROR·STOPPED일 때 이유 ('Fail to pick', 'EMERGENCY STOP'), 그 밖에는 ''
    # UI 표시: f'{status} : {reason}'  (ERROR = 공정 오류, STOPPED = 로봇 정지 → 색을 다르게)
def get_fixed_poses(self) -> dict:         # DB에 저장된 고정 좌표 (기준값), 없으면 None
    # {'home_j': [6개, deg], 'measure_zone': [x, y, z, mm],
    #  'place_zones': {'A': [x, y], 'B': [x, y], 'C': [x, y], 'D': [x, y]}}   ← v0.4 적재 구역 1단
    # 관제가 이 값을 모든 파트 Goal에 실어 보낸다. 없거나 구역이 빠지면 관제가 작업을 시작하지 않음
# UI 적재 이미지: box_comm.common.stack_layout(수량) → 자리별 층 수 (예: 5 → [3, 2], 3층 x 2자리)
# UI 비상정지 버튼: self.node.request_emergency_stop('ui')   (관제가 없으면 False)
```

### 2-2-1. Feedback stage (필수)

작업 함수(`scan`·`judge`·`place`) 안에서 단계가 바뀔 때마다 `report('stage')`를 호출합니다. 값은 아래 목록에서만 씁니다. (`box_comm/common.py`의 `STAGES`)
목록에 없는 값을 보내면 경고 로그가 남습니다. 새 stage가 필요하면 관제에게 요청합니다.

**DB·UI에 남길 일은 `report.log()`** 로 보냅니다. 관제가 받아서 `/event_log` 토픽으로 DB에 넘깁니다. (파트가 DB에 직접 보내지 않음)

```python
report('picking')                                  # 진행 단계 → 로그로 안 올라감
report.log('집기 실패, 재시도 2/3', 'WARN')         # 지금 단계 그대로 + 로그로 올라감
report.log('정렬 완료 (90° 회전)')                  # level 기본값 'INFO'
```

- level: `'INFO'` / `'WARN'` / `'ERROR'`
- 남길 만한 것: 스캔 결과, 집기 재시도, 정렬·회전, 단 꽉 참 등 나중에 확인하고 싶은 일. 매 단계마다 쓸 필요는 없습니다.

| 파트 | stage 순서 |
| --- | --- |
| 탐지 | `space_scan` → `picking` → `moving` → `placing` |
| 판별 | `measuring_length1` → `measuring_length2` → `measuring_length3` → `measuring_weight` → (`measuring_materiality`, 옵션) → `judging` |
| 적재 | `picking` → `moving` → `placing` |

```python
def run(self, report) -> bool:
    report('space_scan')
    ...                      # 스캔
    report('picking')
    ...                      # 집기 (재시도도 이 단계 안에서)
    report('moving')
    ...
    report('placing')
    ...
```

- 판별의 길이 측정은 물품 방향에 따라 가로·세로·높이가 달라질 수 있어 번호로 구분합니다. 템플릿의 `classify()`는 측정한 길이 3개와 기준 W·D·H를 각각 **크기순으로 정렬해서** 비교하므로, 측정 순서가 바뀌어도 판별에는 영향이 없습니다.
- 적재가 잰 수량은 관제가 검증합니다. 기대값(**적재 전 재고의 `result_class` 수량 + 1**)과 다르면 관제가 **측정값**으로 DB에 반영하고 경고를 남깁니다 (v0.4: 실제 적재 상태가 기준).
- 미분류 `U`(v0.5)는 쌓지 않으므로 적재가 세지 않습니다. 관제가 적재 성공마다 재고 `U`를 +1 해서 DB에 보냅니다.

### 2-3. 실패 처리

실패하면 `PartError`를 raise합니다. `box_comm`이 Result(`success=False`, `error_code`)로 바꿔 관제에 보냅니다.

```python
raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
```

| 코드 | 뜻 |
| --- | --- |
| `NOT_FOUND` | 물품을 찾지 못함 |
| `GRASP_FAILED` | 집기 실패 — **정해진 횟수만큼 다시 집어 보고 포기했을 때만** (아래 2-3-2). 관제가 공간 스캔부터 다시 |
| `OVER_FORCE` | 과도한 힘 감지 |
| `MEASURE_FAILED` | 측정 실패 |
| `NO_SLOT` | 적재할 칸 없음 |
| `SCAN_FAILED` | 탐지 실패 (집기 말고 예상 못 한 오류 — 탐지 기본 코드) |
| `PLACE_FAILED` | 적재 실패 |
| `ROBOT_SETUP_FAILED` | 기본 TCP·툴 적용 실패 (`box_comm`이 작업 함수 전에 보냄, 파트 코드에서는 쓰지 않음) |
| `CANCELED` | 관제가 정지를 요청함 (`box_comm`이 robot 명령·`report()`에서 자동으로 냄, 파트 코드에서는 쓰지 않음) |

- **등록된 코드만** 쓸 수 있습니다. 새 코드가 필요하면 관제에게 요청합니다.
- 예상하지 못한 예외가 나도 `box_comm`이 잡아서 Result를 보냅니다. 관제가 멈추지 않습니다.
  알고리즘 방식이면 그 전에 로봇이 움직였거나 힘 제어 중이었을 때 **로봇을 멈춥니다**(힘·순응제어 해제 + `move_stop`). 코드는 파트 기본값(탐지 `SCAN_FAILED`, 판별 `MEASURE_FAILED`, 적재 `PLACE_FAILED`). 예상 못 한 오류는 재스캔하지 않고 작업을 끝냅니다.
- DB는 `PartError` 없이 아무 예외나 raise하면 `success=False`로 전달됩니다.

**정지 (비상정지·로봇 이상 감지)**: 관제가 정지를 요청하면 로봇에 `move_stop`을 보내고 내 Goal을 취소합니다.
알고리즘 방식에서는 **`self.robot`의 명령이 보내기 직전에 정지 요청을 확인**해서 `PartError('CANCELED')`로 빠져나옵니다. `report.check()`를 넣을 필요가 없습니다. `report(stage)`에서도 확인합니다.

```python
report('moving')
self.robot.movel(above, vel=60, acc=100)     # 보내기 전에 정지 확인 → 정지 요청이면 여기서 CANCELED
self.robot.movel(target, vel=60, acc=100)
```

- 힘 제어 하강처럼 **오래 걸리는 루프**는 매 반복 `self.robot.force_z()`를 읽으면 됩니다. 1초에 한 번 관제에 진행 알림(keepalive)이 자동으로 갑니다. 힘을 안 읽는 긴 대기 루프라면 `report.keepalive()`를 부릅니다.
- `except PartError`로 잡아서 계속 진행하지 마세요. 잡아야 한다면 정리(그리퍼 열기 등)만 하고 다시 `raise` 합니다.
- `except Exception:`으로 모든 예외를 삼키는 코드도 `CANCELED`를 막으니 피합니다.
- 한 번에 Goal 하나만 실행됩니다. 실행 중에 온 새 Goal은 `box_comm`이 거절합니다.

### 2-3-1. 파트 코드 구조와 로봇 명령 (11/17, 세 파트 공통)

알고리즘(담당자)과 관제 연동을 파일로 나눴습니다. **서로 다른 파일만 고칩니다.**

| 파일 | 소유 | 하는 일 |
| --- | --- | --- |
| `box_xxx/xxx_algorithm.py` | 파트 | 내 로직 전부. **관제 코드 없음** (적재는 설정 `config/place_config.json`도 파트 소유) |
| `box_xxx/xxx_main.py` | 파트 | 실행 진입점 한 줄 `start_algorithm(XxxAlgorithm)` |
| `box_comm/dsr_io.py` | 관제 | 로봇 명령 모음 `DsrIO`. **명령 전에 정지 확인, `force_z()`·`force()`에서 1초마다 keepalive** |
| `box_comm/gripper.py` | 관제 | RG2 그리퍼 Modbus (판별 실측 코드를 옮김, 세 파트 공용). `robot`이 Goal 동안만 접속 |
| `box_comm/algorithm.py` | 관제 | 알고리즘에 `robot`·`poses`를 넣고, 실패하면 로봇 정지 + `PartError` 변환 |
| `box_comm/xxx_interface.py` | 관제 | Action 서버, TCP·툴 확인, 좌표 꺼내기, Result 만들기 |

`self.robot`이 제공하는 명령 (좌표는 BASE 기준 mm·deg)

| 명령 | 내용 |
| --- | --- |
| `movel(pose6, vel, acc, tol=2.0)` | 직선 이동 후 도달 확인 (오차가 tol mm를 넘으면 예외) |
| `movej(joints6, vel, acc)` | 관절 이동 (예: `poses['home_j']`) |
| `grip(opened)` | RG2 기본 열기(`True`)/닫기(`False`). 닫기는 아래 `grip_close()` 결과를 돌려줌 (11/17부터 Modbus, `wait_s`는 무시) |
| `grip_open(width_mm, force_n)` | 지정한 패드 간격으로 열기 (기본 `common.GRIPPER`) |
| `grip_close(force_n)` | 닫고 `(grabbed, width_mm, raw_mm)` 반환 — **잡혔는지·폭**. 못 잡아도 예외 없음 |
| `gripper_width()` / `gripper` | 지금 폭 [mm] / RG2 객체 그대로 (판별 폭 측정처럼 세밀한 제어) |
| `pose()` / `joints()` | 현재 TCP `[x, y, z, a, b, c]` / 관절각 6개 [deg] |
| `force_z()` / `force(ref)` | 툴 외력 Fz [N] / 6축 `[Fx, Fy, Fz, Tx, Ty, Tz]` (`ref='base'`·`'tool'`, 힘은 BASE·모멘트는 ref 기준). 둘 다 진행 알림 포함 |
| `amovel_rel(delta6, vel, acc, time_s)` / `mwait()` / `move_stop(mode)` | 비동기 상대 이동 / 끝까지 대기 / 그 이동만 멈춤 (1 빠른, 2 부드러운) — 접촉하면 바로 멈추는 하강용 |
| `amovel(pose6, vel, acc)` / `motion_state()` | 비동기 **절대** 직선 이동(`vel`·`acc`는 숫자 또는 `[직선, 회전]`) / 이동 상태 0 정지·1 가감속·2 이동 중 — 이동하면서 힘을 읽는 탐지 스캔용 (11/17) |
| `movejx(pose6, vel, acc, tol=2.0)` | TCP 목표로 관절 보간 이동(지금 해 공간 유지) + 도달 확인. 직선이 아니므로 높은 곳에서만 (11/17) |
| `compliance_on(stx)` / `force_on(fd, dir, time)` | 순응제어 / 목표 힘 (상대 모드) |
| `stop(emergency=False)` | 힘·순응제어 해제 (+ `emergency`면 `move_stop`) 후 멈춤 확인 |
| `ok()` | ROS가 살아 있는지 (긴 루프 조건) |

- 로봇은 `self.robot`으로만 움직입니다. `DSR_ROBOT2`를 직접 부르면 정지 확인이 빠집니다. **필요한 명령이 없으면 관제에게 요청**합니다(예: 회전 이동).
- 두산 패키지가 없는 PC에서는 `robot`이 `None`으로 들어옵니다. 템플릿의 FAKE 동작은 로봇을 쓰지 않으니 그대로 돕니다. 서비스 준비 확인도 첫 명령 때 하므로 bringup 없이 FAKE를 돌려도 기다리지 않습니다.
- 약속(작업 함수 인자·반환, `robot` 명령 목록)을 바꿀 때만 관제와 합의합니다.
- 적재가 완성 예시입니다: `box_place/box_place/place_algorithm.py` (힘 제어 하강 `_descend`, 상태 파일, 미분류 투하).

### 2-3-2. 집기 실패 처리 (11/17, 세 파트 공통)

못 집었을 때 **파트는 자기 방식으로 다시 집고**, 그래도 안 되면 `GRASP_FAILED`로 Goal을 끝냅니다(그리퍼 열고 안전 높이로). **그다음은 전부 관제가 처리합니다.** 파트 algorithm은 실패까지만 책임집니다.

| 단계 | 누가 | 하는 일 |
| --- | --- | --- |
| ① 다시 집기 | 파트 | `res = self.robot.grip_close()` → `res.grabbed`가 False면 다시 시도. 시도마다 `report.log('집기 실패, 재시도 2/3', 'WARN')` (UI 로그). 최대 `common.GRIP_RETRY`(3)번. 그래도 안 되면 그리퍼를 열고 `raise PartError('GRASP_FAILED', '3회 실패')` |
| ② 측정존 비우기 | **관제** (`box_control/recovery.py`, 판별·적재만) | 파트 Goal이 끝난 뒤 관제가 직접 로봇을 움직여 측정존 박스를 스캔 공간으로 밀어내고(`common.RECOVERY push_path`), 측정존을 그리퍼로 잡아 봐서 비었는지 확인. 못 비우면 `ERROR : Measure zone not cleared`. **11/18: 경로 미정이라 fake**(WARN 로그만 남기고 성공 처리 → 스캔하는 동안 사람이 치움) |
| ③ 흐름 되돌리기 | 관제 | `GRASP_FAILED`면 작업을 끝내지 않고 **공간 스캔부터 다시** (재고·박스 번호 그대로, UI 로그 `BOX_RETRY`). 연속 `RECOVER_LIMIT`(3)번을 넘거나 `RECOVERY_FAILED`면 작업 종료(`ERROR`) |

파트별 다시 집기 (11/17 정함)

| 파트 | 다시 집는 방법 | 다 실패하면 |
| --- | --- | --- |
| 탐지 | 높이를 찾은 첫 지점에서 그리퍼 0° → 45° → 90°(원본 실로봇 시험값, `scan_config.GRIP_RETRY_ANGLES`) 돌려서 3번 | `GRASP_FAILED` → 관제가 공간 스캔부터 다시 (박스가 스캔 공간에 그대로라 밀어내기 없음, 탐지는 그 접촉점 −50mm부터 이어서) |
| 판별 | 같은 자리·같은 자세로 다시 열고 내려가 sync 닫기 3번 (가로·세로, 11/18 구현 `_close_retry`). 가로 하강에서 핑거가 얹히면 다시 해도 같으므로 재시도 없이 바로 | `GRASP_FAILED` → 측정존 비우기(지금 fake) → 공간 스캔부터 다시 |
| 적재 | 같은 자리·같은 자세로 sync 집기 3번 (11/18) | `GRASP_FAILED` → 측정존 비우기(지금 fake) → 공간 스캔부터 다시 |

```python
from box_comm.common import GRIP_RETRY

from box_comm.gripper import check_grip, holding

for attempt in range(1, GRIP_RETRY + 1):
    ...  # 파트 방식대로 접근 (탐지는 시도마다 그리퍼 각도를 바꿈)
    res = self.robot.grip_close()
    ok, why = check_grip(res.width_mm, 1) if res.grabbed else (False, '빈손')   # 폭 범위까지 확인 (11/18)
    if ok:
        ...  # 들어 올린 뒤 holding(self.robot.gripper, res.width_mm)로 한 번 더 (폭이 변했으면 놓친 것)
        break
    report.log(f'집기 실패 {attempt}/{GRIP_RETRY}: {why}', 'WARN')
    self.robot.grip_open()
else:
    raise PartError('GRASP_FAILED', f'{GRIP_RETRY}회 시도 실패')
```

- `GRASP_FAILED`는 **"다시 집어 보고 포기했다"는 뜻으로만** 씁니다. 예상 못 한 오류에 쓰면 관제가 재스캔을 반복합니다.
- 측정존 밀어내기 경로(`push_path`)가 아직 미정이라, 지금은 판별·적재에서 못 집으면 **비우기를 fake로 넘기고 바로 재스캔**합니다(사전 점검 WARN, 이벤트 로그 `ZONE_CLEAR` WARN). 스캔하는 동안 사람이 측정존 박스를 치워 주세요. 실물로 경로를 정하면 `common.RECOVERY`에 넣습니다.
- 그리퍼는 세 파트 모두 **Modbus**(`box_comm/gripper.py`)입니다. Compute Box는 동시 접속이 1개라 `robot`이 Goal 동안만 접속을 열고 닫습니다. OnRobot 드라이버(`m0609_rg2_bringup mode:=real`)가 떠 있으면 접속이 안 되니 `pkill -f OnRobotRGControllerServer`로 끕니다. 에뮬레이터·fake 시험은 `BOX_GRIPPER=fake`(가짜 그리퍼, `fake_run.sh`가 설정).

### 2-4. 그 밖의 규칙

- 로봇 동작과 **홈 복귀까지 끝난 뒤에** 반환합니다.
- 좌표는 로봇 base 기준, 단위는 mm입니다.
- **단계가 바뀔 때마다 `report(stage)`를 반드시 호출합니다.** 관제가 이 값으로 진행 상황을 추적하고, 멈췄을 때 어느 단계였는지 기록합니다. (위 2-2-1)
- DB의 같은 `box_id` 중복 무시, `status` 값 검사는 `box_comm`이 이미 처리합니다.

### 2-5. 두산 로봇 연동

**TCP·툴은 신경 쓰지 않아도 됩니다.** Goal을 받으면 작업 함수 직전에 `box_comm`이 기본 TCP(`GripperDA_v1`)와 툴(`Tool Weight`)을 확인하고, 다르면 적용합니다. 관제 없이 혼자 `send_goal`로 테스트할 때도 똑같이 적용됩니다.

- 실제 로봇·시연에서는 `--ros-args -p robot_setup:=on`으로 실행합니다. 적용에 실패하면 로봇을 움직이지 않고 `ROBOT_SETUP_FAILED`로 끝납니다.
- 자기 동작 안에서 다른 TCP가 필요하면 바꿔도 됩니다. (예: 무게 측정용 기준점) 다음 Goal에서 기본값으로 돌아갑니다.
  단, **로봇에 등록된 이름**이어야 합니다. 등록되지 않은 이름은 조용히 실패해서 플랜지 기준으로 움직입니다. 새 TCP가 필요하면 관제에게 요청하세요.

**공통 좌표(HOME, 측정존, 적재 구역)는 작업 함수의 `poses` 인자로 받습니다.** 직접 숫자를 적지 않습니다.
관제가 DB 값을 Goal에 실어 보내면, `box_comm`이 Goal마다 작업 함수에 넘겨 줍니다.

```python
def scan(self, poses, report):
    home = poses['home_j']               # HOME 관절 각도 [deg]
    zone = poses['measure_zone']         # 측정존 x, y, z [mm] (None 또는 (0,0,0)이면 미설정 → 움직이지 말 것)
    # (적재) poses['place_zones']['B']   # B 구역 1단 x, y [mm] (None이면 미설정)
    self.robot.movej(home, vel=30, acc=30)
```

- 혼자 `send_goal`로 테스트하면 Goal에 좌표가 없어서 `common.py` 기본값이 들어옵니다(경고 로그).
- 템플릿의 `_measure_zone()`이 미설정 여부까지 확인해 줍니다.
- 좌표를 `__init__`에서 변수에 저장해 두지 마세요. Goal마다 DB 값이 바뀔 수 있으므로 작업 함수 안에서 매번 `poses`로 읽습니다.

- 두산 로봇 연결(`connect_dsr()`, 두산 전용 노드 분리, executor)은 **`box_comm`이 대신 합니다.** 알고리즘 방식에서는 신경 쓰지 않아도 됩니다.
- 예전 방식(`run()` 안에서 `self.dr.movel` 직접 호출)을 쓸 때만 아래가 해당됩니다.
  ```python
  from box_comm.robot import connect_dsr

  class ScanWork:
      def __init__(self, node):
          self.dr, self.dsr_node = connect_dsr('scan_dsr')   # 판별 'judge_dsr', 적재 'place_dsr'
  ```
  ⚠ 클래스 안에서 `DR_init.__dsr__node = ...`를 직접 쓰지 마세요. 파이썬이 `__`로 시작하는 이름을 클래스 이름이 붙은 다른 이름으로 바꿔 버려서, 두산 쪽이 노드를 찾지 못합니다(`'NoneType' object has no attribute 'create_client'`).
- `box_comm`이 만든 노드(`self.node`)와 **로봇 제어용 노드를 분리**합니다. (`connect_dsr()`가 해 줍니다)
  `DSR_ROBOT2` 함수는 등록된 노드를 내부에서 spin하므로, 같은 노드를 쓰면 충돌할 수 있습니다.
- 노드 이름은 파트별로 다르게 합니다. (`scan_dsr`, `judge_dsr`, `place_dsr`)
- **실제로 움직이는 완성 예시**: 알고리즘 방식은 `box_place/box_place/place_algorithm.py`(실제 로봇에서 적재 검증). 예전 방식 예시는 `box_control/box_control/test_scan_motion.py`입니다.

### 2-5-1. 측정존 인수인계 약속 (탐지 → 판별 → 적재, 11/17)

세 파트는 코드가 아니라 **측정존 위의 박스 상태**로 맞물립니다. 앞 파트가 이 약속을 지키면 뒤 파트는 그대로 믿고 동작합니다.

**측정존 좌표 (11/18 통일)**: DB `measure_zone` = 측정존 중심 x, y, **바닥 z** — 세 파트가 공유하는 건 이 사실 하나뿐입니다. 집는·놓는 높이는 각 파트가 이 바닥에서 계산하고 자기 설정에 둡니다.

| 파트 | 측정존에서 높이를 정하는 법 | 설정 |
| --- | --- | --- |
| 탐지 | 놓을 때: 예상 접촉 높이 위에서 천천히 내려가 닿으면 놓음 (하한 바닥 + 2mm). 집기(작업공간)는 윗면 − 15mm | `scan_config` `GRIP_DEPTH_FROM_TOP_MM`, `PLACE_*` |
| 판별 | 바닥 z를 테이블 기준으로 높이·폭 측정. 길이 측정 때문에 '들어 올리면서 집기'는 판별만 씀 | `judge_algorithm` `PARAMS` (`table_ref_file`로 실측값 대체 가능) |
| 적재 | sync 집기 (11/18, 판별과 같은 `common.SYNC_GRIP`): 100mm로 열고 핑거 끝을 바닥 + 2.6mm에 둔 채 닫으면서 TCP를 올림 → 폭과 상관없이 박스 아래쪽을 잡음. 층 판단은 TCP Z − (집은 TCP − 바닥) | `common.SYNC_GRIP` (판별·적재 공용) |
| 관제 (측정존 확인) | 바닥 + 5mm에서 닫아 봄 | `common.RECOVERY` `probe_above_floor_mm` |

**집기 기준 (11/18 정리)** — 공통은 `box_comm`(관제 관리)에서 받아 쓰고, 나머지는 파트 몫입니다.

| 과정 | 공통 (`box_comm`) | 탐지 | 판별 | 적재 |
| --- | --- | --- | --- | --- |
| 여는 폭 | `GRIPPER open_width_mm` 100 | 공통 | 공통 | 공통 |
| 내려가기 | — | 힘 제어 하강, 8N 닿으면 '걸림' → 다음 각도 | 높이: 닫고 위치 하강(정상 접촉) / 가로: 열고 위치 하강, 6N 얹힘 → `GRASP_FAILED` / 세로: 힘 제어 하강 + 회피 | 위치 하강만 (판별이 정렬해 줌, 얹힘 감시 없음) |
| 집는 높이 | `SYNC_GRIP` 핑거 끝 바닥 + 2.6 | 윗면 − 15mm (고정) | sync | sync |
| 닫는 힘 | `GRIPPER grip_force_n` 40 / `SYNC_GRIP close_force_n` 10 | 40 | 10 (폭 측정) | 10 → 40 조임 |
| 잡힘 판정 | `gripper.check_grip`/`holding` | 비트 + 범위 + 폭 변화 | 비트 + 빈손 + 폭 변화 (범위 안 봄 → U) | 비트 + 범위 + 폭 변화 |
| 다시 집기 | `GRIP_RETRY` 3 | 각도 0/45/90° | 같은 자리 3번 | 같은 자리 3번 |
| 최종 실패 | `PartError('GRASP_FAILED')` | 관제가 바로 재스캔 | 관제: 측정존 비우기(지금 fake) → 재스캔 | 같음 |
| 내려놓기 | — | 닿을 때까지 내려놓기 | 정렬해서 제자리에 | 힘 제어로 층 접촉 → 재고 |

내려가기 접촉 기준(탐지 8N, 판별 6N)은 내려가는 방식(힘 제어 / 위치 하강)이 달라 같은 숫자로 맞추지 않습니다.

| 단계 | 누가 | 측정존에 남기는 상태 |
| --- | --- | --- |
| 탐지 → 판별 | 탐지 | 박스의 **일부가 측정존 중심에 걸치도록** 놓는다. 방향은 정해진 그리퍼 방향. 정확한 정렬은 하지 않는다 (11/17 구현: 집은 TCP를 측정존 x·y에 맞춰 바닥에 닿을 때까지 내려놓음. 집을 때 돌린 각도 0/45/90°가 그대로 남음) |
| 판별 → 적재 | 판별 | 측정·정렬을 끝낸 뒤 **측정존 중심 x·y**(DB `measure_zone`)에 놓는다. **가장 긴 면 = 가로, 두 번째로 긴 면 = 세로**, 가장 짧은 면 = 높이(바닥에 눕힘). 돌리기·정렬·가운데 맞추기는 판별이 한다 |
| 적재 | 적재 | 위 상태를 **확정된 것으로 보고** 매번 같은 자세(`place_config.json` `pick_orientation`)로 집는다. 방향 값은 DB·관제로 전달하지 않는다 (관제는 A~D·U만 넘김) |

- **가로 = 로봇 base X축.** 가장 긴 면이 X, 두 번째 면이 Y, 가장 짧은 면이 Z(높이)
  → 적재 그리퍼는 base **Y** 방향으로 닫혀 두 번째 면(40 / 29mm)을 잡는다

물품 크기 (길이1 ≥ 길이2 ≥ 길이3 = 정렬 후 X / Y / Z)

| 분류 | 물품 | 길이1 (X) | 길이2 (Y) | 길이3 (Z, 높이) |
| --- | --- | --- | --- | --- |
| A | 알루미늄 묶음 | 60 | 40 | 20 |
| B | 젠가 묶음 | 75 | 29 | 25 |
| C | 알루미늄 묶음 | 50 | 40 | 20 |
| D | 지우개 | A 또는 C와 같음 → 크기로 구분 안 됨, **물성으로 분류** | | 20 |

- **옆으로 선 박스**(가장 짧은 변이 높이가 아님, 특히 B: 길이2 29 / 높이 25): 판별이 **Rx·Ry 90° 회전으로 눕혀서** 위 규칙을 맞춘다. 수직축 회전만으로는 바닥에 닿은 면이 안 바뀌므로 필요
- D는 높이·두 번째 면이 A·C와 같아서 적재는 높이 20으로 설정하면 된다 (구역 좌표·첫 접촉 Z만 실측)

확인 필요 (판별·적재 담당이 채움)
- 적재 `pick_orientation`에서 그리퍼가 실제로 base Y 방향으로 닫히는지 (설정의 자세는 모두 a = c라 같은 방향 — 실물로 한 번 확인)
- 탐지 결과가 정렬 가능한 범위를 벗어났을 때 판별의 처리: ____

### 2-6. 단독 테스트

관제 없이 내 노드만 띄워서 직접 Goal/Request를 보냅니다. Result가 돌아오면 통신은 정상입니다.
아래 명령은 Goal에 `poses`를 넣지 않으므로 `common.py` 기본 좌표가 쓰입니다. 특정 좌표로 시험하려면 Goal에 `poses: {available: true, home_j: [...], measure_zone: [x, y, z]}`를 추가합니다.

```bash
# 탐지
ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback

# 판별
ros2 action send_goal /judge box_interfaces/action/Judge \
  "{specs: [{class_id: 'A', width_mm: 40.0, depth_mm: 40.0, height_mm: 40.0, weight_g: 100.0},
            {class_id: 'B', width_mm: 60.0, depth_mm: 40.0, height_mm: 30.0, weight_g: 150.0}]}" --feedback

# 적재 (v0.4: 구역 좌표가 필요하므로 poses를 넣어서 보냄)
ros2 action send_goal /place box_interfaces/action/Place \
  "{result_class: 'B', poses: {available: true, home_j: [0,0,90,0,90,0], measure_zone: [400,0,100],
    place_zones: [{class_id: 'B', x: 300.0, y: -200.0}]}}" --feedback

# DB
ros2 service call /get_job_data box_interfaces/srv/GetJobData "{}"
ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}"
ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'ERROR', reason: 'Fail to pick'}"
ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \
  "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0}}"
```

관제와 연결해 보려면 내 노드만 실제로, 나머지는 fake로 띄웁니다.
예: `box_scan scan_main`(내 노드) + `fake_judge` + `fake_place` + `fake_database` + `control`

---

## 3. Git 사용 규칙

모두 `main` 하나에서 작업합니다. 각자 고치는 폴더가 나뉘어 있어서 충돌이 거의 생기지 않습니다.

### 3-1. 수정 권한

| 누가 | 고치는 폴더 |
| --- | --- |
| 관제 | `box_interfaces/`, `box_comm/`, `box_control/`, `README.md`, `docs/` |
| 탐지 | `box_scan/` |
| 판별 | `box_judge/` |
| 적재 | `box_place/` |
| DB·UI | `box_database/` |

- **다른 사람의 폴더와 공용 파일은 건드리지 않습니다.**
- 예외: 각 파트 폴더 안의 `*_template.py`는 **관제가 관리**합니다. 파트 담당은 템플릿을 고치지 않고, 관제는 파트 담당의 메인 코드를 고치지 않습니다.
- `box_interfaces/`, `box_comm/` 변경이 필요하면 관제에게 요청합니다.

**관제 전용 파일 보호** — `box_comm/`, `box_interfaces/`, 각 폴더의 `*_template.py`

| 장치 | 동작 |
| --- | --- |
| commit 훅 (`.githooks/pre-commit`) | 이 파일을 바꾼 채 commit하면 **commit이 거부**되고 되돌리는 명령이 안내됩니다. 1-2의 `git config core.hooksPath .githooks`로 켭니다 |
| GitHub Actions (`.github/workflows/protect-paths.yml`) | 관제가 아닌 사람의 push에 이 파일 변경이 있으면 GitHub에 **빨간 X**가 뜨고 관제에게 알림이 갑니다 |

- GitHub 무료 public 저장소에서는 경로별로 push를 완전히 막을 수 없어서, 위 두 장치로 실수를 막습니다. `--no-verify`로 훅을 피하지 마세요.

### 3-2. 매일 작업 루틴

Git 명령은 항상 `box_project` 폴더 안에서 합니다. (빌드만 `~/ws`에서)

```bash
cd ~/ws/src/box_project

git pull                                  # ① 작업 시작 전: 최신 받기

# ... 자기 폴더에서 작업 ...

git add box_scan/                         # ② 자기 폴더만 담기 (git add . 금지)
git commit -m "scan: 외력 감지 로직 추가"   # ③ 무엇을 했는지 적어서 저장
git pull                                  # ④ 올리기 직전에 한 번 더 받기
git push                                  # ⑤ 업로드
```

### 3-3. commit 메시지

`머리말: 무엇을 했는지` 형식으로 한 줄 씁니다. 날짜는 Git이 자동으로 기록합니다.

| 머리말 | 예시 |
| --- | --- |
| `scan:` | `scan: 외력 감지로 물품 탐지 구현` |
| `judge:` | `judge: 그리퍼 개구폭으로 W 측정` |
| `place:` | `place: 클래스별 슬롯 좌표 추가` |
| `db:` | `db: SQLite 저장 연동` |
| `control:` | `control: 에러 시 재시도 1회 추가` |
| `comm:` | `comm: 비상정지 서비스 추가` (전체 공지 필요) |
| `docs:` | `docs: 진행 현황 업데이트` |

### 3-4. 관제가 공용 패키지를 고쳤다고 알렸을 때

```bash
cd ~/ws/src/box_project && git pull
cd ~/ws
colcon build --symlink-install --packages-up-to box_control box_scan box_judge box_place box_database
source install/setup.bash
```

- `box_comm`의 파이썬 내용만 바뀐 경우, `git pull`만 해도 반영됩니다.
- `box_interfaces`가 바뀌었거나 파일이 추가된 경우에는 빌드가 필요합니다.
- 작업 함수(`scan`·`judge`·`place`)의 입력/출력 형식이 바뀌는 변경은 관제가 미리 알리며, 이때는 해당 파트도 코드를 수정합니다.

**템플릿이 바뀌었을 때** (공지에 "템플릿 업데이트"가 있으면)

```bash
cd ~/ws/src/box_project
git log --oneline -5 -- box_scan/box_scan/scan_template.py        # 템플릿이 바뀐 commit 목록
git log -p -1 -- box_scan/box_scan/scan_template.py               # 가장 최근에 바뀐 내용 (+ 추가, - 삭제)
git diff --no-index box_scan/box_scan/scan_algorithm.py box_scan/box_scan/scan_template.py   # 내 코드와 템플릿 비교
```

바뀐 부분을 보고 내 메인 코드에 직접 옮깁니다. 템플릿을 내 코드에 통째로 덮어쓰지 마세요.

### 3-5. pull할 때 알아 둘 것

- `pull`은 **다른 사람이 올린 변경만** 받아 옵니다. 내가 작업 중인 파일은 그대로 남습니다.
- 내가 commit하지 않은 파일을 누가 고쳐서 올렸다면, Git이 덮어쓰지 않으려고 pull을 멈춥니다.
  먼저 `git add` → `git commit` 한 뒤 다시 `git pull` 합니다.

### 3-6. 자주 만나는 상황

| 화면 | 할 일 |
| --- | --- |
| `! [rejected] ... (fetch first)` | 다른 사람이 먼저 올림 → `git pull` 후 다시 `git push` |
| pull 후 편집기 화면이 열림 | 자동 합치기 메모 → vim이면 `:wq` Enter, nano면 `Ctrl+X` |
| `Your local changes ... would be overwritten` | 내 작업 먼저 commit → 다시 `git pull` |
| `CONFLICT` | **손대지 말고** `git merge --abort` 후 관제에게 연락 |
| `Authentication failed` | `gh auth login` 다시 하기 |
| `✋ commit 거부: 아래 파일은 관제 전용입니다` | 안내된 `git restore --staged --worktree <파일>`로 **그 파일만** 되돌린 뒤 다시 `git commit` (내 다른 작업은 그대로 남음) |
| 내 상태 확인 | `git status` |
