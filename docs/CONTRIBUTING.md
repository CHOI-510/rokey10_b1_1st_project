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
ros2 run box_control control          # 마지막에 실행
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
- 메인 코드는 템플릿을 복사해서 만듭니다. 파일 이름은 자유이며, 아래는 `scan.py`로 만드는 예입니다.

```bash
cd ~/ws/src/box_project/box_scan
cp box_scan/scan_template.py box_scan/scan.py
# setup.py의 console_scripts에 한 줄 추가:
#     'scan = box_scan.scan:main',
cd ~/ws && colcon build --symlink-install --packages-select box_scan && source install/setup.bash
ros2 run box_scan scan
```

- 메인 코드의 `[FAKE]` 부분을 실제 동작으로 바꿔 갑니다.
- **템플릿 노드와 내 노드를 동시에 켜지 마세요.** 서버 이름(`/scan` 등)이 같아서 관제가 엉뚱한 결과를 받습니다.
- 모션·측정 함수가 길어지면 **같은 패키지 안에 파일을 추가**해서 import해도 됩니다.
  예: `box_scan/box_scan/motion.py` → `scan.py`에서 `from box_scan.motion import ...`

### 2-2. 지켜야 할 약속: `run()`의 입력과 출력

**탐지 — `scan.py`**

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

**판별 — `judge.py`**

```python
def run(self, specs, report) -> str:
    # specs = [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
    #           'height_mm': 40.0, 'weight_g': 100.0}, ...]
    # 측정 후 'A' / 'B' / 'C' / 'D' 중 하나를 반환
```

**적재 — `place.py`**

```python
def run(self, result_class, report) -> int:       # v0.4
    # result_class = 'B'
    # 매번 1단부터 박스를 쥔 채 내려가며 놓을 자리를 찾고(5층 높이에서 닿으면 다음 단),
    # 놓은 위치로 잰 수량을 반환: count_from_position(단, 층)  예: 2단 2층 → 6
    # 단·층·z는 관제에 보내지 않음 (자기 로그로만 남김)
    # 구역 1단 x, y는 self.poses['place_zones']['B'] (DB 값)
    # 단 간격·z·박스 높이는 이 파일 안에서만 다룸
```

**DB — `database.py`**

```python
def get_job_data(self) -> tuple:           # (specs, inventory) 반환
def update_inventory(self, box_id, inventory) -> None:
def set_job_status(self, status) -> None:  # 'BEFORE' / 'RUNNING' / 'DONE'
def get_fixed_poses(self) -> dict:         # DB에 저장된 고정 좌표 (기준값), 없으면 None
    # {'home_j': [6개, deg], 'measure_zone': [x, y, z, mm],
    #  'place_zones': {'A': [x, y], 'B': [x, y], 'C': [x, y], 'D': [x, y]}}   ← v0.4 적재 구역 1단
    # 관제가 이 값을 모든 파트 Goal에 실어 보낸다. 없거나 구역이 빠지면 관제가 작업을 시작하지 않음
# UI 적재 이미지: box_comm.common.stack_layout(수량) → 단별 층 수 (예: 6 → [4, 2])
```

### 2-2-1. Feedback stage (필수)

`run()` 안에서 단계가 바뀔 때마다 `report('stage')`를 호출합니다. 값은 아래 목록에서만 씁니다. (`box_comm/common.py`의 `STAGES`)
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

### 2-3. 실패 처리

실패하면 `PartError`를 raise합니다. `box_comm`이 Result(`success=False`, `error_code`)로 바꿔 관제에 보냅니다.

```python
raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
```

| 코드 | 뜻 |
| --- | --- |
| `NOT_FOUND` | 물품을 찾지 못함 |
| `GRASP_FAILED` | 집기 실패 |
| `OVER_FORCE` | 과도한 힘 감지 |
| `MEASURE_FAILED` | 측정 실패 |
| `NO_SLOT` | 적재할 칸 없음 |
| `PLACE_FAILED` | 적재 실패 |
| `ROBOT_SETUP_FAILED` | 기본 TCP·툴 적용 실패 (`box_comm`이 `run()` 전에 보냄, 파트 코드에서는 쓰지 않음) |

- **등록된 코드만** 쓸 수 있습니다. 새 코드가 필요하면 관제에게 요청합니다.
- 예상하지 못한 예외가 나도 `box_comm`이 잡아서 Result를 보냅니다. 관제가 멈추지 않습니다.
- DB는 `PartError` 없이 아무 예외나 raise하면 `success=False`로 전달됩니다.

### 2-4. 그 밖의 규칙

- 로봇 동작과 **홈 복귀까지 끝난 뒤에** 반환합니다.
- 좌표는 로봇 base 기준, 단위는 mm입니다.
- **단계가 바뀔 때마다 `report(stage)`를 반드시 호출합니다.** 관제가 이 값으로 진행 상황을 추적하고, 멈췄을 때 어느 단계였는지 기록합니다. (위 2-2-1)
- DB의 같은 `box_id` 중복 무시, `status` 값 검사는 `box_comm`이 이미 처리합니다.

### 2-5. 두산 로봇 연동

**TCP·툴은 신경 쓰지 않아도 됩니다.** Goal을 받으면 `run()` 직전에 `box_comm`이 기본 TCP(`GripperDA_v1`)와 툴(`Tool Weight`)을 확인하고, 다르면 적용합니다. 관제 없이 혼자 `send_goal`로 테스트할 때도 똑같이 적용됩니다.

- 실제 로봇·시연에서는 `--ros-args -p robot_setup:=on`으로 실행합니다. 적용에 실패하면 로봇을 움직이지 않고 `ROBOT_SETUP_FAILED`로 끝납니다.
- 자기 동작 안에서 다른 TCP가 필요하면 바꿔도 됩니다. (예: 무게 측정용 기준점) 다음 Goal에서 기본값으로 돌아갑니다.
  단, **로봇에 등록된 이름**이어야 합니다. 등록되지 않은 이름은 조용히 실패해서 플랜지 기준으로 움직입니다. 새 TCP가 필요하면 관제에게 요청하세요.

**공통 좌표(HOME, 측정존, 적재 구역)는 `self.poses`로 읽습니다.** 직접 숫자를 적지 않습니다.
관제가 DB 값을 Goal에 실어 보내면, `box_comm`이 Goal을 받을 때마다 작업 클래스의 `self.poses`에 넣어 줍니다.

```python
def run(self, report):
    home = self.poses['home_j']          # HOME 관절 각도 [deg]
    zone = self.poses['measure_zone']    # 측정존 x, y, z [mm] (None 또는 (0,0,0)이면 미설정 → 움직이지 말 것)
    xy = self.poses['place_zones']['B']  # (적재) B 구역 1단 x, y [mm] (None이면 미설정)
    self.dr.movej(home)
```

- 혼자 `send_goal`로 테스트하면 Goal에 좌표가 없어서 `common.py` 기본값이 들어옵니다(경고 로그).
- 템플릿의 `_measure_zone_xyz()`가 미설정 여부까지 확인해 줍니다.
- 좌표를 `__init__`에서 변수에 저장해 두지 마세요. Goal마다 DB 값으로 덮어써지므로 `run()` 안에서 매번 `self.poses`로 읽습니다.

- `run()` 안에서 `DSR_ROBOT2` 함수(`movej`, `movel` 등)를 **그대로 호출해도 됩니다.** `box_comm`이 파트 노드를 전용 executor로 돌리기 때문에, 두산 함수가 내부에서 응답을 기다려도 충돌(`Executor is already spinning`)하지 않습니다.
- 두산 로봇 연결은 **`box_comm.robot.connect_dsr()`로** 합니다. 두산 전용 노드를 따로 만들어 연결해 줍니다.
  ```python
  from box_comm.robot import connect_dsr

  class ScanWork:
      def __init__(self, node):
          self.dr, self.dsr_node = connect_dsr('scan_dsr')   # 판별 'judge_dsr', 적재 'place_dsr'

      def run(self, report):
          self.dr.movej(HOME_J)
          self.dr.movel([0, 0, 50, 0, 0, 0], mod=1)
  ```
  ⚠ 클래스 안에서 `DR_init.__dsr__node = ...`를 직접 쓰지 마세요. 파이썬이 `__`로 시작하는 이름을 클래스 이름이 붙은 다른 이름으로 바꿔 버려서, 두산 쪽이 노드를 찾지 못합니다(`'NoneType' object has no attribute 'create_client'`).
- `box_comm`이 만든 노드(`self.node`)와 **로봇 제어용 노드를 분리**합니다. (`connect_dsr()`가 해 줍니다)
  `DSR_ROBOT2` 함수는 등록된 노드를 내부에서 spin하므로, 같은 노드를 쓰면 충돌할 수 있습니다.
- 노드 이름은 파트별로 다르게 합니다. (`scan_dsr`, `judge_dsr`, `place_dsr`)
- **실제로 움직이는 완성 예시**: `box_control/box_control/test_scan_motion.py` — 가상·실제 로봇에서 검증된 코드입니다. 이 파일의 구조(`connect_dsr` → `run()` 안에서 `movej`/`movel` → `report()`)를 그대로 따라 하면 됩니다.

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
ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \
  "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0}}"
```

관제와 연결해 보려면 내 노드만 실제로, 나머지는 fake로 띄웁니다.
예: `box_scan scan`(내 노드) + `fake_judge` + `fake_place` + `fake_database` + `control`

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
- `run()`의 입력/출력 형식이 바뀌는 변경은 관제가 미리 알리며, 이때는 해당 파트도 코드를 수정합니다.

**템플릿이 바뀌었을 때** (공지에 "템플릿 업데이트"가 있으면)

```bash
cd ~/ws/src/box_project
git log --oneline -5 -- box_scan/box_scan/scan_template.py        # 템플릿이 바뀐 commit 목록
git log -p -1 -- box_scan/box_scan/scan_template.py               # 가장 최근에 바뀐 내용 (+ 추가, - 삭제)
git diff --no-index box_scan/box_scan/scan.py box_scan/box_scan/scan_template.py   # 내 코드와 템플릿 비교
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
