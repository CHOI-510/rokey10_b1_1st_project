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

cd ~/ws
colcon build --symlink-install --packages-up-to box_control box_scan box_judge box_place box_database
source install/setup.bash
ros2 pkg list | grep box_       # 7개가 나오면 성공
```

- 폴더 이름은 모두 `box_project`로 맞춥니다. (`clone` 뒤에 `box_project`를 꼭 붙이기)
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

관제 터미널에 `===== 상태: 작업완료 =====`가 나오면 환경 세팅이 끝난 것입니다.

---

## 2. 내 파트 구현하기

### 2-1. 고치는 파일은 하나

| 담당 | 고치는 파일 |
| --- | --- |
| 탐지 | `box_scan/box_scan/scan.py` |
| 판별 | `box_judge/box_judge/judge.py` |
| 적재 | `box_place/box_place/place.py` |
| DB·UI | `box_database/box_database/database.py` |

- 통신(Action/Service 서버, Result 보내기)은 `box_comm`이 처리합니다. **신경 쓰지 않아도 됩니다.**
- 지금 파일은 fake로 동작합니다. `[FAKE]` 표시된 부분을 실제 동작으로 바꿔 갑니다.
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
def run(self, result_class, inventory, report) -> dict:
    # result_class = 'B'
    # inventory    = {'A': 1, 'B': 0, 'C': 2, 'D': 0}   (적재 전)
    # 적재 후 inventory를 같은 형식으로 반환
    # 슬롯 배정·좌표는 이 파일 안에서만 다룸
```

**DB — `database.py`**

```python
def get_job_data(self) -> tuple:           # (specs, inventory) 반환
def update_inventory(self, box_id, inventory) -> None:
def set_job_status(self, status) -> None:  # 'BEFORE' / 'RUNNING' / 'DONE'
```

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

- **등록된 코드만** 쓸 수 있습니다. 새 코드가 필요하면 관제에게 요청합니다.
- 예상하지 못한 예외가 나도 `box_comm`이 잡아서 Result를 보냅니다. 관제가 멈추지 않습니다.
- DB는 `PartError` 없이 아무 예외나 raise하면 `success=False`로 전달됩니다.

### 2-4. 그 밖의 규칙

- 로봇 동작과 **홈 복귀까지 끝난 뒤에** 반환합니다.
- 좌표는 로봇 base 기준, 단위는 mm입니다.
- `report('단계명')`은 진행 알림입니다. 쓰지 않아도 됩니다.
- DB의 같은 `box_id` 중복 무시, `status` 값 검사는 `box_comm`이 이미 처리합니다.

### 2-5. 두산 로봇 연동

- `box_comm`이 만든 노드(`self.node`)와 **로봇 제어용 노드를 분리**합니다.
  `DSR_ROBOT2` 함수는 등록된 노드를 내부에서 spin하므로, 같은 노드를 쓰면 충돌할 수 있습니다.
- 예시는 `scan.py`의 `DR_init` 주석을 참고합니다. 노드 이름은 파트별로 다르게 합니다. (`scan_dsr`, `judge_dsr`, `place_dsr`)

### 2-6. 단독 테스트

관제 없이 내 노드만 띄워서 직접 Goal/Request를 보냅니다. Result가 돌아오면 통신은 정상입니다.

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

관제와 연결해 보려면 내 노드만 실제로, 나머지는 fake로 띄웁니다.
예: `box_scan scan` + `fake_judge` + `fake_place` + `fake_database` + `control`

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
- `box_interfaces/`, `box_comm/` 변경이 필요하면 관제에게 요청합니다.

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
| 내 상태 확인 | `git status` |
