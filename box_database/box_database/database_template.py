#!/usr/bin/env python3
"""
database_template.py — DB·UI 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(이승재)은 이 파일을 참고해 자기 메인 코드(예: database.py)를 만든다.
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_database/box_database/database_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_database database_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
아래는 메인 코드가 지킬 약속과 예시다. 통신은 database_interface.py가 처리한다.

지킬 약속은 네 개 (+ 선택 하나):
  get_job_data() -> (specs, inventory)
      specs     = [{'class_id': 'A', 'width_mm': .., 'depth_mm': ..,
                    'height_mm': .., 'weight_g': ..}, ...]
      inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
  update_inventory(box_id, inventory) -> None
  set_job_status(status, reason) -> None status: 'BEFORE' / 'RUNNING' / 'DONE' / 'ERROR' / 'STOPPED'
      reason: ERROR·STOPPED일 때 이유 ('Fail to pick', 'Zone full (B)', 'EMERGENCY STOP'), 그 밖에는 ''
      UI 표시: f'{status} : {reason}'   — ERROR = 공정 오류(로그 확인), STOPPED = 로봇 정지(현장 확인)
  get_fixed_poses() -> dict               DB에 저장된 고정 좌표 — 기준값 (필수)
      {'home_j': [6개, deg], 'measure_zone': [x, y, z, mm],
       'place_zones': {'A': [x, y], 'B': [x, y], 'C': [x, y], 'D': [x, y]}}     ← v0.4
      DB가 기준이다 (v0.3). 관제는 이 값으로 자기 좌표를 덮어쓰고 모든 파트 Goal에 실어 보낸다.
      place_zones = 클래스별 적재 구역 '1단' x, y [mm, base]. 단 간격·z는 적재만 가진다.
      None을 반환하거나 구역이 하나라도 빠지면 관제가 작업을 시작하지 않는다 (사전 점검 FAIL).

  on_event(event) -> None                (선택) 이벤트 로그 한 줄 — /event_log 토픽 (관제가 보냄)
      event = {'stamp', 'job_id', 'source', 'stage', 'level', 'code', 'box_id',
               'class_id', 'expected', 'measured', 'message'}
      DB에 저장 + UI 로그 창에 message 표시. code 목록은 box_comm/common.py LOG_CODES
      예: level='WARN', code='COUNT_MISMATCH', message='B 재고 불일치: 기대 4, 측정 6 (+2) → 측정값 반영'
      응답은 없다 (보내기만 하는 로그). 늦게 켜지면 최근 100개까지만 받는다.

UI 비상정지 버튼 (정지 기능)
  버튼을 누르면:  self.node.request_emergency_stop('ui')
  → 관제가 로봇 move_stop + 진행 중인 파트 취소 → set_job_status('STOPPED', 'EMERGENCY STOP — by ui')
  관제가 꺼져 있으면 False를 돌려준다 (UI에 '관제 연결 안 됨' 표시).

재고와 UI 적재 이미지 (v0.4)
  update_inventory로 받는 재고는 적재가 실제로 놓은 위치로 잰 수량이다 (관제가 기대값과 비교·기록함).
  적재 규칙: 1단(가장 안쪽)에 4층까지 → 다 차면 +y로 한 칸 앞 2단에 1층부터 ...
  수량 → 이미지:  from box_comm.common import stack_layout
                  stack_layout(6) → [4, 2]  (1단 4층, 2단 2층)  → 미리 만든 이미지 선택·표시

  실패하면 아무 예외나 raise → 관제에 success=False로 전달된다.
  같은 box_id 중복 무시, status 값 검사는 interface가 이미 처리한다.

[FAKE] 표시된 부분을 실제 DB(SQLite 등) 읽기/쓰기로 바꾸면 된다.
UI 연동도 이 파일(또는 여기서 import하는 모듈)에서 한다. 관제는 UI를 모른다.

실행: ros2 run box_database database_template
단독 테스트:
  ros2 service call /get_job_data box_interfaces/srv/GetJobData "{}"
  ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}"
  ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'ERROR', reason: 'Fail to pick'}"
  ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0}}"
================================================================
"""

from box_comm.common import stack_layout
from box_comm.database_interface import start

# [FAKE] 임시 기준값 — 실제 물품 실측값으로 교체 (RG2 개구 폭 이내로)
FAKE_SPECS = [
    {'class_id': 'A', 'width_mm': 40.0,  'depth_mm': 40.0, 'height_mm': 40.0, 'weight_g': 100.0},
    {'class_id': 'B', 'width_mm': 60.0,  'depth_mm': 40.0, 'height_mm': 30.0, 'weight_g': 150.0},
    {'class_id': 'C', 'width_mm': 80.0,  'depth_mm': 60.0, 'height_mm': 40.0, 'weight_g': 250.0},
    {'class_id': 'D', 'width_mm': 100.0, 'depth_mm': 80.0, 'height_mm': 50.0, 'weight_g': 400.0},
]

# [FAKE] DB에 저장된 고정 좌표 — 실제로는 DB에서 읽어 온다 (기준값)
#        common.py의 FIXED_POSES 기본값도 같은 값으로 맞춰 두면 팀원 단독 테스트가 편하다
FAKE_POSES = {
    'home_j': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    'measure_zone': [0.0, 0.0, 0.0],      # TODO: 측정존 실측값
    'place_zones': {                      # TODO: 구역별 1단 x, y 실측값 [mm, base]
        'A': [300.0, -300.0],
        'B': [400.0, -300.0],
        'C': [500.0, -300.0],
        'D': [600.0, -300.0],
    },
}


class DatabaseWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # [FAKE] 메모리 저장소 — 노드를 끄면 사라짐
        self._specs = [dict(s) for s in FAKE_SPECS]
        self._poses = {'home_j': list(FAKE_POSES['home_j']),
                       'measure_zone': list(FAKE_POSES['measure_zone']),
                       'place_zones': {c: list(xy) for c, xy in FAKE_POSES['place_zones'].items()}}
        self._inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self._status = 'BEFORE'
        self._reason = ''
        self._history = []                 # (box_id, inventory) 기록
        self._events = []                  # 이벤트 로그 (UI 로그 창용)

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def get_job_data(self):
        return [dict(s) for s in self._specs], dict(self._inventory)

    def update_inventory(self, box_id, inventory):
        self._inventory = dict(inventory)
        self._history.append((box_id, dict(inventory)))
        self.refresh_ui()

    def set_job_status(self, status, reason):
        self._status = status
        self._reason = reason
        self.refresh_ui()

    def get_fixed_poses(self):
        return {'home_j': list(self._poses['home_j']),
                'measure_zone': list(self._poses['measure_zone']),
                'place_zones': {c: list(xy) for c, xy in self._poses['place_zones'].items()}}

    def on_estop_button(self):
        # [FAKE] 실제로는 UI 비상정지 버튼의 클릭 이벤트에 연결
        if not self.node.request_emergency_stop('ui'):
            self.log.error('[database] 관제 연결 안 됨 — 로봇 펜던트의 비상정지 버튼을 사용하세요')

    def on_event(self, event):
        # [FAKE] 실제로는 DB 테이블에 저장하고 UI 로그 창에 추가
        self._events.append(event)
        # rclpy 로거는 같은 줄에서 레벨을 바꿔 부르면 오류 → 레벨마다 따로 호출
        line = (f'[database] 로그 {event["stamp"]} [{event["source"]}/{event["code"]}] '
                f'{event["box_id"]} {event["message"]}')
        if event['level'] == 'ERROR':
            self.log.error(line)
        elif event['level'] == 'WARN':
            self.log.warn(line)
        else:
            self.log.info(line)

    # ------------------------------------------------------------------
    # [FAKE] UI 갱신 자리
    # ------------------------------------------------------------------
    def refresh_ui(self):
        layout = {c: stack_layout(n) for c, n in self._inventory.items()}   # 적재 이미지용
        status = f'{self._status} : {self._reason}' if self._reason else self._status
        self.log.info(f'[database] UI 갱신: 상태={status}, 재고={self._inventory}, '
                      f'적재 상태(단별 층 수)={layout}')


def main():
    start(DatabaseWork)


if __name__ == '__main__':
    main()
