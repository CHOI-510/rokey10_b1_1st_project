#!/usr/bin/env python3
"""
database_template.py — DB 노드 약속 예시 (관제 관리, 참고용)
================================================================
  실제 DB 노드는 database_main.py (+ database_work / database_store / database_config).
  이 파일은 'database_interface가 무엇을 부르고 무엇을 돌려받는지'만 보여 주는 가장 짧은 예시다.
  값은 메모리에만 있고, Supabase·로그 파일·버튼 창구는 없다.
  실행 비교: ros2 run box_database database_template   (database_main과 동시에 켜지 말 것 — 같은 서버 이름)

약속 (database_interface가 부름)
  get_job_data() -> (specs, inventory)
      specs     = [{'class_id': 'A', 'width_mm', 'depth_mm', 'height_mm', 'weight_g'}, ...]  A~D 4개
      inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'U': 0}        U = 미분류
  get_fixed_poses() -> dict | None        (필수: None이거나 구역이 빠지면 관제가 시작 안 함)
      {'measure_zone': [x, y, z], 'place_zones': {'A': [x, y], ..., 'D': [x, y]}}
      'home_j'는 넣지 않아도 됨 (common.FIXED_POSES['home_j']를 interface가 채움)
  update_inventory(box_id, inventory) -> None     재고 최종값 (증감 아님). 같은 box_id는 interface가 거름
  set_job_status(status, reason) -> None          BEFORE / RUNNING / DONE / ERROR / STOPPED (+ 이유)
  on_event(event) -> None          (선택) /event_log 한 줄. seq로 중복 거르기 (재시작하면 최근 100개 다시 옴)
  on_state(state) -> None          (선택) /control_status JSON. 관제 단계·박스·로봇 상태. 2초마다 다시 옴

DB 노드가 부르는 것 (UI 버튼)
  self.node.request_emergency_stop('ui')          -> bool (False = 관제 없음)
  self.node.request_job_command('start', 'ui')    -> (ok, message)
      ※ 응답을 기다리므로 ROS 콜백 안에서 부르지 말고 별도 스레드에서 (database_work는 HTTP 스레드)

실패하면 아무 예외나 raise → 관제에 success=False → 관제가 'DB failed'로 멈춤.

단독 시험
  ros2 service call /get_job_data box_interfaces/srv/GetJobData "{}"
  ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'ERROR', reason: 'Fail to pick'}"
  ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \\
    "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0, u: 0}}"
================================================================
"""

from box_comm.common import stack_layout
from box_comm.database_interface import start

SPECS = [   # 실제 값은 Supabase product
    {'class_id': 'A', 'width_mm': 60.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 130.0},
    {'class_id': 'B', 'width_mm': 75.0, 'depth_mm': 29.0, 'height_mm': 25.0, 'weight_g': 0.0},
    {'class_id': 'C', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 108.0},
    {'class_id': 'D', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 55.3},
]
POSES = {   # 실제 값은 Supabase loc
    'measure_zone': [300.0, -160.0, 30.9],
    'place_zones': {'A': [480.0, -160.0], 'B': [650.0, -160.0], 'C': [580.0, 300.0], 'D': [300.0, 300.0]},
}


class DatabaseWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'U': 0}
        self.status = 'BEFORE'

    def get_job_data(self):
        return [dict(s) for s in SPECS], dict(self.inventory)

    def get_fixed_poses(self):
        return {'measure_zone': list(POSES['measure_zone']),
                'place_zones': {c: list(xy) for c, xy in POSES['place_zones'].items()}}

    def update_inventory(self, box_id, inventory):
        self.inventory = dict(inventory)
        self.log.info(f'[db_template] {box_id} 재고 {inventory} | 적재 그림 '
                      + ' '.join(f'{c}{stack_layout(n)}' for c, n in inventory.items() if c != 'U'))

    def set_job_status(self, status, reason):
        self.status = f'{status} : {reason}' if reason else status
        self.log.info(f'[db_template] 상태 {self.status}')

    def on_event(self, event):
        self.log.info(f'[db_template] 로그 #{event["seq"]} [{event["source"]}/{event["code"]}] {event["message"]}')

    def on_state(self, state):
        key = (state.get('state'), state.get('box_seq'), state.get('robot_state_name'))
        if key == getattr(self, '_last_state', None):
            return                       # 2초마다 같은 값이 다시 옴 → 바뀔 때만 출력
        self._last_state = key
        self.log.info(f'[db_template] 관제 {state.get("state")} 박스 {state.get("box_seq")} '
                      f'로봇 {state.get("robot_state_name")} ok={state.get("robot_ok")}')


def main():
    start(DatabaseWork)


if __name__ == '__main__':
    main()
