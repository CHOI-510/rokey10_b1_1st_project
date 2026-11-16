#!/usr/bin/env python3
"""
fake_database.py — 관제 테스트용 가짜 DB 노드
================================================================
database_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
데이터는 메모리에만 있고, 노드를 끄면 사라진다.
UI 대신 상태·재고가 바뀔 때마다 로그로 보여 준다.
고정 좌표는 box_comm/common.py의 FIXED_POSES를 복사해서 돌려준다.
미설정(None)인 측정존·적재 구역은 아래 FAKE 값으로 채운다 (사전 점검 통과용).

파라미터 (preflight 테스트용)
  pose_offset      (float, 기본 0.0)  고정 좌표에 이 값을 더해서 돌려줌 → 불일치 테스트
  poses_available  (bool,  기본 True) False면 좌표 없음(available=False)으로 응답
  start_inventory  (int[], 기본 [0,0,0,0,0]) A~D(+U) 처음 재고 → fake_place의 fake_start와 다르게 주면 불일치 테스트
                   4개만 주면 U(미분류)는 0
  estop_at         (int, 기본 0)  N번째 박스 재고 반영 직후 UI 비상정지 버튼을 누른 것처럼 동작 (0 = 끔)

실행
  ros2 run box_control fake_database
  ros2 run box_control fake_database --ros-args -p pose_offset:=5.0
================================================================
"""

from box_comm.common import FIXED_POSES, INVENTORY_KEYS, UNCLASSIFIED, stack_layout
from box_comm.database_interface import start

FAKE_SPECS = [
    {'class_id': 'A', 'width_mm': 40.0,  'depth_mm': 40.0, 'height_mm': 40.0, 'weight_g': 100.0},
    {'class_id': 'B', 'width_mm': 60.0,  'depth_mm': 40.0, 'height_mm': 30.0, 'weight_g': 150.0},
    {'class_id': 'C', 'width_mm': 80.0,  'depth_mm': 60.0, 'height_mm': 40.0, 'weight_g': 250.0},
    {'class_id': 'D', 'width_mm': 100.0, 'depth_mm': 80.0, 'height_mm': 50.0, 'weight_g': 400.0},
]


# 미설정일 때 쓰는 가짜 적재 구역 1단 x, y [mm]
FAKE_PLACE_ZONES = {'A': [300.0, -300.0], 'B': [400.0, -300.0],
                    'C': [500.0, -300.0], 'D': [600.0, -300.0]}


class FakeDatabase:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.specs = [dict(s) for s in FAKE_SPECS]
        self.inventory = {c: 0 for c in INVENTORY_KEYS}
        self.status = 'BEFORE'
        self.history = []
        node.declare_parameter('pose_offset', 0.0)
        node.declare_parameter('poses_available', True)
        node.declare_parameter('start_inventory', [0, 0, 0, 0, 0])   # A~D, U 처음 재고 (불일치 테스트용)
        self.estop_at = int(node.declare_parameter('estop_at', 0).value)   # 비상정지 버튼 테스트
        start = [int(n) for n in node.get_parameter('start_inventory').value]
        self.inventory = {c: (start[i] if i < len(start) else 0) for i, c in enumerate(INVENTORY_KEYS)}
        self.pose_offset = float(node.get_parameter('pose_offset').value)
        self.poses_available = bool(node.get_parameter('poses_available').value)
        self._show('시작')

    def get_job_data(self):
        return [dict(s) for s in self.specs], dict(self.inventory)

    def update_inventory(self, box_id, inventory):
        self.inventory = dict(inventory)
        self.history.append(box_id)
        self._show(f'{box_id} 반영')
        if self.estop_at and len(self.history) == self.estop_at:
            self.log.warn(f'[fake_db] (테스트) {self.estop_at}번째 박스 뒤 UI 비상정지 버튼')
            self.node.request_emergency_stop('fake_db')

    def set_job_status(self, status, reason=''):
        self.status = f'{status} : {reason}' if reason else status
        self._show(f'상태 {self.status}')

    def get_fixed_poses(self):
        if not self.poses_available:
            return None
        off = self.pose_offset
        home = [v + off for v in FIXED_POSES['home_j']]
        zone = FIXED_POSES['measure_zone'] or [0.0, 0.0, 0.0]
        zone = [v + off for v in zone]
        if off:
            self.log.warn(f'[fake_db] (테스트) 고정 좌표에 {off} 더해서 반환')
        zones = {c: list(FIXED_POSES['place_zones'].get(c) or FAKE_PLACE_ZONES[c])
                 for c in 'ABCD'}
        return {'home_j': home, 'measure_zone': zone, 'place_zones': zones}

    def on_event(self, event):
        # rclpy 로거는 같은 줄에서 레벨을 바꿔 부르면 오류 → 레벨마다 따로 호출
        line = f'[fake_db] 로그 [{event["source"]}/{event["code"]}] {event["box_id"]} {event["message"]}'
        if event['level'] == 'ERROR':
            self.log.error(line)
        elif event['level'] == 'WARN':
            self.log.warn(line)
        else:
            self.log.info(line)

    def _show(self, event):
        self.log.info(
            f'[fake_db] ({event}) 상태={self.status} | 재고={self.inventory} '
            f'| 처리 {len(self.history)}개 | 적재 이미지 '
            + ' '.join(f'{c}{stack_layout(n)}' for c, n in self.inventory.items() if c != UNCLASSIFIED)
            + f' | 미분류 {self.inventory.get(UNCLASSIFIED, 0)}개')


def main():
    start(FakeDatabase)


if __name__ == '__main__':
    main()
