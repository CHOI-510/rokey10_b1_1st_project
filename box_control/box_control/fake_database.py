#!/usr/bin/env python3
"""
fake_database.py — 관제 테스트용 가짜 DB 노드
================================================================
실제 database.py 대신 띄워서 관제(control.py)를 테스트한다.
데이터는 메모리에만 있고, 노드를 끄면 사라진다.
UI 대신 상태·재고가 바뀔 때마다 로그로 보여 준다.

실행
  ros2 run box_control fake_database
================================================================
"""

from box_comm.database_interface import start

FAKE_SPECS = [
    {'class_id': 'A', 'width_mm': 40.0,  'depth_mm': 40.0, 'height_mm': 40.0, 'weight_g': 100.0},
    {'class_id': 'B', 'width_mm': 60.0,  'depth_mm': 40.0, 'height_mm': 30.0, 'weight_g': 150.0},
    {'class_id': 'C', 'width_mm': 80.0,  'depth_mm': 60.0, 'height_mm': 40.0, 'weight_g': 250.0},
    {'class_id': 'D', 'width_mm': 100.0, 'depth_mm': 80.0, 'height_mm': 50.0, 'weight_g': 400.0},
]


class FakeDatabase:
    def __init__(self, node):
        self.log = node.get_logger()
        self.specs = [dict(s) for s in FAKE_SPECS]
        self.inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.status = 'BEFORE'
        self.history = []
        self._show('시작')

    def get_job_data(self):
        return [dict(s) for s in self.specs], dict(self.inventory)

    def update_inventory(self, box_id, inventory):
        self.inventory = dict(inventory)
        self.history.append(box_id)
        self._show(f'{box_id} 반영')

    def set_job_status(self, status):
        self.status = status
        self._show(f'상태 {status}')

    def _show(self, event):
        self.log.info(
            f'[fake_db] ({event}) 상태={self.status} | 재고={self.inventory} '
            f'| 처리 {len(self.history)}개')


def main():
    start(FakeDatabase)


if __name__ == '__main__':
    main()
