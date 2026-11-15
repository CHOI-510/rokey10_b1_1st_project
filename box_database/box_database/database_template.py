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

지킬 약속은 네 개:
  get_job_data() -> (specs, inventory)
      specs     = [{'class_id': 'A', 'width_mm': .., 'depth_mm': ..,
                    'height_mm': .., 'weight_g': ..}, ...]
      inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
  update_inventory(box_id, inventory) -> None
  set_job_status(status) -> None         status: 'BEFORE' / 'RUNNING' / 'DONE'
  get_fixed_poses() -> dict               DB에 저장된 고정 좌표 사본
      {'home_j': [6개, deg], 'measure_zone': [x, y, z, mm]}
      원본은 box_comm/common.py의 FIXED_POSES. 관제가 작업 시작 전에 둘을 비교해서
      다르면 "측정존 설정값이 DB와 다릅니다"로 시작하지 않는다.
      저장된 값이 없으면 None 반환 (관제는 경고만 남기고 진행)

  실패하면 아무 예외나 raise → 관제에 success=False로 전달된다.
  같은 box_id 중복 무시, status 값 검사는 interface가 이미 처리한다.

[FAKE] 표시된 부분을 실제 DB(SQLite 등) 읽기/쓰기로 바꾸면 된다.
UI 연동도 이 파일(또는 여기서 import하는 모듈)에서 한다. 관제는 UI를 모른다.

실행: ros2 run box_database database_template
단독 테스트:
  ros2 service call /get_job_data box_interfaces/srv/GetJobData "{}"
  ros2 service call /set_job_status box_interfaces/srv/SetJobStatus "{status: 'RUNNING'}"
  ros2 service call /update_inventory box_interfaces/srv/UpdateInventory \
    "{box_id: 'box_001', inventory: {a: 1, b: 0, c: 0, d: 0}}"
================================================================
"""

from box_comm.database_interface import start

# [FAKE] 임시 기준값 — 실제 물품 실측값으로 교체 (RG2 개구 폭 이내로)
FAKE_SPECS = [
    {'class_id': 'A', 'width_mm': 40.0,  'depth_mm': 40.0, 'height_mm': 40.0, 'weight_g': 100.0},
    {'class_id': 'B', 'width_mm': 60.0,  'depth_mm': 40.0, 'height_mm': 30.0, 'weight_g': 150.0},
    {'class_id': 'C', 'width_mm': 80.0,  'depth_mm': 60.0, 'height_mm': 40.0, 'weight_g': 250.0},
    {'class_id': 'D', 'width_mm': 100.0, 'depth_mm': 80.0, 'height_mm': 50.0, 'weight_g': 400.0},
]

# [FAKE] DB에 저장된 고정 좌표 사본 — 실제로는 DB에서 읽어 온다
#        값은 box_comm/common.py의 FIXED_POSES와 같아야 한다
FAKE_POSES = {
    'home_j': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
    'measure_zone': [0.0, 0.0, 0.0],      # TODO: 측정존 실측값 (common.py와 같은 값으로)
}


class DatabaseWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # [FAKE] 메모리 저장소 — 노드를 끄면 사라짐
        self._specs = [dict(s) for s in FAKE_SPECS]
        self._poses = {k: list(v) for k, v in FAKE_POSES.items()}
        self._inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self._status = 'BEFORE'
        self._history = []                 # (box_id, inventory) 기록

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def get_job_data(self):
        return [dict(s) for s in self._specs], dict(self._inventory)

    def update_inventory(self, box_id, inventory):
        self._inventory = dict(inventory)
        self._history.append((box_id, dict(inventory)))
        self.refresh_ui()

    def set_job_status(self, status):
        self._status = status
        self.refresh_ui()

    def get_fixed_poses(self):
        return {k: list(v) for k, v in self._poses.items()}

    # ------------------------------------------------------------------
    # [FAKE] UI 갱신 자리
    # ------------------------------------------------------------------
    def refresh_ui(self):
        self.log.info(f'[database] UI 갱신: 상태={self._status}, 재고={self._inventory}')


def main():
    start(DatabaseWork)


if __name__ == '__main__':
    main()
