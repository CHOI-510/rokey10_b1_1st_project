#!/usr/bin/env python3
"""
scan.py — 탐지 파트 메인 (예시: fake 동작)
================================================================
담당자가 작성하는 파일. 통신은 scan_interface.py가 처리한다.

지킬 약속은 하나:
  run(report) -> bool
    - 물품을 측정존에 두고 홈 복귀까지 끝났으면 True
    - 작업공간에 물품이 없으면 False
    - 실패하면 raise PartError('GRASP_FAILED', '이유')  (등록된 코드만)
    - report('단계명')은 진행 알림 (선택)

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다.
필요하면 모션·감지 함수를 다른 파일로 나눠 import해도 된다.

실행: ros2 run box_scan scan
      ros2 run box_scan scan --ros-args -p box_count:=5   (fake 박스 개수)
단독 테스트:
      ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
================================================================
"""

import time

from box_comm.scan_interface import PartError, start  # noqa: F401  (PartError는 실패 시 사용)


class ScanWork:
    def __init__(self, node):
        self.node = node                   # 로그, 파라미터 등에 사용
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시 (예시 — 기어 조립 프로젝트 방식과 맞춰 확인) ----
        # Action 서버 노드(self.node)와 로봇 제어 노드를 분리한다.
        # import rclpy, DR_init
        # DR_init.__dsr__id = 'dsr01'
        # DR_init.__dsr__model = 'm0609'
        # self.dsr_node = rclpy.create_node('scan_dsr', namespace='dsr01')
        # DR_init.__dsr__node = self.dsr_node
        # from DSR_ROBOT2 import movej, movel, ...    # 노드 등록 후 import

        # [FAKE 전용] 실제 구현 시 삭제
        #   box_count: "작업공간에 박스가 N개 있다"고 가정 (인터페이스와 무관)
        #   fake_delay_sec: 로봇 동작 대신 기다리는 시간
        node.declare_parameter('box_count', 3)
        node.declare_parameter('fake_delay_sec', 1.0)
        self._remaining = node.get_parameter('box_count').value
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, report) -> bool:
        # 1-1 작업공간 스캔, 1-2 물품 감지
        report('scanning')
        if not self.detect_box():
            self.go_home()
            return False

        # 1-3 픽업 (실패 시 raise PartError('GRASP_FAILED', ...))
        report('picking')
        self.pick()

        # 1-4 측정존 이동 후 내려놓기
        report('moving')
        self.place_on_measure_zone()

        self.go_home()
        return True

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 동작으로 교체
    # ------------------------------------------------------------------
    def detect_box(self) -> bool:
        """스캔 경로를 돌며 외력 감지 등으로 물품 존재 확인"""
        time.sleep(self._delay)
        if self._remaining <= 0:
            return False
        self._remaining -= 1
        return True

    def pick(self):
        time.sleep(self._delay)

    def place_on_measure_zone(self):
        time.sleep(self._delay)

    def go_home(self):
        time.sleep(self._delay * 0.5)


def main():
    start(ScanWork)


if __name__ == '__main__':
    main()
