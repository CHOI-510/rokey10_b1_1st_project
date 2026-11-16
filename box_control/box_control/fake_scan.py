#!/usr/bin/env python3
"""
fake_scan.py — 관제 테스트용 가짜 탐지 노드
================================================================
scan_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
통신은 scan_interface.py를 그대로 쓰므로 실제 노드와 같은 방식으로 응답한다.

파라미터
  box_count       (int,   기본 3)   이 개수만큼 found=True, 그 뒤 found=False
  fake_delay_sec  (float, 기본 1.0) 동작 하나당 가짜 소요 시간

실행
  ros2 run box_control fake_scan
  ros2 run box_control fake_scan --ros-args -p box_count:=5 -p fake_delay_sec:=0.2
================================================================
"""

import time

from box_comm.scan_interface import start


class FakeScan:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('box_count', 3)
        node.declare_parameter('fake_delay_sec', 1.0)
        self.remaining = node.get_parameter('box_count').value
        self.delay = node.get_parameter('fake_delay_sec').value
        self.log.info(f'[fake_scan] box_count={self.remaining}, delay={self.delay}s')

    def run(self, report) -> bool:
        report('space_scan')
        time.sleep(self.delay)
        if self.remaining <= 0:
            self.log.info('[fake_scan] 남은 박스 없음 → found=False')
            return False

        report.log(f'물품 감지 (남은 박스 {self.remaining}개)')      # 이벤트 로그 예시
        report('picking')
        time.sleep(self.delay)
        report('moving')
        time.sleep(self.delay)
        report('placing')
        time.sleep(self.delay * 0.5)

        self.remaining -= 1
        self.log.info(f'[fake_scan] 측정존에 놓음 (남은 박스 {self.remaining})')
        return True


def main():
    start(FakeScan)


if __name__ == '__main__':
    main()
