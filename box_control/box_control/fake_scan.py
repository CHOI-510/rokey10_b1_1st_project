#!/usr/bin/env python3
"""
fake_scan.py — 관제 테스트용 가짜 탐지 노드
================================================================
scan_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
통신은 scan_interface.py를 그대로 쓰므로 실제 노드와 같은 방식으로 응답한다.

파라미터
  box_count       (int,   기본 3)   이 개수만큼 found=True, 그 뒤 found=False
  fake_delay_sec  (float, 기본 1.0) 동작 하나당 가짜 소요 시간
  grasp_fail_at   (str,   기본 "")  "2,3"처럼 주면 그 번째 Goal에서 집기 실패(GRASP_FAILED, 11/17)
                                     → 관제가 공간 스캔부터 다시 하는지 시험. 박스 수는 줄지 않음
  관제가 측정존 박스를 스캔 공간으로 '밀어냈으면'(clear_zone fake_ok) 파일(BOX_FAKE_RETURN_FILE,
  기본 /tmp/box_fake_return)에 남긴 개수만큼 남은 박스를 다시 늘린다.

실행
  ros2 run box_control fake_scan
  ros2 run box_control fake_scan --ros-args -p box_count:=5 -p fake_delay_sec:=0.2
================================================================
"""

import os
import time

from rcl_interfaces.msg import ParameterDescriptor

from box_comm.scan_interface import PartError, start

ANY_TYPE = ParameterDescriptor(dynamic_typing=True)   # -p x:=2 (정수)와 -p x:="1,2" (문자열) 둘 다 받음
RETURN_FILE = os.environ.get('BOX_FAKE_RETURN_FILE', '/tmp/box_fake_return')


def take_returned() -> int:
    """관제가 스캔 공간으로 밀어낸 박스 수 (읽고 지움)"""
    try:
        with open(RETURN_FILE) as f:
            n = sum(int(x) for x in f.read().split() if x.strip())
        os.remove(RETURN_FILE)
        return n
    except (FileNotFoundError, ValueError):
        return 0


def parse_at(text) -> set:
    return {int(x) for x in str(text).split(',') if x.strip()}


class FakeScan:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('box_count', 3)
        node.declare_parameter('fake_delay_sec', 1.0)
        node.declare_parameter('grasp_fail_at', '', ANY_TYPE)
        self.remaining = node.get_parameter('box_count').value
        self.delay = node.get_parameter('fake_delay_sec').value
        self.fail_at = parse_at(node.get_parameter('grasp_fail_at').value)
        self.goals = 0
        take_returned()                                  # 이전 실행에서 남은 기록 지움
        self.log.info(f'[fake_scan] box_count={self.remaining}, delay={self.delay}s, '
                      f'grasp_fail_at={sorted(self.fail_at)}')

    def run(self, report) -> bool:
        self.goals += 1
        report('space_scan')
        time.sleep(self.delay)
        back = take_returned()
        if back:
            self.remaining += back
            self.log.info(f'[fake_scan] 측정존에서 밀려온 박스 {back}개 → 남은 박스 {self.remaining}')
        if self.remaining <= 0:
            self.log.info('[fake_scan] 남은 박스 없음 → found=False')
            return False

        report.log(f'물품 감지 (남은 박스 {self.remaining}개)')      # 이벤트 로그 예시
        report('picking')
        time.sleep(self.delay)
        if self.goals in self.fail_at:
            for i in range(2, 4):
                report.log(f'집기 실패, 그리퍼 돌려서 재시도 {i}/3 (fake)', 'WARN')
                time.sleep(self.delay * 0.3)
            raise PartError('GRASP_FAILED', f'{self.goals}번째 Goal: 3회 시도 실패 (fake)')
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
