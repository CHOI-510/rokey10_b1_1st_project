#!/usr/bin/env python3
"""
fake_place.py — 관제 테스트용 가짜 적재 노드
================================================================
place_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
받은 inventory에서 해당 클래스를 1 올려서 돌려준다.

파라미터
  fake_delay_sec  (float, 기본 1.0) 동작 하나당 가짜 소요 시간
  miscount_at     (int,   기본 0)   N번째 적재에서 일부러 틀린 재고를 반환 (관제 재고 검증 테스트용)

실행
  ros2 run box_control fake_place
================================================================
"""

import time

from box_comm.place_interface import start


class FakePlace:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('fake_delay_sec', 1.0)
        node.declare_parameter('miscount_at', 0)   # N번째 적재에서 일부러 틀린 재고 반환 (0이면 끔)
        self.delay = node.get_parameter('fake_delay_sec').value
        self.miscount_at = node.get_parameter('miscount_at').value
        self.count = 0
        self.log.info(f'[fake_place] delay={self.delay}s, miscount_at={self.miscount_at}')

    def run(self, result_class, inventory, report) -> dict:
        report('picking')
        time.sleep(self.delay * 0.5)
        report('moving')
        time.sleep(self.delay)
        report('placing')
        time.sleep(self.delay * 0.5)

        self.count += 1
        if self.count == self.miscount_at:
            # 관제 재고 검증 테스트용: 엉뚱한 칸을 +1 해서 반환
            wrong = next(c for c in ('A', 'B', 'C', 'D') if c != result_class)
            inventory[wrong] += 1
            self.log.warn(f'[fake_place] (테스트) 일부러 틀린 재고 반환: {wrong} +1 → {inventory}')
            return inventory

        inventory[result_class] += 1
        self.log.info(f'[fake_place] {result_class} 적재 → {inventory}')
        return inventory


def main():
    start(FakePlace)


if __name__ == '__main__':
    main()
