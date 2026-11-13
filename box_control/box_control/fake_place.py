#!/usr/bin/env python3
"""
fake_place.py — 관제 테스트용 가짜 적재 노드
================================================================
실제 place.py 대신 띄워서 관제(control.py)를 테스트한다.
받은 inventory에서 해당 클래스를 1 올려서 돌려준다.

파라미터
  fake_delay_sec  (float, 기본 1.0) 동작 하나당 가짜 소요 시간

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
        self.delay = node.get_parameter('fake_delay_sec').value
        self.log.info(f'[fake_place] delay={self.delay}s')

    def run(self, result_class, inventory, report) -> dict:
        report('moving')
        time.sleep(self.delay)
        report('placing')
        time.sleep(self.delay)

        inventory[result_class] += 1
        self.log.info(f'[fake_place] {result_class} 적재 → {inventory}')
        return inventory


def main():
    start(FakePlace)


if __name__ == '__main__':
    main()
