#!/usr/bin/env python3
"""
fake_place.py — 관제 테스트용 가짜 적재 노드 (v0.4)
================================================================
place_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
구역별로 '실제로 쌓인 개수'를 메모리에 두고, 적재할 때마다 +1 한 뒤
그 수량(count)을 돌려준다. (실제 적재의 '놓은 위치로 수량 측정' 흉내)
미분류 U(v0.5)는 놓기만 하고 0을 돌려준다 (수량은 관제가 셈).

파라미터
  fake_delay_sec  (float,  기본 1.0)          동작 하나당 가짜 소요 시간
  fake_start      (int[4], 기본 [0,0,0,0])    A~D 구역에 처음부터 쌓여 있는 개수
                  → DB 재고와 다르게 주면 첫 적재에서 관제 불일치(측정값 반영) 테스트
  miscount_at     (int,    기본 0)            N번째 적재에서 수량을 일부러 +1 틀리게 측정 (0이면 끔)
  grasp_fail_at   (str,    기본 "")           "2"처럼 주면 그 번째 Goal에서 집기 실패(GRASP_FAILED, 11/17)
                  → 관제가 측정존을 비우고(fake면 비운 셈 치고) 재스캔

실행
  ros2 run box_control fake_place
  ros2 run box_control fake_place --ros-args -p fake_start:="[0, 5, 0, 0]"
  ros2 run box_control fake_place --ros-args -p miscount_at:=2
================================================================
"""

import time

from rcl_interfaces.msg import ParameterDescriptor

from box_comm.common import PartError, position_from_count, stack_capacity
from box_comm.place_interface import start


ANY_TYPE = ParameterDescriptor(dynamic_typing=True)   # -p x:=2 (정수)와 -p x:="1,2" (문자열) 둘 다 받음


def parse_at(text) -> set:
    return {int(x) for x in str(text).split(',') if x.strip()}


def fake_grasp_failure(log, report, goal_no, fail_at, delay):
    """(fake) 집기 실패 흉내: 재시도 로그 → GRASP_FAILED (측정존 비우기는 관제가 함)"""
    if goal_no not in fail_at:
        return
    for i in range(2, 4):
        report.log(f'집기 실패, 재시도 {i}/3 (fake)', 'WARN')
        time.sleep(delay * 0.3)
    log.info('[fake] 3회 집기 실패 → GRASP_FAILED (관제가 측정존을 비우고 재스캔)')
    raise PartError('GRASP_FAILED', f'{goal_no}번째 Goal: 3회 시도 실패 (fake)')


class FakePlace:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('fake_delay_sec', 1.0)
        node.declare_parameter('fake_start', [0, 0, 0, 0])
        node.declare_parameter('miscount_at', 0)
        node.declare_parameter('grasp_fail_at', '', ANY_TYPE)
        self.fail_at = parse_at(node.get_parameter('grasp_fail_at').value)
        self.goals = 0
        self.delay = node.get_parameter('fake_delay_sec').value
        self.stack = dict(zip('ABCD', [int(n) for n in node.get_parameter('fake_start').value]))
        self.miscount_at = node.get_parameter('miscount_at').value
        self.count = 0
        self.log.info(f'[fake_place] delay={self.delay}s, 처음 쌓인 개수={self.stack}, '
                      f'miscount_at={self.miscount_at}')

    def run(self, result_class, report) -> int:
        self.goals += 1
        report('picking')
        time.sleep(self.delay * 0.5)
        fake_grasp_failure(self.log, report, self.goals, self.fail_at, self.delay)
        report('moving')
        time.sleep(self.delay)
        report('placing')
        time.sleep(self.delay * 0.5)

        if result_class == 'U':
            self.count += 1
            self.log.info('[fake_place] 미분류 투하 → 0 반환 (수량은 관제가 셈)')
            report.log('미분류 투하 완료 (fake)')
            return 0
        if self.stack[result_class] >= stack_capacity():
            raise PartError('NO_SLOT', f'{result_class} 구역 꽉 참 ({stack_capacity()}개)')
        self.stack[result_class] += 1
        self.count += 1

        measured = self.stack[result_class]
        if self.count == self.miscount_at and measured < stack_capacity():
            measured += 1      # 관제 불일치 처리 테스트용: 측정이 하나 틀렸다고 가정
            self.log.warn(f'[fake_place] (테스트) 일부러 틀린 수량 측정: {measured}')

        tier, level = position_from_count(measured)
        self.log.info(f'[fake_place] {result_class} {tier}단 {level}층 → 수량 {measured}')
        return measured


def main():
    start(FakePlace)


if __name__ == '__main__':
    main()
