#!/usr/bin/env python3
"""
fake_judge.py — 관제 테스트용 가짜 판별 노드
================================================================
judge_template / 담당 메인 코드 대신 띄워서 관제(control.py)를 테스트한다.
받은 specs를 로그로 보여 주고, 판별 결과를 정해진 순서나 무작위로 돌려준다.

파라미터
  class_sequence  (str,   기본 "")  "A,B,C,D"처럼 주면 그 순서대로 반복해서 반환
                                     U(미분류, v0.5)도 넣을 수 있음: "A,U,B"
                                     비워 두면 받은 specs(A~D) 중 무작위
  fake_delay_sec  (float, 기본 1.0) 동작 하나당 가짜 소요 시간
  grasp_fail_at   (str,   기본 "")  "2"처럼 주면 그 번째 Goal에서 집기 실패(GRASP_FAILED, 11/17).
                                     관제가 측정존을 비우고(fake면 비운 셈 치고) 재스캔

실행
  ros2 run box_control fake_judge
  ros2 run box_control fake_judge --ros-args -p class_sequence:="A,A,B,D"
================================================================
"""

import random
import time

from rcl_interfaces.msg import ParameterDescriptor

from box_comm.judge_interface import PartError, start


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


class FakeJudge:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('class_sequence', '')
        node.declare_parameter('fake_delay_sec', 1.0)
        node.declare_parameter('grasp_fail_at', '', ANY_TYPE)
        self.fail_at = parse_at(node.get_parameter('grasp_fail_at').value)
        self.goals = 0
        seq = node.get_parameter('class_sequence').value
        self.sequence = [c.strip().upper() for c in seq.split(',') if c.strip()]
        self.delay = node.get_parameter('fake_delay_sec').value
        self.count = 0
        mode = f'순서 {self.sequence}' if self.sequence else '무작위'
        self.log.info(f'[fake_judge] 판별 모드: {mode}, delay={self.delay}s')

    def run(self, specs, report) -> str:
        for s in specs:
            self.log.info(
                f'[fake_judge] 기준 {s["class_id"]}: {s["width_mm"]}x{s["depth_mm"]}x'
                f'{s["height_mm"]} mm, {s["weight_g"]} g')

        self.goals += 1
        for stage in ('measuring_length1', 'measuring_length2', 'measuring_length3',
                      'measuring_weight'):
            report(stage)
            time.sleep(self.delay * 0.25)
            if stage == 'measuring_length2':      # 가로 측정에서 집기 실패를 흉내
                fake_grasp_failure(self.log, report, self.goals, self.fail_at, self.delay)
        report('judging')
        time.sleep(self.delay * 0.5)

        if self.sequence:
            result = self.sequence[self.count % len(self.sequence)]
        else:
            result = random.choice(specs)['class_id']
        self.count += 1
        self.log.info(f'[fake_judge] {self.count}번째 판별 → {result}')
        return result


def main():
    start(FakeJudge)


if __name__ == '__main__':
    main()
