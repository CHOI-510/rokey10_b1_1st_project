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

실행
  ros2 run box_control fake_judge
  ros2 run box_control fake_judge --ros-args -p class_sequence:="A,A,B,D"
================================================================
"""

import random
import time

from box_comm.judge_interface import start


class FakeJudge:
    def __init__(self, node):
        self.log = node.get_logger()
        node.declare_parameter('class_sequence', '')
        node.declare_parameter('fake_delay_sec', 1.0)
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

        for stage in ('measuring_length1', 'measuring_length2', 'measuring_length3',
                      'measuring_weight'):
            report(stage)
            time.sleep(self.delay * 0.25)
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
