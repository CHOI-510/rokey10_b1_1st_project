#!/usr/bin/env python3
"""
judge.py — 판별 파트 메인 (예시: fake 동작)
================================================================
담당자가 작성하는 파일. 통신은 judge_interface.py가 처리한다.

지킬 약속은 하나:
  run(specs, report) -> str
    - specs: [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
               'height_mm': 40.0, 'weight_g': 100.0}, ...]   (DB의 물품값)
    - 측정존 물품을 측정하고 'A' / 'B' / 'C' / 'D' 중 하나를 반환
    - 실패하면 raise PartError('MEASURE_FAILED', '이유')  (등록된 코드만)
    - report('단계명')은 진행 알림 (선택)

[FAKE] 표시된 부분을 실제 측정으로 바꾸면 된다.
classify()는 예시 로직 — 분류 기준·오차 범위가 정해지면 수정.

실행: ros2 run box_judge judge
단독 테스트:
  ros2 action send_goal /judge box_interfaces/action/Judge \
    "{specs: [{class_id: 'A', width_mm: 40.0, depth_mm: 40.0, height_mm: 40.0, weight_g: 100.0},
              {class_id: 'B', width_mm: 60.0, depth_mm: 40.0, height_mm: 30.0, weight_g: 150.0}]}" --feedback
================================================================
"""

import random
import time

from box_comm.judge_interface import PartError, start  # noqa: F401


class JudgeWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시: scan.py의 DR_init 예시 참고 (노드 이름만 'judge_dsr') ----

        # [FAKE 전용] 실제 구현 시 삭제
        node.declare_parameter('fake_delay_sec', 1.0)
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, specs, report) -> str:
        # 2-1 W·D·H 측정, 2-2 무게 측정
        report('measuring')
        w, d, h = self.measure_size(specs)
        weight = self.measure_weight(specs)
        self.log.info(f'[judge] 측정값 W={w:.1f} D={d:.1f} H={h:.1f} mm, {weight:.1f} g')

        # 2-4 판별
        report('judging')
        result_class = self.classify(specs, w, d, h, weight)

        self.go_home()
        return result_class

    # ------------------------------------------------------------------
    # 판별 로직 (예시) — 기준이 정해지면 수정
    # ------------------------------------------------------------------
    def classify(self, specs, w, d, h, weight) -> str:
        """각 기준과의 상대 오차 합이 가장 작은 class_id 선택"""
        def rel(a, b):
            return abs(a - b) / b if b > 0 else 0.0

        best = min(specs, key=lambda s: rel(w, s['width_mm']) + rel(d, s['depth_mm'])
                   + rel(h, s['height_mm']) + rel(weight, s['weight_g']))
        return best['class_id']

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 측정으로 교체
    # ------------------------------------------------------------------
    def measure_size(self, specs):
        """그리퍼로 W·D·H 측정 (fake: 기준 하나를 골라 ±3% 노이즈)"""
        time.sleep(self._delay)
        self._fake_target = random.choice(specs)
        t = self._fake_target
        n = lambda v: v * random.uniform(0.97, 1.03)
        return n(t['width_mm']), n(t['depth_mm']), n(t['height_mm'])

    def measure_weight(self, specs):
        """토크센서로 무게 측정 (fake: ±5% 노이즈)"""
        time.sleep(self._delay * 0.5)
        return self._fake_target['weight_g'] * random.uniform(0.95, 1.05)

    def go_home(self):
        time.sleep(self._delay * 0.5)


def main():
    start(JudgeWork)


if __name__ == '__main__':
    main()
