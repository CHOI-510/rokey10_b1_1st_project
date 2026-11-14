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
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출 (관제가 진행 상황을 추적)
      stage: 'measuring_length1' → 'measuring_length2' → 'measuring_length3'
             → 'measuring_weight' → ('measuring_materiality', 옵션) → 'judging'
      (box_comm/common.py STAGES)

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
from box_comm.common import FIXED_POSES

# ----------------------------------------------------------------------
# 공통 고정 좌표 — 원본은 box_comm/common.py (관제 관리). 숫자를 직접 적지 않는다.
#   값이 바뀌면 관제가 common.py만 고치고, 모든 파트에 같이 적용된다.
#   관제는 작업 시작 전에 이 값과 DB 사본을 비교한다 (preflight).
# ----------------------------------------------------------------------
HOME_J = FIXED_POSES['home_j']              # HOME 관절 각도 [deg] → movej(HOME_J)
MEASURE_ZONE = FIXED_POSES['measure_zone']  # 측정존 x, y, z [mm, base 기준] (None = 아직 미설정)
#   측정존 자세(a, b, c)는 각 파트 동작에 맞게 정한다. 예: [*MEASURE_ZONE, 0.0, 180.0, 0.0]


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
        # 2-1 길이 측정 3회 (물품 방향에 따라 가로/세로/높이가 달라질 수 있어 번호로 구분)
        lengths = []
        for i in (1, 2, 3):
            report(f'measuring_length{i}')
            lengths.append(self.measure_length(i, specs))

        # 2-2 무게 측정
        report('measuring_weight')
        weight = self.measure_weight(specs)

        # 2-3 강성 추정 (옵션) — 구현하면 아래 주석 해제
        # report('measuring_materiality')
        # stiffness = self.measure_materiality()

        self.log.info(f'[judge] 측정값 L1={lengths[0]:.1f} L2={lengths[1]:.1f} '
                      f'L3={lengths[2]:.1f} mm, {weight:.1f} g')

        # 2-4 판별
        report('judging')
        result_class = self.classify(specs, lengths, weight)

        self.go_home()
        return result_class

    # ------------------------------------------------------------------
    # 판별 로직 (예시) — 기준이 정해지면 수정
    # ------------------------------------------------------------------
    def classify(self, specs, lengths, weight) -> str:
        """각 기준과의 상대 오차 합이 가장 작은 class_id 선택

        측정 순서가 물품 방향에 따라 달라질 수 있으므로, 측정한 길이 3개와
        기준의 W·D·H를 각각 크기순으로 정렬해서 비교한다.
        """
        def rel(a, b):
            return abs(a - b) / b if b > 0 else 0.0

        measured = sorted(lengths)

        def score(s):
            ref = sorted([s['width_mm'], s['depth_mm'], s['height_mm']])
            return sum(rel(m, r) for m, r in zip(measured, ref)) + rel(weight, s['weight_g'])

        return min(specs, key=score)['class_id']

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 측정으로 교체
    # ------------------------------------------------------------------
    def measure_length(self, i, specs):
        """그리퍼로 i번째 길이 측정 (fake: 기준 하나를 골라 ±3% 노이즈, 순서는 섞음)"""
        zone = self._measure_zone_xyz()   # 실제: 측정존(zone) 기준으로 접근해서 그리퍼로 측정
        time.sleep(self._delay * 0.5)
        if i == 1:
            self._fake_target = random.choice(specs)
            t = self._fake_target
            dims = [t['width_mm'], t['depth_mm'], t['height_mm']]
            random.shuffle(dims)                 # 물품 방향이 매번 다르다고 가정
            self._fake_dims = dims
        return self._fake_dims[i - 1] * random.uniform(0.97, 1.03)

    def measure_weight(self, specs):
        """토크센서로 무게 측정 (fake: ±5% 노이즈)"""
        time.sleep(self._delay * 0.5)
        return self._fake_target['weight_g'] * random.uniform(0.95, 1.05)

    def go_home(self):
        # 실제: movej(HOME_J)
        time.sleep(self._delay * 0.5)

    def _measure_zone_xyz(self):
        """측정존 x, y, z. 미설정이면 실제 동작에서는 움직이면 안 된다."""
        if MEASURE_ZONE is None:
            if not getattr(self, '_zone_warned', False):
                self.log.warn('측정존 좌표가 아직 미설정 (box_comm/common.py) — '
                              '[FAKE]에서는 넘어가지만 실제 동작에서는 움직이지 않도록 처리할 것')
                self._zone_warned = True
            return None
        return list(MEASURE_ZONE)


def main():
    start(JudgeWork)


if __name__ == '__main__':
    main()
