#!/usr/bin/env python3
"""
place.py — 적재 파트 메인 (예시: fake 동작)
================================================================
담당자가 작성하는 파일. 통신은 place_interface.py가 처리한다.

지킬 약속은 하나:
  run(result_class, inventory, report) -> dict
    - result_class: 'A' / 'B' / 'C' / 'D'
    - inventory:    {'A': 1, 'B': 0, 'C': 2, 'D': 0}   (적재 전)
    - 적재 후 inventory를 같은 형식의 dict로 반환
    - 슬롯 배정·좌표는 이 파일 안에서만 처리 (관제·인터페이스는 모름)
    - 실패하면 raise PartError('PLACE_FAILED' 또는 'NO_SLOT', '이유')
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출 (관제가 진행 상황을 추적)
      stage: 'picking' → 'moving' → 'placing'  (box_comm/common.py STAGES)
    - 반환한 재고는 관제가 검증한다 (적재 전 재고에서 result_class 칸만 +1이어야 함)

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다.

실행: ros2 run box_place place
단독 테스트:
  ros2 action send_goal /place box_interfaces/action/Place \
    "{result_class: 'B', inventory: {a: 1, b: 0, c: 2, d: 0}}" --feedback
================================================================
"""

import time

from box_comm.place_interface import PartError, start
from box_comm.common import FIXED_POSES

# ----------------------------------------------------------------------
# 공통 고정 좌표 — 원본은 box_comm/common.py (관제 관리). 숫자를 직접 적지 않는다.
#   값이 바뀌면 관제가 common.py만 고치고, 모든 파트에 같이 적용된다.
#   관제는 작업 시작 전에 이 값과 DB 사본을 비교한다 (preflight).
# ----------------------------------------------------------------------
HOME_J = FIXED_POSES['home_j']              # HOME 관절 각도 [deg] → movej(HOME_J)
MEASURE_ZONE = FIXED_POSES['measure_zone']  # 측정존 x, y, z [mm, base 기준] (None = 아직 미설정)
#   측정존 자세(a, b, c)는 각 파트 동작에 맞게 정한다. 예: [*MEASURE_ZONE, 0.0, 180.0, 0.0]

SLOTS_PER_CLASS = 4      # 예시: 클래스당 적재 칸 수 — 실제 레이아웃에 맞게 수정


class PlaceWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시: scan.py의 DR_init 예시 참고 (노드 이름만 'place_dsr') ----

        # [FAKE 전용] 실제 구현 시 삭제
        node.declare_parameter('fake_delay_sec', 1.0)
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, result_class, inventory, report) -> dict:
        # 3-4 슬롯 배정 (내부 전용)
        slot = self.assign_slot(result_class, inventory)

        # 3-1 측정존 물품 픽업
        report('picking')
        self.pick_from_measure_zone()

        # 3-2 적재 위치로 이동
        report('moving')
        self.move_to_slot(result_class, slot)

        # 3-3 적재
        report('placing')
        self.place_to_slot(result_class, slot)

        self.go_home()

        # 적재 후 재고
        inventory[result_class] += 1
        return inventory

    # ------------------------------------------------------------------
    # 슬롯 배정 (예시) — 실제 레이아웃이 정해지면 수정
    # ------------------------------------------------------------------
    def assign_slot(self, result_class, inventory) -> int:
        slot = inventory[result_class]          # n번째 물품 → n번 칸
        if slot >= SLOTS_PER_CLASS:
            raise PartError('NO_SLOT', f'{result_class} 칸이 가득 참 ({slot}/{SLOTS_PER_CLASS})')
        self.log.info(f'[place] 슬롯 배정: {result_class} → slot {slot}')
        return slot

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 로봇 동작으로 교체 (좌표는 로봇 base 기준 mm)
    # ------------------------------------------------------------------
    def pick_from_measure_zone(self):
        zone = self._measure_zone_xyz()   # 실제: movel([*zone, a, b, c]) → 그리퍼 닫기
        time.sleep(self._delay)

    def move_to_slot(self, result_class, slot):
        time.sleep(self._delay)

    def place_to_slot(self, result_class, slot):
        time.sleep(self._delay * 0.5)

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
    start(PlaceWork)


if __name__ == '__main__':
    main()
