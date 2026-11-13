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
    - report('단계명')은 진행 알림 (선택)

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다.

실행: ros2 run box_place place
단독 테스트:
  ros2 action send_goal /place box_interfaces/action/Place \
    "{result_class: 'B', inventory: {a: 1, b: 0, c: 2, d: 0}}" --feedback
================================================================
"""

import time

from box_comm.place_interface import PartError, start

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

        # 3-1 측정존 물품 픽업, 3-2 적재 공간 이동
        report('moving')
        self.pick_from_measure_zone()

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
        time.sleep(self._delay)

    def place_to_slot(self, result_class, slot):
        time.sleep(self._delay)

    def go_home(self):
        time.sleep(self._delay * 0.5)


def main():
    start(PlaceWork)


if __name__ == '__main__':
    main()
