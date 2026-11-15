#!/usr/bin/env python3
"""
place_template.py — 적재 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(정수현)은 이 파일을 참고해 자기 메인 코드(예: place.py)를 만든다.
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_place/box_place/place_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_place place_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
아래는 메인 코드가 지킬 약속과 예시다. 통신은 place_interface.py가 처리한다.

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

실행: ros2 run box_place place_template
단독 테스트:
  ros2 action send_goal /place box_interfaces/action/Place \
    "{result_class: 'B', inventory: {a: 1, b: 0, c: 2, d: 0}}" --feedback
================================================================
"""

import time

from box_comm.place_interface import PartError, start

# ----------------------------------------------------------------------
# 고정 좌표 (HOME, 측정존) — 기준은 DB. 숫자를 직접 적지 않는다.
#   관제가 DB 값을 Goal에 실어 보내고, interface가 self.poses에 넣어 준다 (Goal마다 갱신).
#     self.poses['home_j']        HOME 관절 각도 [deg]      → movej(self.poses['home_j'])
#     self.poses['measure_zone']  측정존 x, y, z [mm, base]  (None = 아직 미설정)
#   혼자 send_goal로 테스트하면 Goal에 좌표가 없어서 common.py 기본값이 들어온다 (경고 로그).
#   좌표는 __init__에서 저장해 두지 말고, run() 안에서 매번 self.poses로 읽는다 (Goal마다 바뀔 수 있음).
#   측정존 자세(a, b, c)는 각 파트 동작에 맞게 정한다. 예: [*zone, 0.0, 180.0, 0.0]
# ----------------------------------------------------------------------

SLOTS_PER_CLASS = 4      # 예시: 클래스당 적재 칸 수 — 실제 레이아웃에 맞게 수정


class PlaceWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시 ----
        # 반드시 box_comm의 connect_dsr()로 연결한다. (클래스 안에서 DR_init.__dsr__node = ...를
        # 직접 쓰면 파이썬 이름 변경 규칙 때문에 두산 쪽이 노드를 못 찾는다)
        # from box_comm.robot import connect_dsr      ← 파일 맨 위 import에 추가
        # self.dr, self.dsr_node = connect_dsr('place_dsr')
        # 이후 self.dr.movej(self.poses['home_j']), self.dr.movel([...], mod=1) 처럼 사용
        # TCP·툴(set_tcp/set_tool)은 box_comm이 Goal마다 자동으로 맞추므로 부르지 않아도 된다.

        # [FAKE 전용] 실제 구현 시 삭제
        node.declare_parameter('fake_delay_sec', 1.0)
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, result_class, inventory, report) -> dict:
        # 이번 Goal로 받은 고정 좌표 (관제가 DB 값을 보내 줌)
        self.log.info(f'[place] 작업 좌표: HOME={self.poses["home_j"]}, '
                      f'측정존={self.poses["measure_zone"]}')
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
        # 실제: self.dr.movej(self.poses['home_j'])
        time.sleep(self._delay * 0.5)

    def _measure_zone_xyz(self):
        """측정존 x, y, z (Goal로 받은 DB 값). 미설정이면 실제 동작에서는 움직이면 안 된다."""
        zone = self.poses.get('measure_zone')
        if zone is None or all(abs(v) < 1e-6 for v in zone):     # (0, 0, 0)도 미설정으로 본다
            if not getattr(self, '_zone_warned', False):
                self.log.warn('측정존 좌표가 아직 미설정 — '
                              '[FAKE]에서는 넘어가지만 실제 동작에서는 움직이지 않도록 처리할 것')
                self._zone_warned = True
            return None
        return list(zone)


def main():
    start(PlaceWork)


if __name__ == '__main__':
    main()
