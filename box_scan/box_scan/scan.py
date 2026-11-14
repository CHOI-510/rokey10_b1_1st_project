#!/usr/bin/env python3
"""
scan.py — 탐지 파트 메인 (예시: fake 동작)
================================================================
담당자가 작성하는 파일. 통신은 scan_interface.py가 처리한다.

지킬 약속은 하나:
  run(report) -> bool
    - 물품을 측정존에 두고 홈 복귀까지 끝났으면 True
    - 작업공간에 물품이 없으면 False
    - 실패하면 raise PartError('GRASP_FAILED', '이유')  (등록된 코드만)
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출 (관제가 진행 상황을 추적)
      stage: 'space_scan' → 'picking' → 'moving' → 'placing'  (box_comm/common.py STAGES)

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다.
필요하면 모션·감지 함수를 다른 파일로 나눠 import해도 된다.

실행: ros2 run box_scan scan
      ros2 run box_scan scan --ros-args -p box_count:=5   (fake 박스 개수)
단독 테스트:
      ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
================================================================
"""

import time

from box_comm.scan_interface import PartError, start  # noqa: F401  (PartError는 실패 시 사용)

# ----------------------------------------------------------------------
# 고정 좌표 (HOME, 측정존) — 기준은 DB. 숫자를 직접 적지 않는다.
#   관제가 DB 값을 Goal에 실어 보내고, interface가 self.poses에 넣어 준다 (Goal마다 갱신).
#     self.poses['home_j']        HOME 관절 각도 [deg]      → movej(self.poses['home_j'])
#     self.poses['measure_zone']  측정존 x, y, z [mm, base]  (None = 아직 미설정)
#   혼자 send_goal로 테스트하면 Goal에 좌표가 없어서 common.py 기본값이 들어온다 (경고 로그).
#   좌표는 __init__에서 저장해 두지 말고, run() 안에서 매번 self.poses로 읽는다 (Goal마다 바뀔 수 있음).
#   측정존 자세(a, b, c)는 각 파트 동작에 맞게 정한다. 예: [*zone, 0.0, 180.0, 0.0]
# ----------------------------------------------------------------------


class ScanWork:
    def __init__(self, node):
        self.node = node                   # 로그, 파라미터 등에 사용
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시 ----
        # 반드시 box_comm의 connect_dsr()로 연결한다. (클래스 안에서 DR_init.__dsr__node = ...를
        # 직접 쓰면 파이썬 이름 변경 규칙 때문에 두산 쪽이 노드를 못 찾는다)
        # from box_comm.robot import connect_dsr      ← 파일 맨 위 import에 추가
        # self.dr, self.dsr_node = connect_dsr('scan_dsr')
        # 이후 self.dr.movej(self.poses['home_j']), self.dr.movel([...], mod=1) 처럼 사용
        # TCP·툴(set_tcp/set_tool)은 box_comm이 Goal마다 자동으로 맞추므로 부르지 않아도 된다.

        # [FAKE 전용] 실제 구현 시 삭제
        #   box_count: "작업공간에 박스가 N개 있다"고 가정 (인터페이스와 무관)
        #   fake_delay_sec: 로봇 동작 대신 기다리는 시간
        node.declare_parameter('box_count', 3)
        node.declare_parameter('fake_delay_sec', 1.0)
        self._remaining = node.get_parameter('box_count').value
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, report) -> bool:
        # 이번 Goal로 받은 고정 좌표 (관제가 DB 값을 보내 줌)
        self.log.info(f'[scan] 작업 좌표: HOME={self.poses["home_j"]}, '
                      f'측정존={self.poses["measure_zone"]}')
        # 1-1 작업공간 스캔, 1-2 물품 감지
        report('space_scan')
        if not self.detect_box():
            self.go_home()
            return False

        # 1-3 픽업 (실패 시 raise PartError('GRASP_FAILED', ...))
        report('picking')
        self.pick()

        # 1-4 측정존 이동
        report('moving')
        self.move_to_measure_zone()

        # 1-4 측정존에 내려놓기
        report('placing')
        self.place_on_measure_zone()

        self.go_home()
        return True

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 동작으로 교체
    # ------------------------------------------------------------------
    def detect_box(self) -> bool:
        """스캔 경로를 돌며 외력 감지 등으로 물품 존재 확인"""
        time.sleep(self._delay)
        if self._remaining <= 0:
            return False
        self._remaining -= 1
        return True

    def pick(self):
        time.sleep(self._delay)

    def move_to_measure_zone(self):
        zone = self._measure_zone_xyz()   # 실제: movel([*zone 위쪽 접근점, a, b, c])
        time.sleep(self._delay)

    def place_on_measure_zone(self):
        zone = self._measure_zone_xyz()   # 실제: movel([*zone, a, b, c]) → 그리퍼 열기
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
    start(ScanWork)


if __name__ == '__main__':
    main()
