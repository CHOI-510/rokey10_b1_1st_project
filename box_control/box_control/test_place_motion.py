#!/usr/bin/env python3
"""
test_place_motion.py — 실제 두산 동작을 넣은 '/place' 테스트 노드 (관제 테스트 전용)
================================================================
목적
  탐지·판별·적재 세 파트가 각자 connect_dsr()로 로봇에 '동시에 연결'해 둔 상태에서
  관제가 순서대로 Goal을 보낼 때 문제없이 움직이는지 확인한다 (PROGRESS 이슈 #8).
  test_scan_motion(/scan) + test_judge_motion(/judge) + test_place_motion(/place) + control

동작 (Goal 한 번당) — 파트마다 방향을 달리해서 눈으로 구분
  HOME → y +30mm → HOME → y -30mm → HOME (적재 흉내)
  수량은 측정하지 않고 노드 안 카운터(클래스별, 0부터)로 +1 해서 돌려준다 (v0.4 Result 형식)
  HOME에서 수평으로만 움직이므로 바닥에 닿지 않는다.

주의
  - 로봇이 실제로 움직인다. 작업공간을 비우고 펜던트 속도를 낮춘 뒤 실행할 것.
  - place_template / 적재 메인 코드와 같은 /place 이름을 쓰므로 동시에 띄우지 않는다.

실행 (bringup이 떠 있는 상태에서)
  ros2 run box_control test_place_motion --ros-args -p robot_setup:=on
================================================================
"""

from box_comm.robot import connect_dsr
from box_comm.common import position_from_count
from box_comm.place_interface import PartError, start

DR_MV_MOD_REL = 1
STEP_MM = 30.0


class PlaceMotionTest:
    def __init__(self, node):
        self.log = node.get_logger()
        dr, self.dsr_node = connect_dsr('place_motion_dsr')
        self.movej, self.movel = dr.movej, dr.movel
        self._speed = (dr.set_velj, dr.set_accj, dr.set_velx, dr.set_accx)
        self.count = 0
        self.stack = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.log.warn('[test_place_motion] ⚠ Goal을 받으면 로봇이 실제로 움직입니다 '
                      f'(HOME → y ±{STEP_MM:.0f}mm → HOME)')

    def run(self, result_class, report) -> int:
        self.count += 1
        set_velj, set_accj, set_velx, set_accx = self._speed
        home = self.poses['home_j']
        try:
            set_velj(30.0); set_accj(60.0); set_velx(100.0, 40.0); set_accx(300.0, 120.0)
            self.log.info(f'[test_place_motion] #{self.count} {result_class} 적재 흉내, movej HOME {home}')
            report('picking')
            self.movej(home)
            report.check()            # 관제가 정지를 요청했으면 다음 동작 전에 멈춤
            self.movel([0.0, STEP_MM, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report.check()            # 관제가 정지를 요청했으면 다음 동작 전에 멈춤
            report('moving')
            self.movel([0.0, -2 * STEP_MM, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report.check()            # 관제가 정지를 요청했으면 다음 동작 전에 멈춤
            report('placing')
            self.movel([0.0, STEP_MM, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report.check()            # 관제가 정지를 요청했으면 다음 동작 전에 멈춤
            self.movej(home)
            report.check()            # 관제가 정지를 요청했으면 다음 동작 전에 멈춤
        except PartError:
            raise                 # CANCELED(관제 정지) 등은 바꾸지 않고 그대로 올림
        except Exception as e:
            raise PartError('PLACE_FAILED', f'DSR 동작 실패: {type(e).__name__}: {e}')

        if result_class == 'U':          # 미분류: 놓기만 함, 수량은 관제가 셈 (v0.5)
            self.log.info(f'[test_place_motion] #{self.count} 동작 완료 → 미분류 투하 (0 반환)')
            return 0
        self.stack[result_class] += 1
        tier, level = position_from_count(self.stack[result_class])
        self.log.info(f'[test_place_motion] #{self.count} 동작 완료 → {result_class} '
                      f'{tier}단 {level}층 (수량 {self.stack[result_class]})')
        return self.stack[result_class]


def main():
    start(PlaceMotionTest)


if __name__ == '__main__':
    main()
