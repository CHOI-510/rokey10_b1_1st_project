#!/usr/bin/env python3
"""
test_place_motion.py — 실제 두산 동작을 넣은 '/place' 테스트 노드 (관제 테스트 전용)
================================================================
목적
  탐지·판별·적재 세 파트가 각자 DsrIO(box_comm/dsr_io.py)로 로봇에 '동시에 연결'해 둔 상태에서
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

from box_comm.common import position_from_count
from box_comm.place_interface import PartError, start_algorithm

STEP_MM = 30.0

VEL_J, ACC_J = 30.0, 60.0        # 시험용 느린 속도 [deg/s, deg/s2]
VEL_L, ACC_L = 100.0, 300.0      # [mm/s, mm/s2]


def move_rel(robot, dx=0.0, dy=0.0, dz=0.0):
    """지금 TCP에서 BASE 기준으로 dx, dy, dz만큼 직선 이동 (DsrIO.movel = 정지 확인 + 도달 확인)"""
    p = robot.pose()
    robot.movel([p[0] + dx, p[1] + dy, p[2] + dz, *p[3:]], vel=VEL_L, acc=ACC_L)


class PlaceMotionTest:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                  # box_comm.dsr_io.DsrIO (관제가 만들어 줌)
        self.count = 0
        self.stack = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.log.warn('[test_place_motion] ⚠ Goal을 받으면 로봇이 실제로 움직입니다 '
                      f'(HOME → y ±{STEP_MM:.0f}mm → HOME)')

    def place(self, result_class, poses, report) -> int:
        if self.robot is None:
            raise PartError('PLACE_FAILED', '로봇 연결 없음 (두산 패키지·bringup 확인)')
        self.count += 1
        r, home = self.robot, poses['home_j']
        self.log.info(f'[test_place_motion] #{self.count} {result_class} 적재 흉내, movej HOME {home}')
        report('picking')
        r.movej(home, vel=VEL_J, acc=ACC_J)
        move_rel(r, dy=STEP_MM)
        report('moving')
        move_rel(r, dy=-2 * STEP_MM)
        report('placing')
        move_rel(r, dy=STEP_MM)
        r.movej(home, vel=VEL_J, acc=ACC_J)
        # 실패하면 box_comm(algorithm.py)이 로봇을 멈추고 PLACE_FAILED로 보낸다 (CANCELED는 그대로)

        if result_class == 'U':          # 미분류: 놓기만 함, 수량은 관제가 셈 (v0.5)
            self.log.info(f'[test_place_motion] #{self.count} 동작 완료 → 미분류 투하 (0 반환)')
            return 0
        self.stack[result_class] += 1
        tier, level = position_from_count(self.stack[result_class])
        self.log.info(f'[test_place_motion] #{self.count} 동작 완료 → {result_class} '
                      f'{tier}단 {level}층 (수량 {self.stack[result_class]})')
        return self.stack[result_class]


def main():
    start_algorithm(PlaceMotionTest)


if __name__ == '__main__':
    main()
