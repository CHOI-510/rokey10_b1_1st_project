#!/usr/bin/env python3
"""
test_judge_motion.py — 실제 두산 동작을 넣은 '/judge' 테스트 노드 (관제 테스트 전용)
================================================================
목적
  탐지·판별·적재 세 파트가 각자 DsrIO(box_comm/dsr_io.py)로 로봇에 '동시에 연결'해 둔 상태에서
  관제가 순서대로 Goal을 보낼 때 문제없이 움직이는지 확인한다 (PROGRESS 이슈 #8).
  test_scan_motion(/scan) + test_judge_motion(/judge) + test_place_motion(/place) + control

동작 (Goal 한 번당) — 파트마다 방향을 달리해서 눈으로 구분
  HOME → x +30mm → HOME → x -30mm → HOME (측정 흉내), 판별 결과는 A→B→C→D 순서로 돌려줌
  HOME에서 수평으로만 움직이므로 바닥에 닿지 않는다.

주의
  - 로봇이 실제로 움직인다. 작업공간을 비우고 펜던트 속도를 낮춘 뒤 실행할 것.
  - judge_template / 판별 메인 코드와 같은 /judge 이름을 쓰므로 동시에 띄우지 않는다.

실행 (bringup이 떠 있는 상태에서)
  ros2 run box_control test_judge_motion --ros-args -p robot_setup:=on
================================================================
"""

from box_comm.judge_interface import PartError, start_algorithm

STEP_MM = 30.0

VEL_J, ACC_J = 30.0, 60.0        # 시험용 느린 속도 [deg/s, deg/s2]
VEL_L, ACC_L = 100.0, 300.0      # [mm/s, mm/s2]


def move_rel(robot, dx=0.0, dy=0.0, dz=0.0):
    """지금 TCP에서 BASE 기준으로 dx, dy, dz만큼 직선 이동 (DsrIO.movel = 정지 확인 + 도달 확인)"""
    p = robot.pose()
    robot.movel([p[0] + dx, p[1] + dy, p[2] + dz, *p[3:]], vel=VEL_L, acc=ACC_L)


class JudgeMotionTest:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                  # box_comm.dsr_io.DsrIO (관제가 만들어 줌)
        self.count = 0
        self.log.warn('[test_judge_motion] ⚠ Goal을 받으면 로봇이 실제로 움직입니다 '
                      f'(HOME → x ±{STEP_MM:.0f}mm → HOME)')

    def judge(self, specs, poses, report) -> str:
        if self.robot is None:
            raise PartError('MEASURE_FAILED', '로봇 연결 없음 (두산 패키지·bringup 확인)')
        self.count += 1
        r, home = self.robot, poses['home_j']
        self.log.info(f'[test_judge_motion] #{self.count} movej HOME {home}')
        report('measuring_length1')
        r.movej(home, vel=VEL_J, acc=ACC_J)
        report('measuring_length2')
        move_rel(r, dx=STEP_MM)
        report('measuring_length3')
        move_rel(r, dx=-2 * STEP_MM)
        report('measuring_weight')
        move_rel(r, dx=STEP_MM)
        report('judging')
        r.movej(home, vel=VEL_J, acc=ACC_J)
        # 실패하면 box_comm(algorithm.py)이 로봇을 멈추고 MEASURE_FAILED로 보낸다 (CANCELED는 그대로)

        ids = [s['class_id'] for s in specs] or ['A']
        result = ids[(self.count - 1) % len(ids)]
        self.log.info(f'[test_judge_motion] #{self.count} 동작 완료 → {result}')
        return result


def main():
    start_algorithm(JudgeMotionTest)


if __name__ == '__main__':
    main()
