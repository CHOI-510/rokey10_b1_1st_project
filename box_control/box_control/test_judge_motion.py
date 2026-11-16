#!/usr/bin/env python3
"""
test_judge_motion.py — 실제 두산 동작을 넣은 '/judge' 테스트 노드 (관제 테스트 전용)
================================================================
목적
  탐지·판별·적재 세 파트가 각자 connect_dsr()로 로봇에 '동시에 연결'해 둔 상태에서
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

from box_comm.robot import connect_dsr
from box_comm.judge_interface import PartError, start

DR_MV_MOD_REL = 1
STEP_MM = 30.0


class JudgeMotionTest:
    def __init__(self, node):
        self.log = node.get_logger()
        dr, self.dsr_node = connect_dsr('judge_motion_dsr')
        self.movej, self.movel = dr.movej, dr.movel
        self._speed = (dr.set_velj, dr.set_accj, dr.set_velx, dr.set_accx)
        self.count = 0
        self.log.warn('[test_judge_motion] ⚠ Goal을 받으면 로봇이 실제로 움직입니다 '
                      f'(HOME → x ±{STEP_MM:.0f}mm → HOME)')

    def run(self, specs, report) -> str:
        self.count += 1
        set_velj, set_accj, set_velx, set_accx = self._speed
        home = self.poses['home_j']
        try:
            set_velj(30.0); set_accj(60.0); set_velx(100.0, 40.0); set_accx(300.0, 120.0)
            self.log.info(f'[test_judge_motion] #{self.count} movej HOME {home}')
            report('measuring_length1')
            self.movej(home)
            report('measuring_length2')
            self.movel([STEP_MM, 0.0, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report('measuring_length3')
            self.movel([-2 * STEP_MM, 0.0, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report('measuring_weight')
            self.movel([STEP_MM, 0.0, 0.0, 0.0, 0.0, 0.0], mod=DR_MV_MOD_REL)
            report('judging')
            self.movej(home)
        except Exception as e:
            raise PartError('MEASURE_FAILED', f'DSR 동작 실패: {type(e).__name__}: {e}')

        ids = [s['class_id'] for s in specs] or ['A']
        result = ids[(self.count - 1) % len(ids)]
        self.log.info(f'[test_judge_motion] #{self.count} 동작 완료 → {result}')
        return result


def main():
    start(JudgeMotionTest)


if __name__ == '__main__':
    main()
