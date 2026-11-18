#!/usr/bin/env python3
"""
test_scan_motion.py — 실제 두산 동작을 넣은 '/scan' 테스트 노드 (관제 테스트 전용)
================================================================
목적
  팀원이 run() 안에 DSR_ROBOT2 동작(movej, movel)을 넣었을 때의 구조를 그대로 재현한다.
    - box_comm.scan_interface.start_algorithm으로 /scan Action 서버를 띄운다 (실제 파트 코드와 같은 구조)
    - Goal을 받으면 box_comm이 기본 TCP·툴을 확인·적용한 뒤 scan(poses, report)를 호출한다
    - 로봇은 관제가 넣어 주는 robot(box_comm.dsr_io.DsrIO)으로만 움직인다 → 정지 확인 자동 (11/18 변경)
  → 확인할 것: 'Executor is already spinning' 없이 로봇이 움직이는지,
               TCP 기준 좌표(get_current_posx)가 그리퍼 끝 기준으로 나오는지

동작 (Goal 한 번당)
  HOME(movej) → TCP 기준 현재 위치 출력 → 위로 50mm(movel 상대) → 아래로 50mm → HOME
  HOME에서 위로만 움직이므로 바닥에 닿지 않는다.

주의
  - 로봇이 실제로 움직인다. 작업공간을 비우고 실행할 것.
  - 탐지 노드(scan_template 또는 탐지 메인 코드)와 같은 /scan 이름을 쓰므로 동시에 띄우지 않는다.
  - gear_test_v2.py도 같이 돌리지 않는다.

파라미터
  box_count  (기본 0 = 제한 없음) N이면 N번 움직인 뒤 다음 Goal에 '물품 없음'(found=False)
             → 관제(control)와 같이 돌릴 때는 꼭 지정 (0이면 관제가 끝나지 않음)

실행 (bringup이 떠 있는 상태에서)
  ros2 run box_control test_scan_motion --ros-args -p robot_setup:=on
  관제와 함께 (파트 3개 동시 연결 시험): README 7장 '세 파트 동시 연결 시험'
  ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
  또는 ros2 run box_control robot_test  (자동 점검)
================================================================
"""

from box_comm.scan_interface import PartError, start_algorithm

LIFT_MM = 50.0           # HOME에서 위로 올렸다 내리는 거리

VEL_J, ACC_J = 30.0, 60.0        # 시험용 느린 속도 [deg/s, deg/s2]
VEL_L, ACC_L = 100.0, 300.0      # [mm/s, mm/s2]


def move_rel(robot, dx=0.0, dy=0.0, dz=0.0):
    """지금 TCP에서 BASE 기준으로 dx, dy, dz만큼 직선 이동 (DsrIO.movel = 정지 확인 + 도달 확인)"""
    p = robot.pose()
    robot.movel([p[0] + dx, p[1] + dy, p[2] + dz, *p[3:]], vel=VEL_L, acc=ACC_L)


class ScanMotionTest:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                  # box_comm.dsr_io.DsrIO (관제가 만들어 줌)
        self.count = 0
        self.box_count = int(node.declare_parameter('box_count', 0).value)
        self.log.warn('[test_scan_motion] ⚠ Goal을 받으면 로봇이 실제로 움직입니다 '
                      f'(HOME → 위로 {LIFT_MM:.0f}mm → 아래로 → HOME)')

    def scan(self, poses, report) -> bool:
        if self.box_count and self.count >= self.box_count:
            report('space_scan')
            self.log.info(f'[test_scan_motion] box_count={self.box_count} 도달 → 물품 없음 (움직이지 않음)')
            return False
        if self.robot is None:
            raise PartError('SCAN_FAILED', '로봇 연결 없음 (두산 패키지·bringup 확인)')
        self.count += 1
        r = self.robot
        report('space_scan')
        home = poses['home_j']                   # Goal로 받은 DB 값 (단독 실행이면 common.py 기본값)
        self.log.info(f'[test_scan_motion] #{self.count} movej HOME {home} | 측정존 {poses["measure_zone"]}')
        r.movej(home, vel=VEL_J, acc=ACC_J)
        self.log.info(f'[test_scan_motion] TCP 기준 현재 위치(posx): {r.pose()}')
        report('picking')
        move_rel(r, dz=LIFT_MM)
        report('moving')
        move_rel(r, dz=-LIFT_MM)
        report('placing')
        r.movej(home, vel=VEL_J, acc=ACC_J)
        # 실패하면 box_comm(algorithm.py)이 로봇을 멈추고 SCAN_FAILED로 보낸다 (CANCELED는 그대로)

        self.log.info(f'[test_scan_motion] #{self.count} 동작 완료')
        return True


def main():
    start_algorithm(ScanMotionTest)


if __name__ == '__main__':
    main()
