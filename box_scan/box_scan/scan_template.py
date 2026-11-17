#!/usr/bin/env python3
"""
scan_template.py — 탐지 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(김창균)은 이 파일을 참고해 자기 메인 코드를 만든다.
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_scan/box_scan/scan_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_scan scan_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
권장 파일 구성 (적재와 같은 방식, 11/17)
  box_scan/scan_algorithm.py   ScanAlgorithm 클래스 — 탐지 로직 전부 (담당자 소유, 관제 코드 없음)
  box_scan/scan_main.py        실행 진입점 한 줄: start_algorithm(ScanAlgorithm)  + setup.py entry_points에 추가
  통신·정지·실패 처리는 box_comm(관제)이 맡는다: scan_interface.py, algorithm.py, dsr_io.py

지킬 약속:
  ScanAlgorithm(node, robot)
    - robot: 관제가 만들어 넣어 주는 로봇 객체 (box_comm/dsr_io.py DsrIO). 두산 패키지가 없으면 None
      로봇은 robot으로만 움직인다 → 명령 전에 관제 정지 요청을 자동으로 확인한다
        robot.movel([x, y, z, a, b, c], vel=60, acc=100)   직선 이동 + 도달 확인 (BASE 기준)
        robot.movej(poses['home_j'], vel=30, acc=30)        관절 이동
        robot.grip(opened=True/False)                        RG2 기본 열기/닫기 (Modbus, 11/17)
        robot.grip_open(width_mm, force_n)                   지정 폭으로 열기
        robot.grip_close(force_n) -> (grabbed, width_mm, raw_mm)   닫기 + 잡혔는지·폭 (못 잡아도 예외 없음)
          못 잡으면(grabbed=False) 파트 방식대로 다시 집기 → common.GRIP_RETRY(3)번 안에 안 되면
          raise PartError('GRASP_FAILED') → 관제가 공간 스캔부터 다시 (판별·적재는 관제가 측정존을 먼저 비움)
        robot.pose() / robot.force_z()                       현재 TCP / 툴 외력 Fz (force_z는 진행 알림 포함)
        robot.compliance_on(...) / robot.force_on(...) / robot.stop()   힘 제어 (외력 감지 스캔 등)
      필요한 명령이 없으면 관제에게 요청 (DSR_ROBOT2를 직접 부르면 정지 확인이 빠진다)
  scan(poses, report) -> bool
    - poses: {'home_j': [6], 'measure_zone': [x, y, z], ...}  — Goal로 받은 DB 값 (숫자를 코드에 적지 않는다)
    - 물품을 측정존에 두고 홈 복귀까지 끝났으면 True / 작업공간에 물품이 없으면 False
    - 못 집으면 다시 집기 (예: 그리퍼를 돌려서 — 실제 코드 scan_algorithm.py는 0/45/90°), GRIP_RETRY번 안에 안 되면 raise PartError('GRASP_FAILED')
      → 관제가 공간 스캔부터 다시. 그 밖의 예외는 box_comm이 로봇을 멈추고 SCAN_FAILED로 보냄 (작업 종료)
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출 (관제가 진행 상황을 추적)
      stage: 'space_scan' → 'picking' → 'moving' → 'placing'  (box_comm/common.py STAGES)
    - 기록할 일이 있으면 report.log('메시지', 'INFO'/'WARN'/'ERROR') — 관제가 /event_log로 DB·UI에 넘김
      예: report.log('집기 실패, 재시도 2/3', 'WARN'), report.log('물품 감지 x=420, y=-80')
    - 정지: report.check()를 부를 필요 없다 (robot 명령이 대신 확인). PartError('CANCELED')는 잡지 말 것
    - TCP·툴은 box_comm이 Goal마다 맞춰 준다

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다. 실제 예시는 주석의 robot.* 코드 참고.

실행: ros2 run box_scan scan_template
      ros2 run box_scan scan_template --ros-args -p box_count:=5   (fake 박스 개수)
단독 테스트:
      ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
================================================================
"""

import time

from box_comm.scan_interface import PartError, start_algorithm  # noqa: F401  (PartError는 실패 시 사용)

SPEED = 60.0          # 직선 이동 속도 [mm/s], 가속 [mm/s²] — 탐지 파트 값으로 조정
ACC = 100.0
HOME_VEL = 30.0       # 관절 이동 속도 [deg/s]
GRIP_WAIT = 1.5       # 그리퍼 동작 대기 [s]
PLACE_RPY = [0.0, 180.0, 0.0]   # 측정존에 놓을 때 자세 a, b, c [deg] — 실제 그리퍼 방향에 맞게


class ScanAlgorithm:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                 # 관제가 넣어 줌 (None이면 두산 패키지 없이 FAKE로만 실행)

        # [FAKE 전용] 실제 구현 시 삭제
        #   box_count: "작업공간에 박스가 N개 있다"고 가정, fake_delay_sec: 로봇 동작 대신 기다리는 시간
        node.declare_parameter('box_count', 3)
        node.declare_parameter('fake_delay_sec', 1.0)
        self._remaining = node.get_parameter('box_count').value
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def scan(self, poses, report) -> bool:
        home = poses['home_j']
        zone = self._measure_zone(poses)
        self.log.info(f'[scan] 작업 좌표: HOME={home}, 측정존={zone}')

        # 1-1 작업공간 스캔, 1-2 물품 감지
        report('space_scan')
        target = self.detect_box()
        if target is None:
            report.log('작업공간에 물품 없음')               # 이벤트 로그 예시 → DB·UI
            self.go_home(home)
            return False
        report.log(f'물품 감지 {target}')

        # 1-3 픽업 (실패 시 raise PartError('GRASP_FAILED', ...))
        report('picking')
        self.pick(target)     # 재시도를 넣으면 실패할 때마다 report.log(f'집기 실패, 재시도 {n}/{max}', 'WARN')

        # 1-4 측정존 이동
        report('moving')
        self.move_above_zone(zone)

        # 1-4 측정존에 내려놓기
        report('placing')
        self.place_on_zone(zone)

        self.go_home(home)
        return True

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 동작으로 교체 (주석 = 실제 예시)
    # ------------------------------------------------------------------
    def detect_box(self):
        """스캔 경로를 돌며 외력 감지 등으로 물품 존재 확인 → 물품 위치(또는 None)
        실제 예:  for p in SCAN_PATH: self.robot.movel(p, vel=SPEED, acc=ACC)
                  ... if abs(self.robot.force_z() - base) > 3.0: return self.robot.pose()"""
        time.sleep(self._delay)
        if self._remaining <= 0:
            return None
        self._remaining -= 1
        return 'fake'

    def pick(self, target):
        # 실제: self.robot.grip(True, GRIP_WAIT); self.robot.movel(위치, ...); self.robot.grip(False, GRIP_WAIT)
        time.sleep(self._delay)

    def move_above_zone(self, zone):
        # 실제: self.robot.movel([zone[0], zone[1], zone[2] + 100, *PLACE_RPY], vel=SPEED, acc=ACC)
        time.sleep(self._delay)

    def place_on_zone(self, zone):
        # 실제: self.robot.movel([*zone, *PLACE_RPY], vel=SPEED, acc=ACC); self.robot.grip(True, GRIP_WAIT)
        time.sleep(self._delay * 0.5)

    def go_home(self, home):
        # 실제: self.robot.movej(home, vel=HOME_VEL, acc=HOME_VEL)
        time.sleep(self._delay * 0.5)

    def _measure_zone(self, poses):
        """측정존 x, y, z (Goal로 받은 DB 값). 미설정이면 실제 동작에서는 움직이면 안 된다."""
        zone = poses.get('measure_zone')
        if zone is None or all(abs(v) < 1e-6 for v in zone):     # (0, 0, 0)도 미설정으로 본다
            if not getattr(self, '_zone_warned', False):
                self.log.warn('측정존 좌표가 아직 미설정 — '
                              '[FAKE]에서는 넘어가지만 실제 동작에서는 raise로 멈출 것')
                self._zone_warned = True
            return None
        return list(zone)


def main():
    start_algorithm(ScanAlgorithm)


if __name__ == '__main__':
    main()
