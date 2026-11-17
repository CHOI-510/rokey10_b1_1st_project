#!/usr/bin/env python3
"""
judge_template.py — 판별 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(송현석)은 이 파일을 참고해 자기 메인 코드를 만든다.
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_judge/box_judge/judge_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_judge judge_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
권장 파일 구성 (적재와 같은 방식, 11/17)
  box_judge/judge_algorithm.py   JudgeAlgorithm 클래스 — 측정·판별 로직 전부 (담당자 소유, 관제 코드 없음)
  box_judge/judge_main.py        실행 진입점 한 줄: start_algorithm(JudgeAlgorithm)  + setup.py entry_points에 추가
  통신·정지·실패 처리는 box_comm(관제)이 맡는다: judge_interface.py, algorithm.py, dsr_io.py

지킬 약속:
  JudgeAlgorithm(node, robot)
    - robot: 관제가 만들어 넣어 주는 로봇 객체 (box_comm/dsr_io.py DsrIO). 두산 패키지가 없으면 None
      로봇은 robot으로만 움직인다 → 명령 전에 관제 정지 요청을 자동으로 확인한다
        robot.movel([x, y, z, a, b, c], vel=60, acc=100)   직선 이동 + 도달 확인 (BASE 기준)
        robot.movej(poses['home_j'], vel=30, acc=30)        관절 이동
        robot.grip(opened=True/False, wait_s=1.5)            RG2 열기/닫기
        robot.pose() / robot.force_z()                       현재 TCP / 툴 외력 Fz (무게 측정, 진행 알림 포함)
        robot.compliance_on(...) / robot.force_on(...) / robot.stop()   힘 제어 (강성 추정 등)
      필요한 명령이 없으면 관제에게 요청 (예: 그리퍼 폭 읽기). DSR_ROBOT2를 직접 부르면 정지 확인이 빠진다
  judge(specs, poses, report) -> str
    - specs: [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
               'height_mm': 40.0, 'weight_g': 100.0}, ...]   (DB의 물품값)
    - poses: {'home_j': [6], 'measure_zone': [x, y, z], ...}  — Goal로 받은 DB 값
    - 측정존 물품을 측정하고 'A' / 'B' / 'C' / 'D' 중 하나를 반환
    - 어느 기준에도 맞지 않으면 'U'(미분류)를 반환 → 적재가 미분류 위치에 놓고, 관제가 미분류 +1
      (from box_comm.common import UNCLASSIFIED)  기준을 얼마나 벗어나면 U인지는 판별이 정함
    - 실패하면 raise PartError('MEASURE_FAILED', '이유') — 그 밖의 예외도 box_comm이 로봇을 멈추고 MEASURE_FAILED로 보냄
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출
      stage: 'measuring_length1' → 'measuring_length2' → 'measuring_length3'
             → 'measuring_weight' → ('measuring_materiality', 옵션) → 'judging'  (box_comm/common.py STAGES)
    - 기록할 일이 있으면 report.log('메시지', 'INFO'/'WARN'/'ERROR') — 관제가 /event_log로 DB·UI에 넘김
    - 정지: report.check()를 부를 필요 없다 (robot 명령이 대신 확인). PartError('CANCELED')는 잡지 말 것
    - TCP·툴은 box_comm이 Goal마다 맞춰 준다

[FAKE] 표시된 부분을 실제 측정으로 바꾸면 된다. classify()는 예시 로직 — 분류 기준·오차 범위가 정해지면 수정.

실행: ros2 run box_judge judge_template
단독 테스트:
  ros2 action send_goal /judge box_interfaces/action/Judge \\
    "{specs: [{class_id: 'A', width_mm: 40.0, depth_mm: 40.0, height_mm: 40.0, weight_g: 100.0},
              {class_id: 'B', width_mm: 60.0, depth_mm: 40.0, height_mm: 30.0, weight_g: 150.0}]}" --feedback
================================================================
"""

import random
import time

from box_comm.judge_interface import PartError, start_algorithm  # noqa: F401

SPEED = 60.0          # 직선 이동 속도 [mm/s], 가속 [mm/s²] — 판별 파트 값으로 조정
ACC = 100.0
HOME_VEL = 30.0       # 관절 이동 속도 [deg/s]


class JudgeAlgorithm:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                 # 관제가 넣어 줌 (None이면 두산 패키지 없이 FAKE로만 실행)

        # [FAKE 전용] 실제 구현 시 삭제
        node.declare_parameter('fake_delay_sec', 1.0)
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def judge(self, specs, poses, report) -> str:
        home = poses['home_j']
        zone = self._measure_zone(poses)
        self.log.info(f'[judge] 작업 좌표: HOME={home}, 측정존={zone}')

        # 2-1 길이 측정 3회 (물품 방향에 따라 가로/세로/높이가 달라질 수 있어 번호로 구분)
        lengths = []
        for i in (1, 2, 3):
            report(f'measuring_length{i}')
            lengths.append(self.measure_length(i, specs, zone))

        # 2-2 무게 측정
        report('measuring_weight')
        weight = self.measure_weight(specs)

        # 2-3 강성 추정 (옵션) — 구현하면 아래 주석 해제
        # report('measuring_materiality')
        # stiffness = self.measure_materiality()

        self.log.info(f'[judge] 측정값 L1={lengths[0]:.1f} L2={lengths[1]:.1f} '
                      f'L3={lengths[2]:.1f} mm, {weight:.1f} g')
        report.log(f'측정값 {lengths[0]:.1f} x {lengths[1]:.1f} x {lengths[2]:.1f} mm, '
                   f'{weight:.1f} g')                       # 이벤트 로그 예시 → DB·UI

        # 2-4 판별
        report('judging')
        result_class = self.classify(specs, lengths, weight)
        report.log(f'판별 결과 {result_class}')
        # 위상 정렬을 넣으면: report.log('정렬 완료 (90° 회전)')

        self.go_home(home)
        return result_class

    # ------------------------------------------------------------------
    # 판별 로직 (예시) — 기준이 정해지면 수정
    # ------------------------------------------------------------------
    def classify(self, specs, lengths, weight) -> str:
        """각 기준과의 상대 오차 합이 가장 작은 class_id 선택

        측정 순서가 물품 방향에 따라 달라질 수 있으므로, 측정한 길이 3개와
        기준의 W·D·H를 각각 크기순으로 정렬해서 비교한다.
        """
        def rel(a, b):
            return abs(a - b) / b if b > 0 else 0.0

        measured = sorted(lengths)

        def score(s):
            ref = sorted([s['width_mm'], s['depth_mm'], s['height_mm']])
            return sum(rel(m, r) for m, r in zip(measured, ref)) + rel(weight, s['weight_g'])

        return min(specs, key=score)['class_id']

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 측정으로 교체 (주석 = 실제 예시)
    # ------------------------------------------------------------------
    def measure_length(self, i, specs, zone):
        """그리퍼로 i번째 길이 측정 (fake: 기준 하나를 골라 ±3% 노이즈, 순서는 섞음)
        실제 예: self.robot.movel([*zone 위 접근점, a, b, c], vel=SPEED, acc=ACC) → 그리퍼로 물고 폭 읽기"""
        time.sleep(self._delay * 0.5)
        if i == 1:
            self._fake_target = random.choice(specs)
            t = self._fake_target
            dims = [t['width_mm'], t['depth_mm'], t['height_mm']]
            random.shuffle(dims)                 # 물품 방향이 매번 다르다고 가정
            self._fake_dims = dims
        return self._fake_dims[i - 1] * random.uniform(0.97, 1.03)

    def measure_weight(self, specs):
        """툴 외력으로 무게 측정 (fake: ±5% 노이즈)
        실제 예: 집기 전 f0 = self.robot.force_z(), 들고 나서 f1 = self.robot.force_z()
                 weight_g = abs(f1 - f0) / 9.81 * 1000"""
        time.sleep(self._delay * 0.5)
        return self._fake_target['weight_g'] * random.uniform(0.95, 1.05)

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
    start_algorithm(JudgeAlgorithm)


if __name__ == '__main__':
    main()
