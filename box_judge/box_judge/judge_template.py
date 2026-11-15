#!/usr/bin/env python3
"""
judge_template.py — 판별 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(송현석)은 이 파일을 참고해 자기 메인 코드(예: judge.py)를 만든다.
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_judge/box_judge/judge_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_judge judge_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
아래는 메인 코드가 지킬 약속과 예시다. 통신은 judge_interface.py가 처리한다.

지킬 약속은 하나:
  run(specs, report) -> str
    - specs: [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
               'height_mm': 40.0, 'weight_g': 100.0}, ...]   (DB의 물품값)
    - 측정존 물품을 측정하고 'A' / 'B' / 'C' / 'D' 중 하나를 반환
    - 실패하면 raise PartError('MEASURE_FAILED', '이유')  (등록된 코드만)
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출 (관제가 진행 상황을 추적)
      stage: 'measuring_length1' → 'measuring_length2' → 'measuring_length3'
             → 'measuring_weight' → ('measuring_materiality', 옵션) → 'judging'
      (box_comm/common.py STAGES)

[FAKE] 표시된 부분을 실제 측정으로 바꾸면 된다.
classify()는 예시 로직 — 분류 기준·오차 범위가 정해지면 수정.

실행: ros2 run box_judge judge_template
단독 테스트:
  ros2 action send_goal /judge box_interfaces/action/Judge \
    "{specs: [{class_id: 'A', width_mm: 40.0, depth_mm: 40.0, height_mm: 40.0, weight_g: 100.0},
              {class_id: 'B', width_mm: 60.0, depth_mm: 40.0, height_mm: 30.0, weight_g: 150.0}]}" --feedback
================================================================
"""

import random
import time

from box_comm.judge_interface import PartError, start  # noqa: F401

# ----------------------------------------------------------------------
# 고정 좌표 (HOME, 측정존) — 기준은 DB. 숫자를 직접 적지 않는다.
#   관제가 DB 값을 Goal에 실어 보내고, interface가 self.poses에 넣어 준다 (Goal마다 갱신).
#     self.poses['home_j']        HOME 관절 각도 [deg]      → movej(self.poses['home_j'])
#     self.poses['measure_zone']  측정존 x, y, z [mm, base]  (None = 아직 미설정)
#   혼자 send_goal로 테스트하면 Goal에 좌표가 없어서 common.py 기본값이 들어온다 (경고 로그).
#   좌표는 __init__에서 저장해 두지 말고, run() 안에서 매번 self.poses로 읽는다 (Goal마다 바뀔 수 있음).
#   측정존 자세(a, b, c)는 각 파트 동작에 맞게 정한다. 예: [*zone, 0.0, 180.0, 0.0]
# ----------------------------------------------------------------------


class JudgeWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # ---- 두산 로봇 사용 시 ----
        # 반드시 box_comm의 connect_dsr()로 연결한다. (클래스 안에서 DR_init.__dsr__node = ...를
        # 직접 쓰면 파이썬 이름 변경 규칙 때문에 두산 쪽이 노드를 못 찾는다)
        # from box_comm.robot import connect_dsr      ← 파일 맨 위 import에 추가
        # self.dr, self.dsr_node = connect_dsr('judge_dsr')
        # 이후 self.dr.movej(self.poses['home_j']), self.dr.movel([...], mod=1) 처럼 사용
        # TCP·툴(set_tcp/set_tool)은 box_comm이 Goal마다 자동으로 맞추므로 부르지 않아도 된다.

        # [FAKE 전용] 실제 구현 시 삭제
        node.declare_parameter('fake_delay_sec', 1.0)
        self._delay = node.get_parameter('fake_delay_sec').value

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def run(self, specs, report) -> str:
        # 이번 Goal로 받은 고정 좌표 (관제가 DB 값을 보내 줌)
        self.log.info(f'[judge] 작업 좌표: HOME={self.poses["home_j"]}, '
                      f'측정존={self.poses["measure_zone"]}')
        # 2-1 길이 측정 3회 (물품 방향에 따라 가로/세로/높이가 달라질 수 있어 번호로 구분)
        lengths = []
        for i in (1, 2, 3):
            report(f'measuring_length{i}')
            lengths.append(self.measure_length(i, specs))

        # 2-2 무게 측정
        report('measuring_weight')
        weight = self.measure_weight(specs)

        # 2-3 강성 추정 (옵션) — 구현하면 아래 주석 해제
        # report('measuring_materiality')
        # stiffness = self.measure_materiality()

        self.log.info(f'[judge] 측정값 L1={lengths[0]:.1f} L2={lengths[1]:.1f} '
                      f'L3={lengths[2]:.1f} mm, {weight:.1f} g')

        # 2-4 판별
        report('judging')
        result_class = self.classify(specs, lengths, weight)

        self.go_home()
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
    # [FAKE] 아래를 실제 측정으로 교체
    # ------------------------------------------------------------------
    def measure_length(self, i, specs):
        """그리퍼로 i번째 길이 측정 (fake: 기준 하나를 골라 ±3% 노이즈, 순서는 섞음)"""
        zone = self._measure_zone_xyz()   # 실제: 측정존(zone) 기준으로 접근해서 그리퍼로 측정
        time.sleep(self._delay * 0.5)
        if i == 1:
            self._fake_target = random.choice(specs)
            t = self._fake_target
            dims = [t['width_mm'], t['depth_mm'], t['height_mm']]
            random.shuffle(dims)                 # 물품 방향이 매번 다르다고 가정
            self._fake_dims = dims
        return self._fake_dims[i - 1] * random.uniform(0.97, 1.03)

    def measure_weight(self, specs):
        """토크센서로 무게 측정 (fake: ±5% 노이즈)"""
        time.sleep(self._delay * 0.5)
        return self._fake_target['weight_g'] * random.uniform(0.95, 1.05)

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
    start(JudgeWork)


if __name__ == '__main__':
    main()
