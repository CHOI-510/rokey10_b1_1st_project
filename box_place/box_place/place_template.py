#!/usr/bin/env python3
"""
place_template.py — 적재 템플릿 (관제 관리, 참고용)
================================================================
  이 파일은 직접 수정하지 않는다. 담당(정수현)은 이 파일을 참고해 자기 메인 코드를 만든다.
  실제 로봇 구현: box_place/place_algorithm.py (+ config/place_config.json, 진입점 place_main.py)
    이 템플릿과 같은 약속(PlaceAlgorithm.place)을 쓴다. 통신·정지·실패 처리는 box_comm(관제)
  관제가 인터페이스·공통 규칙을 바꾸면 이 파일을 업데이트한다.
  pull 후 바뀐 부분 확인:  git log -p -- box_place/box_place/place_template.py
  → 바뀐 내용을 자기 메인 코드에 직접 반영한다.
  실행 비교: ros2 run box_place place_template  (메인 코드 노드와 동시에 켜지 말 것 — 같은 서버 이름이라 결과가 섞임)
================================================================
적재 규칙 (v0.4, box_comm/common.py STACK)
  - 구역 하나에 같은 클래스만. 구역의 '1단' x, y는 DB 값 → poses['place_zones'][class]
  - 1단(첫 자리)에 1층부터 STACK['levels']층(현재 3)까지, 다 차면 다음 자리(2단)에 다시 1층부터
    다음 자리 위치 = 1단 x + (단-1) * TIER_PITCH_X, y는 같음 (현재 −x 방향 80mm, place_config의 slot_dx_mm)
  - 놓을 자리는 '재고 숫자'로 정하지 않고 실제로 확인한다 (매번 1단부터):
      1단 위로 가서 박스를 쥔 채 내려감 → 더 내려가지 않는 높이(z)에서 층 계산
        z가 1~3층 높이  → 그 자리에 놓음
        z가 4층 높이    → 1단이 꽉 참 → 놓지 않고 올라가서 2단에서 다시 시도
      놓은 (단, 층)으로 수량 계산: (단-1)*3 + 층   예: 2단 2층 → 5개
  - 관제에는 이 수량만 보낸다. 단·층·z는 이 파트 안에서만 쓰고 로그로 남긴다.
  - 이 수량이 관제 → DB → UI로 올라가는 '실제 재고'다. 관제가 기대값(+1)과 비교·기록한다.
  - 한계: 앞 단은 모두 찼다고 보고 센다. 안쪽 단에서 박스를 빼면 그 빈자리부터 다시 채우며
          수량은 '놓은 위치' 기준으로 나온다 (실제보다 적게 나올 수 있음).

지킬 약속:
  PlaceAlgorithm(node, robot)
    - robot: 관제가 만들어 넣어 주는 로봇 객체 (box_comm/dsr_io.py DsrIO). 두산 패키지가 없으면 None
      로봇은 robot으로만 움직인다 → 명령 전에 관제 정지 요청을 자동으로 확인한다
        robot.movel([x, y, z, a, b, c], vel=60, acc=100) / robot.movej(home, vel, acc)
        robot.grip(opened, wait_s) / robot.pose() / robot.force_z()  (force_z는 진행 알림 포함)
        robot.compliance_on(stx) / robot.force_on(fd, dir) / robot.stop()   힘 제어 하강
      필요한 명령이 없으면 관제에게 요청 (DSR_ROBOT2를 직접 부르면 정지 확인이 빠진다)
  place(result_class, poses, report) -> int
    - result_class: 'A' / 'B' / 'C' / 'D' / 'U'
    - poses: {'home_j': [6], 'measure_zone': [x, y, z], 'place_zones': {'A': [x, y], ...}} — Goal의 DB 값
    - U(미분류, v0.5): 미분류 투하 위치에 놓기만 하고 0을 반환 (쌓지 않음, 수량은 관제가 셈)
      실제 코드: place_algorithm.py의 _place_unclassified (위치는 place_config.json unclassified)
    - 반환: 적재 후 그 구역의 수량 (놓은 단·층으로 계산 — count_from_position(단, 층))
    - 단 간격·접근 높이·바닥 z·박스 높이는 이 파일 안에서만 처리 (관제·DB는 모름)
    - 모든 단이 차면 raise PartError('NO_SLOT', ...), 그 밖의 실패는 PartError('PLACE_FAILED', ...)
      또는 아무 예외 (box_comm이 로봇을 멈추고 PLACE_FAILED로 보냄)
    - 단계가 바뀔 때마다 report(stage)를 반드시 호출
    - 기록할 일이 있으면 report.log('메시지', 'INFO'/'WARN'/'ERROR') — 관제가 /event_log로 DB·UI에 넘김
      예: report.log('집기 실패, 재시도 2/3', 'WARN'), report.log('정렬 완료 (90° 회전)')
      진행만 알리는 report(stage)는 로그로 올라가지 않는다
    - 정지: report.check()를 부를 필요 없다 (robot 명령이 대신 확인). PartError('CANCELED')는 잡지 말 것
      stage: 'picking' → 'moving' → 'placing'  (box_comm/common.py STAGES)

[FAKE] 표시된 부분을 실제 로봇 동작으로 바꾸면 된다.
  FAKE에서는 구역별로 쌓인 개수를 메모리에 흉내 낸다 (fake_start 파라미터로 처음 개수 지정).

실행: ros2 run box_place place_template
      ros2 run box_place place_template --ros-args -p fake_start:="[0, 3, 4, 0]"   (A~D 처음 개수)
단독 테스트 (Goal에 poses를 넣지 않으면 common.py 기본값 → 구역 좌표 None이라 PLACE_FAILED):
  ros2 action send_goal /place box_interfaces/action/Place \\
    "{result_class: 'B', poses: {available: true, home_j: [0,0,90,0,90,0], measure_zone: [400,0,100],
      place_zones: [{class_id: 'B', x: 300.0, y: -200.0}]}}" --feedback
================================================================
"""

import time

from box_comm.place_interface import PartError, STACK, count_from_position, start_algorithm

# ----------------------------------------------------------------------
# 고정 좌표 (HOME, 측정존, 적재 구역 1단 x,y) — 기준은 DB. 숫자를 직접 적지 않는다.
#   관제가 DB 값을 Goal에 실어 보내고, place()의 poses 인자로 들어온다 (Goal마다 갱신).
#     poses['home_j']              HOME 관절 각도 [deg]
#     poses['measure_zone']        측정존 x, y, z [mm, base]   (None = 미설정)
#     poses['place_zones']['A']    A 구역 1단 x, y [mm, base]  (None = 미설정)
#   좌표는 __init__에서 저장해 두지 말고, place() 안에서 매번 poses로 읽는다.
# ----------------------------------------------------------------------

# ----------------------------------------------------------------------
# 적재 파트만 가지는 값 — 실측 후 수정 (단위 mm, 로봇 base 기준)
# ----------------------------------------------------------------------
TIER_PITCH_X = -80.0     # 단(자리) 간격: n단 x = 1단 x + (n-1) * TIER_PITCH_X, y는 같음 (−x 방향)
APPROACH_Z = 350.0       # 구역 위 접근 높이 (TCP z). 최대 층 + 쥔 박스 1개 + 여유보다 높게
FLOOR_Z = 120.0          # 박스를 쥔 채 '바닥'에 닿았을 때 TCP z (1층 기준) — 실측
BOX_HEIGHT = {           # 놓는 방향 기준 박스 높이 (판별이 위상을 고정해서 넘긴다는 전제) — 실측
    'A': 40.0,
    'B': 30.0,
    'C': 40.0,
    'D': 50.0,
}
LEVEL_TOL = 0.35         # 층 판단 허용 오차 (박스 높이 대비 비율). 넘으면 기울어짐 등으로 보고 경고
PLACE_RPY = [0.0, 180.0, 0.0]   # 적재 자세 a, b, c [deg] — 실제 그리퍼 방향에 맞게

SPEED, ACC = 60.0, 100.0         # 직선 이동 속도 [mm/s], 가속 [mm/s²]
HOME_VEL = 30.0                  # 관절 이동 속도 [deg/s]
GRIP_WAIT = 1.5                  # 그리퍼 동작 대기 [s]

LEVELS = STACK['levels']         # 층 수 (common.py, 전 파트 공통 — 현재 3)
MAX_TIERS = STACK['max_tiers']   # 구역당 단 수 (common.py, 전 파트 공통)


class PlaceAlgorithm:
    def __init__(self, node, robot):
        self.log = node.get_logger()
        self.robot = robot                 # 관제가 넣어 줌 (None이면 두산 패키지 없이 FAKE로만 실행)

        # [FAKE 전용] 실제 구현 시 삭제 — 구역별로 쌓인 개수를 메모리에 흉내 냄
        node.declare_parameter('fake_delay_sec', 1.0)
        node.declare_parameter('fake_start', [0, 0, 0, 0])   # A~D 처음 개수
        self._delay = node.get_parameter('fake_delay_sec').value
        start_counts = [int(n) for n in node.get_parameter('fake_start').value]
        self._fake_stack = {c: _layout(n) for c, n in zip('ABCD', start_counts)}

    # ------------------------------------------------------------------
    # 약속된 함수
    # ------------------------------------------------------------------
    def place(self, result_class, poses, report) -> int:
        home = poses['home_j']
        if result_class == 'U':                    # 미분류: 놓기만 하고 0 (v0.5)
            report('picking')
            self.pick_from_measure_zone(poses)
            report('moving')
            report('placing')
            self.log.info('[place] (FAKE) 미분류 투하 위치에 놓음 → 0 반환')
            report.log('미분류 투하 완료')
            self.go_home(home)
            return 0
        x1, y = self._zone_xy(poses, result_class)
        h = BOX_HEIGHT[result_class]
        self.log.info(f'[place] {result_class} 구역 1단 x={x1}, y={y}, 박스 높이 {h}mm')

        # 1. 측정존 물품 픽업
        report('picking')
        self.pick_from_measure_zone(poses)

        # 2. 구역 1단 위로 이동
        report('moving')
        self.move_above(x1, y)

        # 3. 1단부터 내려가며 놓을 자리 찾기
        report('placing')
        for tier in range(1, MAX_TIERS + 1):
            x = x1 + (tier - 1) * TIER_PITCH_X
            if tier > 1:
                self.move_above(x, y)
            z = self.descend_until_contact(result_class, tier)
            level = self.level_from_z(z, h)

            if level > LEVELS:                     # 이 단은 이미 꽉 참 → 놓지 않고 다음 단
                self.log.info(f'[place] {tier}단 꽉 참 (z={z:.1f} → {level}층 높이) → 다음 단')
                report.log(f'{result_class} 구역 {tier}단 꽉 참 → {tier + 1}단으로')
                self.move_above(x, y)              # 쥔 채로 올라감
                continue

            self.release(result_class, tier)
            self.move_above(x, y)
            self.go_home(home)
            count = count_from_position(tier, level)
            report.log(f'{result_class} {tier}단 {level}층에 적재 → 수량 {count}')
            self.log.info(f'[place] {result_class} {tier}단 {level}층에 적재 '
                          f'(x={x:.1f}, y={y:.1f}, z={z:.1f}) → 수량 {count}')
            return count

        # 모든 단이 참 → 박스를 쥔 채로 끝남. 측정존에 되돌릴지는 관제 에러 정책과 함께 정함
        raise PartError('NO_SLOT', f'{result_class} 구역 {MAX_TIERS}단 모두 꽉 참 '
                                   f'({MAX_TIERS * LEVELS}개)')

    # ------------------------------------------------------------------
    # z → 층 (실제 동작에서도 그대로 쓰는 계산)
    # ------------------------------------------------------------------
    def level_from_z(self, z, box_h) -> int:
        """박스가 닿은 TCP z → 층. 바닥(FLOOR_Z)에서 닿으면 1층, 박스 1개 위면 2층 ..."""
        stacked = (z - FLOOR_Z) / box_h          # 아래에 깔린 박스 수 (실수)
        level = round(stacked) + 1
        if level < 1:
            raise PartError('PLACE_FAILED', f'바닥보다 낮게 닿음 (z={z:.1f}, FLOOR_Z={FLOOR_Z}) '
                                            '— FLOOR_Z 실측값 확인')
        if abs(stacked - round(stacked)) > LEVEL_TOL:
            self.log.warn(f'[place] 층 판단 애매 (z={z:.1f} → 박스 {stacked:.2f}개 높이) '
                          f'— 아래 박스 기울어짐·높이값 확인 필요, {level}층으로 처리')
        return level

    # ------------------------------------------------------------------
    # [FAKE] 아래를 실제 로봇 동작으로 교체
    # ------------------------------------------------------------------
    def pick_from_measure_zone(self, poses):
        zone = self._measure_zone_xyz(poses)
        # 실제: self.robot.grip(True, GRIP_WAIT); self.robot.movel([*zone, *PLACE_RPY], vel=SPEED, acc=ACC)
        #       self.robot.grip(False, GRIP_WAIT)
        time.sleep(self._delay * 0.5)

    def move_above(self, x, y):
        # 실제: self.robot.movel([x, y, APPROACH_Z, *PLACE_RPY], vel=SPEED, acc=ACC)
        time.sleep(self._delay * 0.3)

    def descend_until_contact(self, result_class, tier) -> float:
        """박스를 쥔 채 내려가다가 더 내려가지 않는 지점의 TCP z를 돌려준다.
        실제 구현 예 (robot = DsrIO, 정지 확인·진행 알림 자동) — 완성 코드는 place_algorithm.py의 _descend
          self.robot.compliance_on([3000, 3000, 500, 200, 200, 200])          # z만 부드럽게
          self.robot.force_on([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0])      # 아래로 15N
          while self.robot.force_z() - base < 6.0: ...                        # 닿을 때까지 (매 반복 진행 알림)
          self.robot.stop()                                                   # 힘·순응제어 해제
          z = self.robot.pose()[2]
        """
        stack = self._fake_stack[result_class]                 # [FAKE]
        below = stack[tier - 1] if tier - 1 < len(stack) else 0
        time.sleep(self._delay * 0.5)
        return FLOOR_Z + below * BOX_HEIGHT[result_class]

    def release(self, result_class, tier):
        # 실제: self.robot.grip(True, GRIP_WAIT) (result_class, tier는 FAKE 스택 갱신용)
        stack = self._fake_stack[result_class]                 # [FAKE]
        while len(stack) < tier:
            stack.append(0)
        stack[tier - 1] += 1
        time.sleep(self._delay * 0.2)

    def go_home(self, home):
        # 실제: self.robot.movej(home, vel=HOME_VEL, acc=HOME_VEL)
        time.sleep(self._delay * 0.3)

    # ------------------------------------------------------------------
    # 좌표 헬퍼
    # ------------------------------------------------------------------
    def _zone_xy(self, poses, result_class):
        """구역 1단 x, y (Goal로 받은 DB 값). 없으면 움직이지 않고 실패."""
        xy = (poses.get('place_zones') or {}).get(result_class)
        if xy is None:
            raise PartError('PLACE_FAILED', f'{result_class} 구역 좌표 없음 (DB place_zones 확인)')
        return float(xy[0]), float(xy[1])

    def _measure_zone_xyz(self, poses):
        """측정존 x, y, z (Goal로 받은 DB 값). 미설정이면 실제 동작에서는 움직이면 안 된다."""
        zone = poses.get('measure_zone')
        if zone is None or all(abs(v) < 1e-6 for v in zone):     # (0, 0, 0)도 미설정으로 본다
            if not getattr(self, '_zone_warned', False):
                self.log.warn('측정존 좌표가 아직 미설정 — '
                              '[FAKE]에서는 넘어가지만 실제 동작에서는 움직이지 않도록 처리할 것')
                self._zone_warned = True
            return None
        return list(zone)


def _layout(n):
    """[FAKE 전용] 개수 → 단별 층 수 (예: 5 → [3, 2])"""
    return [min(LEVELS, n - i) for i in range(0, max(n, 0), LEVELS)]


def main():
    start_algorithm(PlaceAlgorithm)


if __name__ == '__main__':
    main()
