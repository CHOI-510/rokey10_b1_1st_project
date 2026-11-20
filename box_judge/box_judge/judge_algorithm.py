#!/usr/bin/env python3
"""
judge_algorithm.py — 판별 파트 메인 로직 (judge_template.py 약속, 11/18 완성판)
================================================================
  순서
    length1 높이  : 그리퍼를 닫고 닫힌 핑거 끝이 테이블 위 78 mm 에서 movel 하강
                    (5 mm/s · Fz +6 N 로 대략 -> 3 mm 위에서 2 mm/s · Fz +4 N 로 정밀)
    length2 가로  : 그리퍼 각도 그대로, H 에 맞춰 덜 내려가 잡고 폭 측정 (닫히면서 z 상승, sync)
                    -> 쥔 채 3 mm 들어 J6 을 돌려 핑거 축을 J5 축과 일직선으로 -> 내려놓기
    length3 세로  : J6 +90 (레버 보정 자세) -> 힘 제어 하강 + 레버 판정으로 회피 -> 폭 측정 -> 포인트로 재정렬
    weight        : 미측정 (report 만)
    judging       : 세 길이를 크기순으로 기준과 비교 (모두 tol_mm 안) -> 후보
                    위상 정렬: L1 x L2 면이 바닥, L1 = 로봇 x, L2 = 로봇 y, 측정존 중앙 (적재가 그대로 집는다)
    materiality   : 후보가 여럿일 때만 (C · D 처럼 치수가 같은 물품). 3 -> 40 N 폭 변화로 말랑 / 딱딱
    결과          : 후보 하나면 그것, 없으면 UNCLASSIFIED('U')

  위상 정렬 (rokey1 phase_align_test 실물 검증판을 줄인 것)
    - 넘기지 않음 (H 가 이미 가장 짧은 변): 세운 그리퍼로 짧은 수평 변을 잡아 J6 로 돌려 놓기
    - 넘기기: 클램핑 축이 팔 방향과 수직이면 먼저 세운 그리퍼로 잡아 J6 로 돌려 놓는다
              (팔 평면 안에서 앞뒤로 기울이는 넘기기는 손목이 못 간다). 그 다음 +40 deg 로 기울여 잡고
              원래 TCP 중심으로 -50 deg 까지 넘겨 그 자세로 놓는다 (닿을 때까지 하강) -> 세운 그리퍼로 J6 수직축 회전
    - 축 맞춤: 세운 그리퍼로 클램핑해 면을 축에 맞추고 중앙으로 (앞 단계에서 이미 클램핑한 축은 건너뜀)
    - J6 는 0 ± 150 deg 안에서 돌리고 끝에 되감는다 (툴 케이블 꼬임 방지)

  템플릿 약속: JudgeAlgorithm(node, robot), judge(specs, poses, report) -> 'A'~'D' 또는 'U'
    실패는 PartError('MEASURE_FAILED' / 'GRASP_FAILED'), CANCELED 는 잡지 않는다. 로봇은 robot(DsrIO)으로만.
  테이블 기준(측정존 바닥 Z): Goal 의 DB 측정존 z (poses['measure_zone'][2], 적재와 같은 값)
================================================================
"""

import math
import statistics
import time

from box_comm.common import GRIP_RETRY, GRIPPER, SYNC_GRIP, TCP, TRAVEL_SPEED_MM_S, UNCLASSIFIED
from box_comm.gripper import holding
from box_comm.judge_interface import PartError
from box_comm.sync_grip import GripMissed, SyncGripper, grasp_height_for_tip, tip_h_mm

from box_judge import phase_plan as pp

# 이름: (기본값, 설명) — rokey1 judge_flow_test · phase_align_test · rg2_stiffness_cycle 실물 값 (11/16 ~ 18)
PARAMS = {
    # sync 집기 (common.SYNC_GRIP 과 같은 값)
    'grasp_height': (round(grasp_height_for_tip(SYNC_GRIP['tip_above_floor_mm'], SYNC_GRIP['ref_raw_mm']), 2),
                     '기준 폭에서 TCP 를 책상 위 몇 mm 에 (핑거 끝 2.6)'),
    'ref_width': (SYNC_GRIP['ref_raw_mm'], 'grasp_height 를 맞춘 원시 폭 [mm]'),
    'tcp_z': (TCP['pos'][2], 'TCP GripperDA_v1 z 오프셋 (common.TCP)'),
    'grip_force': (SYNC_GRIP['close_force_n'], '폭 잴 때 닫는 힘 [N] (닫힘 시간식 8~12 N)'),
    # 아래 4개는 common.GRIPPER가 원본 (11/18 공용값으로 — 값은 그대로: 1.0 / 0.30 / 100 / 20)
    'cal_gain': (GRIPPER['cal_gain'], '폭 보정 기울기 (패드 장착, 실제 = 원시 - 10.30)'),
    'cal_offset': (GRIPPER['cal_offset_mm'], '폭 보정 오프셋 [mm]'),
    'open_width': (GRIPPER['open_width_mm'], '열 때 패드 간격 [mm]'),
    'open_force': (GRIPPER['open_force_n'], '여는 힘 [N]'),
    # 측정
    'obj_h_max': (75.0, '최대 물체 높이 [mm]'),
    'approach_clear': (10.0, '열린 핑거로 내려가기 시작할 때 핑거 끝 - 윗면 [mm]'),
    'body_clear': (31.0, '핑거 끝 위로 들어올 수 있는 물체 높이 [mm]'),
    'clear_after': (5.0, '내려놓은 뒤 열린 핑거 끝 - 윗면 [mm]'),
    'close_clear': (10.0, '빈 그리퍼 닫기 전 닫힌 핑거 끝 - 최대 물체 [mm]'),
    'h_floor_offset': (-1.16, '높이 기준 = 측정존 바닥 z + 이 값 [mm]: 닫힌 핑거 끝 바닥 접촉 TCP z 29.74 - 바닥 30.9 (11/17 실측). sync 집기는 바닥 z 그대로'),
    'h_start': (78.0, '높이: 닫힌 핑거 끝 테이블 위 출발 높이 [mm]'),
    'h_fast_vel': (5.0, '높이 1차 속도 [mm/s] (11/18 8 -> 5 로 되돌림)'),
    'h_fast_thr': (6.0, '높이 1차 판정 Fz [N] (5 mm/s 에서 실측한 값)'),
    'h_backoff': (3.0, '1차 접촉 뒤 상승 [mm]'),
    'h_vel': (2.0, '높이 2차 속도 [mm/s]'),
    'h_thr': (4.0, '높이 2차 판정 Fz [N]'),
    'p1_vel': (25.0, '물체 윗면 아래로 잡으러 내려가는 속도 [mm/s] (11/18)'),
    'p1_thr': (12.0, '그때 핑거가 얹힌 것으로 보는 Fz [N]'),
    'carry_lift': (3.0, '쥔 채 돌리거나 옮길 때 들어 올림 [mm]'),
    'put_vel': (8.0, '쥔 물체 내려놓기 속도 [mm/s]'),
    'push_force': (20.0, '회피 힘 제어 하강 [N]'),
    'stall_time': (0.6, '회피: z 정체 시간 [s]'),
    'press_settle': (0.5, '회피: 누른 채 대기 [s]'),
    'force_timeout': (25.0, '회피: 최대 시간 [s]'),
    'lever_mid_x': (-23.0, '레버 중간점 x [mm] (J6 -1.1 자세 · 20 N · 회전 강성 50 보정)'),
    'lever_mid_y': (14.1, '레버 중간점 y [mm]'),
    'lever_dir_x': (-0.501, '레버 b->a 방향 x'),
    'lever_dir_y': (-0.866, '레버 b->a 방향 y'),
    'lever_margin': (10.0, '판정 불가 범위 [mm]'),
    'side_sign': (-1, '핑거 a 가 툴 -x 쪽이면 -1'),
    'shift': (20.0, '회피 한 번 이동 [mm] (패드 간격 99.5 - 최대 물체 75 = 24.5 까지)'),
    'max_iter': (8, '회피 최대 횟수'),
    'max_shift': (60.0, '포인트에서 최대 이탈 [mm] (11/19 40 -> 60, 회피 20 mm x 3번)'),
    'speed': (60.0, '일반 직선 이동 속도 [mm/s]'),
    'home_vel': (60.0, '홈 관절 이동 [deg/s]'),
    # 판별
    'tol_mm': (2.5, '가로 · 세로 허용 오차 [mm] (11/20 3 -> 2.5, 탐지 방향 정렬 뒤 대각 잡기 없음)'),
    'tol_h_mm': (3.0, '높이 허용 오차 [mm] (바닥 기준이 들어가 여유를 더 둠)'),
    'soft_classes': ('D', '말랑한 물품 class (쉼표). 치수가 같은 후보를 물성으로 가를 때'),
    'soft_dw': (1.0, '3 -> 40 N 폭 변화가 이 이상이면 말랑 [mm] (알루미늄 0.6~0.7, 지우개 1.3~1.6)'),
    # 위상 정렬
    'phase_align': (True, '위상 정렬 실행'),
    'tilt': (40.0, '기울여 잡는 각 [deg] (놓을 때 = 이 값 - 90)'),
    'margin': (4.0, '핑거 · 물체 최저점과 책상 사이 최소 여유 [mm]'),
    'flip_grip_force': (20.0, '넘길 때 쥐는 힘 [N]'),
    'grip_vel': (25.0, '세운 그리퍼로 잡으러 내려가는 속도 [mm/s] (11/18)'),
    'grip_thr': (12.0, '그때 얹힘 판정 Fz [N]'),
    'approach_vel': (20.0, '기운 자세 접근 속도 [mm/s]'),
    'approach_thr': (6.0, '기운 접근 중 |dF| 판정 [N]'),
    'rot_vel': (60.0, '자세 바꿀 때 회전 속도 [deg/s]'),
    'place_vel': (5.0, '넘긴 물체 내려놓기 속도 [mm/s]'),
    'place_thr': (6.0, '내려놓기 접촉 Fz [N]'),
    'place_gap': (8.0, '예상 접촉 위 이 높이부터 천천히 [mm]'),
    'place_bias': (-4.5, '놓기 예상 접촉 보정 [mm] (11/18 실측 -4.1 ~ -4.8)'),
    'place_over': (45.0, '예상보다 이만큼 더 내려가도 안 닿으면 그대로 놓음 [mm] (핑거 여유 안에서)'),
    'square_open': (pp.PAD_MAX, '세운 그리퍼 클램핑 때 변 + 이만큼 열기 [mm] (최대 패드 간격까지, 기본 = 최대로 열기)'),
    'j6_window': (150.0, 'J6 를 0 ± 이 안에 두려 한다 [deg] (케이블)'),
    'tip_table_z': (7.18, '기운 자세 계산용 실제 책상 base z [mm] (11/17 drop 맞춤)'),
    'tip_L': (17.44, '핑거 끝이 TCP 보다 툴 z 로 앞인 거리 [mm] @ tip_raw'),
    'tip_raw': (50.3, '위 값을 잰 원시 폭 [mm]'),
    'tip_b': (2.83, '핑거 끝 반두께 [mm]'),
    'tip_y0': (4.57, '핑거 끝 중심의 툴 y 위치 [mm]'),
    # 물성
    'stiff_settle': (1.0, '40 N 뒤 폭 읽기 전 대기 [s] (말랑한 물체는 기다릴수록 더 눌림)'),
    'regrip_gap': (6.0, '물성: 폭 + 이만큼 열고 내려가 3 N 으로 잡음 [mm]'),
}

PLAN_CFG = {'tilt_sign': 1, 'flip_tol': 2.0, 'tip_past': 8.0, 'grip_edge': 6.0, 'grid': 1.0, 'lift_extra': 5.0,
            'approach': 25.0, 'obj_clear': 10.0, 'open_margin': 8.0}
RELEASE_WIDTH = pp.PAD_MAX


class _Fail(Exception):
    pass


class RobotPort:
    """판별에 필요한 로봇 · 그리퍼 명령 모음 — 전부 robot(DsrIO) (정지 확인 · 진행 알림 포함)."""

    def __init__(self, node, robot, p):
        self.node, self.robot, self.p = node, robot, p

    @property
    def g(self):
        return self.robot.gripper                    # 이번 Goal 의 RG2 (처음 부를 때 접속, Goal 끝에 box_comm 이 끊음)

    @property
    def pad_total(self):
        return self.g.pad_total

    def pose(self):
        return [float(v) for v in self.robot.pose()]

    def cur_z(self):
        return self.pose()[2]

    def movel_abs(self, pose6, vel):
        self.robot.movel(list(pose6), vel=vel, acc=vel * 3)

    def move_z_to(self, z, vel):
        p = self.pose()
        p[2] = z
        self.movel_abs(p, vel)

    def move_xy_to(self, x, y, vel):
        p = self.pose()
        p[0], p[1] = x, y
        self.movel_abs(p, vel)

    def move_tool_x(self, d, vel):
        p = self.pose()
        R = pp.zyz_mat(*p[3:6])
        p[0] += R[0][0] * d
        p[1] += R[1][0] * d
        self.movel_abs(p, vel)

    def mwait(self):
        self.robot.mwait()

    def amovel(self, rel6, vel=None, acc=None, time=None, **_):
        self.robot.amovel_rel(rel6, vel=vel, acc=acc, time_s=time)

    def stop(self, mode=2):
        return self.robot.move_stop(mode)

    def j6(self):
        return float(self.robot.joints()[5])

    def turn_j6(self, d):
        if abs(d) > 0.05:
            q = self.robot.joints()
            q[5] += d
            self.robot.movej(q, vel=60, acc=120)

    def ftm(self, n, interval=0.03):
        """(base Fz, 툴 Tx, 툴 Ty) 평균, Fz 변동 폭."""
        fz, tx, ty = [], [], []
        for _ in range(n):
            fb, ft = self.robot.force('base'), self.robot.force('tool')
            fz.append(float(fb[2]))
            tx.append(float(ft[3]))
            ty.append(float(ft[4]))
            time.sleep(interval)
        return (statistics.mean(fz), statistics.mean(tx), statistics.mean(ty)), max(fz) - min(fz)

    def fvec(self):
        return [float(v) for v in self.robot.force('base')[:3]]

    def force_down_on(self):
        self.robot.compliance_on([2000, 2000, 2000, 50, 50, 50])
        time.sleep(0.5)

    def force_push(self, push):
        self.robot.force_on([0, 0, -abs(push), 0, 0, 0], [0, 0, 1, 0, 0, 0], time_s=0.0)

    def force_off(self):
        self.robot.stop()


class JudgeAlgorithm:
    def __init__(self, node, robot):
        self.node = node
        self.log = node.get_logger()
        self.robot = robot
        self.p = {}
        for k, (v, _desc) in PARAMS.items():
            node.declare_parameter(k, v)
            self.p[k] = node.get_parameter(k).value
        p = self.p
        self.tip_clearance = p['grasp_height'] + (p['tcp_z'] - tip_h_mm(p['ref_width']))
        self.model = pp.FingerModel(p['tip_table_z'], p['tip_L'], p['tip_raw'], p['tip_b'], p['tip_y0'])
        self.io = RobotPort(node, robot, p) if robot is not None else None
        self.sg = None
        self._report = None

    def _make_sync_gripper(self, table_z):
        """Goal마다 만든다. table_z = Goal의 측정존 바닥 z (DB) — 높이 측정과 같은 값 (11/18 관제: 예전 PARAMS table_z 30.9 삭제)"""
        p, g = self.p, self.io.g
        st = g.status()                               # 닫혀 있으면 반영 폭이 음수 -> 부호 없이 읽으면 패드 합이 -6543 (11/18)
        if not (0.0 <= g.pad_total <= 30.0) or abs(g.pad_total - 2.0 * st['offset_mm']) > 1.0:
            self.log.warn(f'[judge] 패드 두께 합 {g.pad_total:.1f} 이상 -> 오프셋 x2 = {2.0 * st["offset_mm"]:.1f} 로 씀')
            g.pad_total = 2.0 * st['offset_mm']
        self.sg = SyncGripper(g, self.io, table_z=table_z, grasp_height=p['grasp_height'],
                              ref_width=p['ref_width'], tcp_z=p['tcp_z'], force=p['grip_force'],
                              cal_gain=p['cal_gain'], cal_offset=p['cal_offset'], put_vel=p['put_vel'],
                              log=self.log.info)

    def _tick(self):
        if self._report is not None and hasattr(self._report, 'keepalive'):
            self._report.keepalive()

    # ------------------------------------------------------------------ 약속된 함수
    def judge(self, specs, poses, report) -> str:
        if self.io is None:
            raise PartError('MEASURE_FAILED', '로봇 객체 없음 (두산 패키지 미설치)')
        self._report = report
        zone = self._measure_zone(poses)
        waypoint_orientation = self.robot.pose()[3:6]
        table_z = float(zone[2])                      # 측정존 바닥 Z (DB 하나 — 높이 측정·sync 집기 공통, 11/18)
        self._make_sync_gripper(table_z)
        px, py = zone[0], zone[1]
        io, p = self.io, self.p
        H = None
        try:
            self.robot.move_to_waypoint(zone, orientation=waypoint_orientation,
                                        vel=p['speed'], acc=p['speed'] * 3,
                                        travel_vel=TRAVEL_SPEED_MM_S)
            io.move_z_to(max(io.cur_z(), self._safe_closed_z(table_z)), p['speed'])

            report('measuring_length1')
            H = self._measure_height(table_z)
            report.log(f'높이 {H:.1f} mm')

            report('measuring_length2')
            W, tip = self._measure_width(H)
            self._turn_and_place(px, py, H, W)
            report.log(f'가로 {W:.1f} mm, J5 일직선으로 돌려 내려놓음')

            report('measuring_length3')
            D, n_avoid = self._avoid_and_depth(px, py, H, tip)
            report.log(f'세로 {D:.1f} mm (회피 {n_avoid}회)')

            report('measuring_weight')                # 무게는 미측정

            report('judging')
            lengths = [H, W, D]
            report.log(f'측정값 {H:.1f} x {W:.1f} x {D:.1f} mm')
            cands = self.candidates(specs, lengths)
            report.log('치수 후보 ' + ('/'.join(s['class_id'] for s in cands) or '없음 (U)'))
            L = sorted(lengths, reverse=True)
            aligned = False
            if p['phase_align']:
                aligned = self._phase_align({'H': H, 'W': W, 'D': D}, px, py, H, required=bool(cands))
            dw = None
            if len(cands) > 1 and aligned:
                report('measuring_materiality')
                dw = self._stiffness(L[1], L[2])
                report.log(f'물성: 3 -> 40 N 폭 변화 {dw:.2f} mm ({"말랑" if dw >= p["soft_dw"] else "딱딱"})')
                report('judging')
        except _Fail as e:
            raise PartError('MEASURE_FAILED', str(e))
        finally:
            self._cleanup(H, table_z)

        result = self.classify(specs, lengths, dw)
        report.log(f'판별 결과 {result}')
        self.robot.move_to_waypoint(zone, orientation=waypoint_orientation,
                                    vel=p['speed'], acc=p['speed'] * 3,
                                    travel_vel=TRAVEL_SPEED_MM_S)
        return result

    # ------------------------------------------------------------------ 판별
    def candidates(self, specs, lengths):
        """lengths = [H, W, D]. 크기순으로 맞춰 비교하되 허용 오차는 측정 축별 (높이 tol_h_mm, 가로 · 세로 tol_mm)."""
        tols = [self.p['tol_h_mm'], self.p['tol_mm'], self.p['tol_mm']]
        m = sorted(zip(lengths, tols))
        out = []
        for s in specs:
            ref = sorted([s['width_mm'], s['depth_mm'], s['height_mm']])
            if all(abs(a - b) <= t for (a, t), b in zip(m, ref)):
                out.append(s)
        return out

    def classify(self, specs, lengths, dw=None) -> str:
        """치수 후보가 하나면 그것. 여럿이면 물성(dw)으로 soft_classes 와 맞는 것. 없으면 U."""
        cands = self.candidates(specs, lengths)
        if len(cands) > 1:
            if dw is None:
                return UNCLASSIFIED
            soft = dw >= self.p['soft_dw']
            softset = {c.strip() for c in str(self.p['soft_classes']).split(',') if c.strip()}
            cands = [s for s in cands if (s['class_id'] in softset) == soft]
        if len(cands) != 1:
            return UNCLASSIFIED
        return cands[0]['class_id']

    # ------------------------------------------------------------------ 측정 단계
    def _measure_height(self, table_z):
        io, p = self.io, self.p
        table_z = table_z + p['h_floor_offset']                   # 높이 기준: 닫힌 핑거 끝이 바닥에 닿는 TCP z (11/20)
        io.g.grip_until_done(p['grip_force'])                     # 최대 물체 위에서 빈 그리퍼 닫기
        io.move_z_to(table_z + p['h_start'], p['speed'])
        sA, zA = self._pos_descent(table_z - 1.0, p['h_fast_vel'], p['h_fast_thr'], 0.4, 1.0)
        if sA != 'contact':
            raise _Fail(f'높이 1차 하강에서 윗면을 못 찾음 ({sA})')
        io.move_z_to(zA + p['h_backoff'], 30.0)
        sB, zB = self._pos_descent(table_z - 1.0, p['h_vel'], p['h_thr'], 0.6, 0.5)
        if sB != 'contact':
            raise _Fail(f'높이 2차 하강에서 윗면을 못 찾음 ({sB})')
        io.move_z_to(zB + p['h_backoff'], 30.0)
        return zB - table_z

    def _measure_width(self, H):
        io, p = self.io, self.p
        tip = max(self.tip_clearance, H - p['body_clear'])

        def descend():
            raw_open = io.g.open(p['open_force'], p['open_width'])
            z_above = self._tcp_open_tip(raw_open, H + p['approach_clear'])
            if io.cur_z() < z_above:
                io.move_z_to(z_above, 30.0)
            s, z = self._pos_descent(self._tcp_open_tip(raw_open, tip), p['p1_vel'], p['p1_thr'], 0.4, 1.0)
            if s != 'clear':
                io.move_z_to(z_above, 30.0)
                if s in ('contact', 'safety'):        # 같은 자리로 다시 내려가도 같음 → 측정존 비우고 재스캔
                    raise PartError('GRASP_FAILED', f'가로 하강 중 {s} (핑거가 물체 위에 얹힘)')
                raise _Fail(f'가로 하강 중 {s}')

        descend()
        r = self._close_retry('가로', descend)
        return r['width_mm'], tip

    def _close_retry(self, what, redescend):
        """sync 닫기. 빈손이면 같은 자리·자세로 다시 열고 내려가 GRIP_RETRY번까지. 다 실패하면 GRASP_FAILED"""
        why = ''
        for n in range(1, GRIP_RETRY + 1):
            try:
                r = self.sg.close_with_lift(SYNC_GRIP['est_raw_mm'])
                if n > 1:
                    self._report.log(f'{what} 집기 성공 ({n}/{GRIP_RETRY}번째)')
                return r
            except GripMissed as e:
                why = str(e)
            nxt = f'재시도 {n + 1}/{GRIP_RETRY}' if n < GRIP_RETRY else '포기'
            self._report.log(f'{what} 집기 실패 {n}/{GRIP_RETRY}: {why} → {nxt}', 'WARN')
            if n < GRIP_RETRY:
                redescend()
        raise PartError('GRASP_FAILED', f'{what}: {GRIP_RETRY}회 모두 못 잡음 ({why})')

    def _check_holding(self, width, where):
        ok, why, _w = holding(self.io.g, width, check_range=False)
        if not ok:
            raise PartError('GRASP_FAILED', f'{where} 놓침: {why}')

    def _turn_and_place(self, px, py, H, W):
        io, p = self.io, self.p
        z_hold = io.cur_z()
        io.move_z_to(z_hold + p['carry_lift'], p['put_vel'])
        self._check_holding(W, '가로 잰 뒤 들어 올리다')
        # 핑거 축 = 팔 방향에 수직 = J5 축과 일직선. J6 는 -91.1 쪽 (그래야 [4] +90 뒤 레버 보정 자세 -1.1)
        self._turn_to_yaw(math.degrees(math.atan2(py, px)) + 90.0, prefer_j6=-91.1)
        io.move_z_to(z_hold, p['put_vel'])
        raw_open = io.g.open(p['open_force'], p['open_width'])
        io.move_z_to(max(io.cur_z(), self._tcp_open_tip(raw_open, H + p['clear_after'])), 30.0)

    def _avoid_and_depth(self, px, py, H, tip):
        io, p = self.io, self.p
        io.turn_j6(90.0)
        if abs(io.j6() - (-1.1)) > 3.0:
            self.log.warn(f'[judge] 회피 자세 J6 {io.j6():+.1f} 가 레버 보정 자세(-1.1)와 다름')
        raw_open = io.g.status()['width_raw_mm']
        z_start = self._tcp_open_tip(raw_open, H + p['approach_clear'])
        z_target = self._tcp_open_tip(raw_open, tip)
        io.move_z_to(z_start, 30.0)
        n = 0
        while True:
            s, z, lever = self._force_probe(z_target)
            if s == 'clear':
                break
            if s != 'contact':
                io.move_z_to(z_start, 30.0)
                raise _Fail(f'회피 하강 {s}')
            side, proj = self._lever_side(lever)
            if side == 0:
                io.move_z_to(z_start, 30.0)
                raise _Fail(f'어느 핑거인지 판정 불가 (투영 {proj:+.1f} mm)')
            n += 1
            self.log.info(f'[judge] 회피 #{n}: 투영 {proj:+.1f} -> 툴 x {side * p["shift"]:+.0f} mm')
            if n > p['max_iter']:
                io.move_z_to(z_start, 30.0)
                raise _Fail(f'회피 {p["max_iter"]}회 초과')
            io.move_z_to(io.cur_z() + 2.0, 30.0)
            io.move_tool_x(side * p['shift'], 30.0)
            q = io.pose()
            if math.hypot(q[0] - px, q[1] - py) > p['max_shift']:
                io.move_z_to(z_start, 30.0)
                raise _Fail('측정존에서 너무 멀리 벗어남')
        q = io.pose()
        off = (q[0] - px, q[1] - py)

        def redescend():
            raw = io.g.open(p['open_force'], p['open_width'])
            s, _z = self._pos_descent(self._tcp_open_tip(raw, tip), p['p1_vel'], p['p1_thr'], 0.4, 1.0)
            if s in ('contact', 'safety'):
                io.move_z_to(z_start, 30.0)
                raise PartError('GRASP_FAILED', f'세로 다시 내려가다 {s} (핑거가 물체 위에 얹힘)')

        r = self._close_retry('세로', redescend)
        if math.hypot(*off) >= 1.0:                                # 쥔 채 측정존 중심으로 옮겨 놓기
            z_hold = io.cur_z()
            io.move_z_to(z_hold + p['carry_lift'], p['put_vel'])
            self._check_holding(r['width_mm'], '세로 잰 뒤 들어 올리다')
            io.move_xy_to(px, py, 30.0)
            io.move_z_to(z_hold, p['put_vel'])
        raw_open = io.g.open(p['open_force'], p['open_width'])
        io.move_z_to(max(io.cur_z(), self._tcp_open_tip(raw_open, H + p['clear_after'])), 30.0)
        return r['width_mm'], n

    # ------------------------------------------------------------------ 위상 정렬
    def _phase_align(self, dims, px, py, H, required):
        """L1 x L2 면을 바닥, L1 = x, L2 = y 로. 계획이 안 나오면 required(치수 후보 있음)일 때만 실패."""
        p = self.p
        cfg = dict(PLAN_CFG, tilts=[p['tilt']], margin=p['margin'], body_clear=p['body_clear'],
                   cal_gain=p['cal_gain'], cal_offset=p['cal_offset'])
        arm = math.degrees(math.atan2(py, px))
        psi = arm                                                   # 측정 끝: D 가 팔 방향, W 가 J5 축 방향
        try:
            plan = pp.plan(cfg, dims, psi, (px, py), self.model, self.io.pad_total, self._x_yaw())
        except pp.PlanFailed as e:
            if required:
                raise _Fail(f'위상 정렬 계획 실패: {e}')
            self._report.log(f'위상 정렬 생략 (미분류 물체, 계획 불가: {e})', 'WARN')
            return False
        self._report.log('위상 정렬: ' + pp.describe(plan))
        centered = None
        if plan['case'] == 'yaw':
            self._hold_turn(plan['clamp_mm'], H, plan['clamp_yaw'], plan['clamp_yaw'] + plan['dpsi'])
            centered = pp.wrap90(plan['clamp_yaw'] + plan['dpsi'])
        else:
            if abs(pp.wrap90(pp.yaw_deg(plan['xhat']) - arm)) > 45.0:
                # 클램핑 축이 팔과 수직 -> 넘길 때 팔 평면 안에서 기울어야 해 손목이 못 간다. 먼저 돌려 놓는다:
                # 넘긴 뒤 이 변이 갈 축(L1 -> x, L2 -> y)이 팔과 45 deg 이내면 그 축(넘긴 뒤 회전 0), 아니면 팔 방향
                goal = 0.0 if plan['clamp_mm'] >= H else 90.0
                want = goal if abs(pp.wrap90(goal - arm)) <= 45.0 else arm
                rot = pp.wrap90(want - pp.yaw_deg(plan['xhat']))
                self._hold_turn(plan['clamp_mm'], H, pp.yaw_deg(plan['xhat']), pp.yaw_deg(plan['xhat']) + rot)
                psi += rot
                plan = pp.plan(cfg, dims, psi, (px, py), self.model, self.io.pad_total, self._x_yaw())
                self._report.log(f'클램핑 축을 {rot:+.0f} deg 돌려 놓고 다시 계획: ' + pp.describe(plan))
            self._flip(plan, px, py, H)
            if abs(plan['dpsi']) > 1.0:
                y0 = pp.yaw_deg(plan['xhat'])
                self._hold_turn(plan['clamp_mm'], plan['final_h'], y0, y0 + plan['dpsi'])
                centered = pp.wrap90(y0 + plan['dpsi'])
        # 축 맞춤: 세운 그리퍼로 클램핑해 면을 축에 맞추고 중앙으로 (y(L2) 먼저, 이미 클램핑한 축은 건너뜀)
        L, h = plan['L'], plan['final_h']
        for dim, yaw_t in ((L[1], 90.0), (L[0], 0.0)):
            if centered is not None and abs(pp.wrap90(centered - yaw_t)) < 10.0:
                continue
            w = self._vclamp(dim, h, yaw_t)
            self._release(w, h)
        self._unwind_j6()
        self._report.log('위상 정렬 완료')
        return True

    def _flip(self, plan, px, py, H):
        io, p = self.io, self.p
        P, R1, R2, place = plan['P'], plan['R1'], plan['R2'], plan['place']
        # 1. 세운 채 J6 로 툴 x 를 정확히 맞춘 뒤 기운 접근 시작점으로 (이동에는 기울기만 남긴다)
        io.move_z_to(max(io.cur_z(), self._safe_z(H), plan['P_start'][2]), p['speed'])
        io.g.open(p['open_force'], plan['clamp_mm'] + PLAN_CFG['open_margin'])
        self._turn_to_yaw(pp.yaw_deg(plan['xhat']), exact=True)
        self._move_pose(plan['P_start'], R1, '기운 접근 시작점')
        # 2. 툴 z 로 들어가 잡기
        s = self._tool_approach(P, R1)
        if s != 'clear':
            self._move_pose(plan['P_start'], R1, '접근 후퇴')
            raise _Fail(f'기운 접근 중 {s} (핑거나 손바닥이 물체에 닿음)')
        io.g.grip_until_done(p['flip_grip_force'])
        raw, _sp = io.g.settled_width()
        width = self.sg.calibrated(raw)
        if not io.g.status()['grip'] or abs(width - plan['clamp_mm']) > 6.0:
            io.g.open(p['open_force'], plan['clamp_mm'] + PLAN_CFG['open_margin'])
            self._move_pose(plan['P_start'], R1, '잡기 실패 후퇴')
            raise PartError('GRASP_FAILED', f'넘기기 잡은 폭 {width:.1f} 이 예상 {plan["clamp_mm"]:.1f} 와 다름')
        # 3. 들어 올려 넘기고 (원래 TCP 중심) 4. 넘긴 자세 그대로 놓을 자리 위로
        z_f = P[2] + plan['lift']
        io.move_z_to(z_f, p['put_vel'])
        self._check_holding(width, '넘기려고 들어 올리다')
        self._move_pose([P[0], P[1], z_f], R2, '넘기기')
        self._move_pose([place[0], place[1], z_f], R2, '놓을 자리 위')
        # 5. 닿을 때까지 내려놓고 넓게 열고 툴 z 반대로 빠진 뒤 그리퍼를 세운다
        z_exp = place[2] + p['place_bias']
        if z_f - z_exp > p['place_gap'] + 1.0:
            io.move_z_to(z_exp + p['place_gap'], 30.0)
        over = max(5.0, min(p['place_over'], plan['cf'] - 2.0))     # 핑거가 책상에 닿기 전까지만
        s, z = self._pos_descent(place[2] - over, p['place_vel'], p['place_thr'], 0.4, 1.0)
        if s != 'contact':
            self._report.log(f'넘긴 물체 내려놓기 {s} (예상 {place[2]:.1f}) - 그대로 놓음', 'WARN')
        io.g.open(p['open_force'], RELEASE_WIDTH)                    # 좁게 열면 빠질 때 패드가 물체를 끈다
        z2 = pp.col(R2, 2)
        self._move_pose(pp.sub(io.pose()[:3], pp.mul(z2, plan['retreat'])), R2, '후퇴')
        self._move_pose([px, py, max(io.cur_z(), self._safe_z(plan['final_h'], 15.0))],
                        pp.down_frame(pp.unit_yaw(pp.yaw_deg(pp.col(R2, 0)))), '그리퍼 세우기')

    def _tool_approach(self, P, R, thr=None):
        """툴 z 방향으로 P 까지 (비동기) 들어가며 |dF| 를 본다. 'clear' / 'contact' / 'timeout'."""
        io, p = self.io, self.p
        thr = p['approach_thr'] if thr is None else thr
        time.sleep(0.3)
        f0 = [statistics.mean(c) for c in zip(*[io.fvec() for _ in range(10)])]
        start = io.pose()[:3]
        dist = math.dist(start, P)
        v = p['approach_vel']
        io.robot.amovel(list(P) + list(pp.mat_zyz(R)), vel=[v, 30.0], acc=[v * 4, 60.0])
        t0, status = time.time(), 'timeout'
        while time.time() - t0 < dist / v + 3.0:
            dF = math.dist(io.fvec(), f0)
            if dF >= thr and time.time() - t0 >= 0.4:
                status = 'contact'
                break
            if math.dist(io.pose()[:3], P) <= 0.3:
                status = 'clear'
                break
            time.sleep(0.02)
        if status != 'clear':
            io.stop(1)
        io.mwait()
        return status

    def _move_pose(self, pos, R, what):
        """절대 자세 이동 + 자세 확인 (컨트롤러가 이동을 거부하면 제자리라 여기서 걸린다)."""
        A, B, C = pp.mat_zyz(R)
        v, r = self.p['speed'], self.p['rot_vel']
        try:
            self.io.robot.movel([pos[0], pos[1], pos[2], A, B, C], vel=[v, r], acc=[v * 3, r * 2])
        except RuntimeError as e:
            raise _Fail(f'{what}: {e}')
        if pp.rot_err_deg(pp.zyz_mat(*self.io.pose()[3:6]), R) > 3.0:
            raise _Fail(f'{what}: 자세가 목표와 다름 (로봇이 이동을 거부)')
        self._tick()

    # ---- 세운 그리퍼 동작
    def _vclamp(self, dim, obj_h, yaw):
        """세운 그리퍼로 방위각 yaw 방향의 dim 변을 잡는다 (잡힌 채 끝남). 잰 폭 반환."""
        io, p = self.io, self.p
        io.move_z_to(max(io.cur_z(), self._safe_z(obj_h)), p['speed'])
        self._turn_to_yaw(yaw)
        raw_open = io.g.open(p['open_force'], min(pp.PAD_MAX, dim + p['square_open']))
        tip = max(self.tip_clearance, obj_h - p['body_clear'])
        z_above = self._tcp_open_tip(raw_open, obj_h + p['approach_clear'])
        if io.cur_z() < z_above:
            io.move_z_to(z_above, 30.0)
        s, _z = self._pos_descent(self._tcp_open_tip(raw_open, tip), p['grip_vel'], p['grip_thr'], 0.4, 1.0)
        if s != 'clear':
            io.move_z_to(z_above, 30.0)
            raise _Fail(f'세운 그리퍼 하강 중 {s} (핑거가 물체 위에 얹힘)')
        try:
            return self.sg.close_with_lift(SYNC_GRIP['est_raw_mm'])['width_mm']
        except GripMissed as e:
            raise _Fail(f'세운 그리퍼로 못 잡음: {e}')

    def _release(self, width, obj_h):
        raw = self.io.g.open(self.p['open_force'], min(pp.PAD_MAX, width + self.p['square_open']))
        self.io.move_z_to(max(self.io.cur_z(), self._tcp_open_tip(raw, obj_h + self.p['clear_after'])), 30.0)

    def _hold_turn(self, dim, obj_h, yaw_from, yaw_to):
        """세운 그리퍼로 dim 변을 잡아 3 mm 들고 J6 로 yaw_to 까지 돌려 내려놓는다."""
        io, p = self.io, self.p
        w = self._vclamp(dim, obj_h, yaw_from)
        z_hold = io.cur_z()
        io.move_z_to(z_hold + p['carry_lift'], p['put_vel'])
        self._check_holding(w, '돌리려고 들어 올리다')
        self._turn_to_yaw(yaw_to)
        io.move_z_to(z_hold, p['put_vel'])
        self._release(w, obj_h)

    # ------------------------------------------------------------------ 물성
    def _stiffness(self, width, obj_h):
        """길이2(로봇 y) 방향을 3 N 으로 잡고 40 N 으로 눌러 폭 변화 [mm] (양수 = 줄어든 양).
        폭을 이미 알고 축 맞춤으로 중앙이라 폭 + gap 으로만 열고 내려간다 (10 N 폭 찾기 생략)."""
        io, p = self.io, self.p
        g = io.g
        io.move_z_to(max(io.cur_z(), self._safe_z(obj_h)), p['speed'])
        self._turn_to_yaw(90.0)
        raw_open = g.open(p['open_force'], width + p['regrip_gap'])
        tip = max(self.tip_clearance + 2.0, obj_h - p['body_clear'])   # 3 N 으로 닫히며 핑거 끝이 1~2 mm 내려감
        z_above = self._tcp_open_tip(raw_open, obj_h + p['approach_clear'])
        if io.cur_z() < z_above:
            io.move_z_to(z_above, 30.0)
        s, _z = self._pos_descent(self._tcp_open_tip(raw_open, tip), p['grip_vel'], p['grip_thr'], 0.4, 1.0)
        if s != 'clear':
            io.move_z_to(z_above, 30.0)
            raise _Fail(f'물성: 내려가다 {s} (핑거가 물체 위에 얹힘)')
        widths = []
        for f, settle in ((3.0, 0.3), (40.0, p['stiff_settle'])):
            g.command(f, 0)                           # 목표 폭 0, 힘만 바꿈 (물체에 막혀 그 자리에서 쥔다)
            g.wait_idle()
            time.sleep(settle)
            raw, _sp = g.settled_width()
            if not g.status()['grip']:
                raise _Fail(f'물성: {f:.0f} N 에서 못 잡음')
            widths.append(self.sg.calibrated(raw))
            self._tick()
        self._release(widths[0], obj_h)
        self._unwind_j6()
        return widths[0] - widths[1]

    # ------------------------------------------------------------------ 하강 · 힘 제어
    def _pos_descent(self, z_target, vel, thr, ignore_s, ignore_mm, safety=25.0):
        io = self.io
        time.sleep(0.3)
        (fz0, _, _), span = io.ftm(10)
        if span > 1.0:
            raise _Fail(f'정지 Fz 변동 {span:.2f} N (비정상)')
        z0 = io.cur_z()
        dist = z0 - z_target
        if dist <= 0.2:
            return 'clear', None
        io.amovel([0.0, 0.0, -dist, 0.0, 0.0, 0.0], vel=vel, acc=vel * 4)
        t0, status, hit = time.time(), 'timeout', None
        while time.time() - t0 < dist / vel + 3.0:
            d = io.ftm(1, 0.0)[0][0] - fz0
            z = io.cur_z()
            if d >= safety:
                status, hit = 'safety', z
                break
            if d >= thr and (time.time() - t0 >= ignore_s or z0 - z >= ignore_mm):
                status, hit = 'contact', z
                break
            if z <= z_target + 0.2:
                status = 'clear'
                break
            time.sleep(0.02)
        if status != 'clear':
            io.stop(1)                                             # quick stop: 덜 눌림
        io.mwait()
        return status, hit

    def _force_probe(self, z_target):
        io, p = self.io, self.p
        p0 = io.pose()
        z0 = p0[2]
        io.force_down_on()
        base, _ = io.ftm(10)
        io.force_push(p['push_force'])
        t0, hist, status, hit, lever = time.time(), [], 'clear', None, None
        try:
            while True:
                t = time.time() - t0
                if t > p['force_timeout']:
                    status = 'timeout'
                    break
                z = io.cur_z()
                hist.append((t, z))
                if z <= z_target:
                    break
                recent = [zz for (tt, zz) in hist if tt >= t - p['stall_time']]
                if t >= p['stall_time'] + 0.5 and max(recent) - min(recent) <= 0.1:
                    if z0 - z >= 1.0:
                        status, hit = 'contact', z
                    elif t >= p['stall_time'] + 3.0:
                        status = 'no_motion'
                    if status != 'clear':
                        break
                time.sleep(0.02)
            if status == 'contact':
                time.sleep(p['press_settle'])
                m, _ = io.ftm(10)
                dfz = abs(m[0] - base[0])
                if dfz > 1.0:
                    lever = ((m[1] - base[1]) / dfz * 1000.0, (m[2] - base[2]) / dfz * 1000.0)
        finally:
            io.force_off()
        time.sleep(0.3)
        pc = io.pose()
        io.movel_abs([p0[0], p0[1], pc[2] + 0.5] + p0[3:6], 10.0)   # 순응 제어 중 밀림 · 기울어짐 지우기
        return status, hit, lever

    def _lever_side(self, lever):
        p = self.p
        if lever is None:
            return 0, 0.0
        dx, dy = p['lever_dir_x'], p['lever_dir_y']
        proj = ((lever[0] - p['lever_mid_x']) * dx + (lever[1] - p['lever_mid_y']) * dy) / (math.hypot(dx, dy) or 1.0)
        if abs(proj) < p['lever_margin']:
            return 0, proj
        return (1 if proj > 0 else -1) * int(p['side_sign']), proj

    # ------------------------------------------------------------------ J6 · 높이 보조
    def _x_yaw(self):
        return pp.yaw_deg(pp.col(pp.zyz_mat(*self.io.pose()[3:6]), 0))

    def _turn_to_yaw(self, target, exact=False, prefer_j6=None):
        """세운 그리퍼의 핑거 축(툴 x)을 방위각 target 에 (exact 가 아니면 180 deg 대칭).
        같은 방향이 되는 J6 각도 중 0 ± j6_window 안에서 가장 적게 도는 쪽 (케이블이 한 방향으로만 감기지 않게)."""
        io = self.io
        wr = pp.wrap180 if exact else pp.wrap90
        e = wr(self._x_yaw() - target)                              # 툴 아래 방향: J6 + = 방위각 -
        j = io.j6()
        cands = [e + k * (360.0 if exact else 180.0) for k in (-2, -1, 0, 1, 2)]
        inside = [c for c in cands if abs(j + c) <= self.p['j6_window']]
        ok = inside or [c for c in cands if abs(j + c) <= 270.0] or cands
        d = min(ok, key=(lambda c: abs(j + c - prefer_j6)) if prefer_j6 is not None else abs)
        sgn = 1.0
        for _ in range(4):
            if abs(d) <= 0.5:
                return
            io.turn_j6(sgn * d)
            d2 = wr(self._x_yaw() - target)
            if abs(d2) > abs(d) and abs(d) < 90.0:
                sgn = -sgn
            d = d2
        if abs(d) > 0.5:
            raise _Fail(f'J6 각도 맞춤 실패 (오차 {d:+.2f} deg)')

    def _unwind_j6(self):
        """빈 그리퍼: 핑거 방향은 그대로(180 대칭) J6 를 0 에 가장 가깝게 되감는다."""
        k = round(self.io.j6() / 180.0)
        if k:
            self.io.turn_j6(-180.0 * k)

    def _tcp_open_tip(self, raw_open, tip):
        return self.sg.grasp_z_for_open(raw_open) + (tip - self.tip_clearance)

    def _safe_z(self, obj_h, clear=10.0):
        """세운 그리퍼(지금 열린 폭)의 핑거 끝이 높이 obj_h 물체 윗면보다 clear 위인 TCP z."""
        return self._tcp_open_tip(self.io.g.status()['width_raw_mm'], obj_h + clear)

    def _safe_closed_z(self, table_z, H=None):
        top = self.p['obj_h_max'] if H is None else H
        return table_z + top + self.p['close_clear']

    def _cleanup(self, H, table_z):
        """실패 · 정상 모두: 힘 제어 해제, 그리퍼 열고 안전한 높이로. CANCELED 면 robot 명령이 예외를 낸다."""
        try:
            self.io.force_off()
        except Exception:
            pass
        try:
            self.io.g.open(self.p['open_force'], self.p['open_width'])
            self.io.move_z_to(max(self.io.cur_z(), self._safe_closed_z(table_z, H)), 30.0)
        except PartError:
            raise
        except Exception as e:
            self.log.error(f'[judge] 정리 중 오류: {e}')

    def _measure_zone(self, poses):
        zone = poses.get('measure_zone')
        if zone is None or all(abs(v) < 1e-6 for v in zone):
            raise PartError('MEASURE_FAILED', '측정존 좌표 미설정')
        return list(zone)
