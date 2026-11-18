#!/usr/bin/env python3
"""
judge_algorithm.py — 판별 파트 메인 로직 (judge_template.py 약속에 맞춘 judge_flow_test v2)
================================================================
  측정 순서 (2026-11-17 확정)
    length1 높이  : 그리퍼를 닫고, 닫힌 핑거 끝이 테이블 위 78 mm 에서 movel 하강
                    (5 mm/s · Fz +6 N 로 대략 -> 3 mm 위에서 2 mm/s · Fz +4 N 로 정밀)
    length2 가로  : 그리퍼 각도 그대로. H 에 맞춰 덜 내려가 잡고 폭 측정 (닫히면서 z 상승, sync)
                    -> 쥔 채 3 mm 들어 J6 을 돌려 핑거 축을 J5 축과 일직선으로 -> 내려놓기
    length3 세로  : J6 +90 (레버 보정 자세) -> 힘 제어 하강 + 레버 판정으로 회피 -> 폭 측정 -> 포인트로 재정렬
    weight        : 미구현 (report 만 호출하고 None)
    judging       : 세 길이를 크기순 정렬해 DB 기준과 비교, 모두 허용 오차 안이면 그 class, 아니면 'U'

  템플릿 약속
    - JudgeAlgorithm(node, robot), judge(specs, poses, report) -> 'A'~'D' 또는 UNCLASSIFIED('U')
    - 실패는 raise PartError('MEASURE_FAILED', 이유). PartError('CANCELED') 는 잡지 않는다.
    - 단계마다 report(stage), 기록은 report.log(msg, level)
    - 로봇은 robot 으로 움직인다: robot.movel / movej / pose / force_z (문서에 있는 것만 사용)

  [관제 요청] 1~5 → 11/17 관제가 robot(DsrIO)에 추가하고 RobotPort를 robot 으로 바꿈 (알고리즘 본체는 그대로)
    1) 그리퍼: box_comm/gripper.py RG2 (이 파트 rg2_sync_grip 원본을 옮김, 세 파트 공용 Modbus)
       → robot.gripper (Goal 동안만 접속, 끝나면 box_comm이 끊음 — 다른 파트도 써야 해서)
    2) 비동기 하강·멈춤: robot.amovel_rel / robot.mwait / robot.move_stop(mode)
    3) 툴 모멘트: robot.force('tool') (Tx, Ty), Fz는 robot.force('base')
    4) 힘 제어: robot.compliance_on(stx) / robot.force_on(fd, dir, time_s) / 해제는 robot.stop()
    5) 관절각: robot.joints()
    -> 모두 관제 정지 확인·진행 알림(1초마다)이 들어 있다. 이전처럼 DSR_ROBOT2 를 직접 부르지 않는다.
  집기 실패 (11/18 관제 구현, 측정 로직은 그대로):
    - 가로·세로 sync 닫기가 빈손이면 같은 자리·같은 자세로 다시 열고 내려가 GRIP_RETRY(3)번까지 (_close_retry)
    - 가로 하강에서 핑거가 박스 위에 얹힘 → 같은 자리로 다시 내려가도 같으므로 바로 GRASP_FAILED
    - 쥔 채 3 mm 들어 돌리거나 옮길 때 놓치면(box_comm.gripper.holding, 폭 범위는 안 봄) GRASP_FAILED
    → 관제가 측정존을 비우고(지금은 fake: 로그만) 공간 스캔부터 다시
    - 폭 범위(17~80mm) 검사는 하지 않는다: 범위 밖 물체도 재서 U로 판정해야 하므로
  sync 집기 설정(핑거 끝 높이 2.6, 기준 폭 25, 닫는 힘 10 N)은 11/18부터 common.SYNC_GRIP(적재와 공용)이 기본값

  파라미터 (node.declare_parameter): table_z, grasp_height, ref_width, table_ref_file 등 — 아래 PARAMS 참고
  테이블 기준(측정존 바닥 Z): 기본은 Goal로 받은 DB 측정존 좌표의 z (poses['measure_zone'][2], 11/17 관제).
               적재와 같은 값을 쓴다. 따로 실측한 값을 쓰려면 -p table_ref_file:=~/.rokey_judge_table_ref.json
               (rokey1 judge_flow_test --table-ref measure로 저장한 파일, 측정존이 빈 상태에서 잼)
================================================================
"""

import json
import math
import os
import statistics
import time

from box_comm.judge_interface import PartError

try:
    from box_comm.common import UNCLASSIFIED
except ImportError:
    UNCLASSIFIED = 'U'

from box_judge.rg2_sync_grip import SyncGripper, tip_h_mm
from box_comm.common import GRIP_RETRY, SYNC_GRIP
from box_comm.gripper import holding
from box_comm.sync_grip import GripMissed, grasp_height_for_tip

DR_BASE = 0
DR_MV_MOD_REL = 1

# 이름: (기본값, 설명) — 2026-11-16 ~ 17 rokey1 judge_flow_test 시험값
PARAMS = {
    'table_z': (30.9, 'base 기준 책상 표면 z [mm]'),
    'grasp_height': (round(grasp_height_for_tip(SYNC_GRIP['tip_above_floor_mm'], SYNC_GRIP['ref_raw_mm']), 2),
                     '기준 폭에서 TCP 를 책상 위 몇 mm 에 (11/18 common.SYNC_GRIP 핑거 끝 2.6 → 0.5)'),
    'ref_width': (SYNC_GRIP['ref_raw_mm'], 'grasp_height 를 맞춘 원시 폭 [mm] (common.SYNC_GRIP)'),
    'tcp_z': (208.4, 'TCP GripperDA_v1 z 오프셋'),
    'obj_h_max': (75.0, '최대 물체 높이 [mm]'),
    'approach_clear': (10.0, '열린 핑거로 내려가기 시작할 때 핑거 끝 - 윗면 [mm]'),
    'body_clear': (31.0, '핑거 끝 위로 들어올 수 있는 물체 높이 [mm] (데이터시트 36 - 여유 5)'),
    'clear_after': (5.0, '내려놓은 뒤 열린 핑거 끝 - 윗면 [mm]'),
    'close_clear': (10.0, '빈 그리퍼 닫기 전 닫힌 핑거 끝 - 최대 물체 [mm]'),
    'open_width': (100.0, '열 때 패드 간격 [mm]'),
    'open_force': (20.0, '여는 힘 [N]'),
    'grip_force': (SYNC_GRIP['close_force_n'], '폭 잴 때 닫는 힘 [N] (닫힘 시간식 8~12 N, common.SYNC_GRIP)'),
    'cal_gain': (1.0, '폭 보정 기울기 (패드 장착, 실제 = 원시 - 10.30)'),
    'cal_offset': (0.30, '폭 보정 오프셋 [mm]'),
    'h_start': (78.0, '높이: 닫힌 핑거 끝 테이블 위 출발 높이 [mm]'),
    'h_fast_vel': (5.0, '높이 1차 속도 [mm/s]'),
    'h_fast_thr': (6.0, '높이 1차 판정 Fz [N]'),
    'h_backoff': (3.0, '1차 접촉 뒤 상승 [mm]'),
    'h_vel': (2.0, '높이 2차 속도 [mm/s]'),
    'h_thr': (4.0, '높이 2차 판정 Fz [N]'),
    'p1_vel': (10.0, '가로 하강 속도 [mm/s]'),
    'p1_thr': (6.0, '가로 하강 중 이 Fz 면 핑거가 얹힌 것 -> 실패 [N]'),
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
    'shift': (12.0, '회피 한 번 이동 [mm]'),
    'max_iter': (8, '회피 최대 횟수'),
    'max_shift': (40.0, '포인트에서 최대 이탈 [mm]'),
    'tol_mm': (3.0, '분류 허용 오차 [mm] (세 길이 모두 이 안이어야 그 class, 아니면 U)'),
    'speed': (60.0, '일반 직선 이동 속도 [mm/s]'),
    'gripper_ip': ('192.168.1.1', 'RG2 Compute Box'),
    'table_ref_file': ('', '테이블 기준 파일 (빈 값 = DB 측정존 z 사용, 11/17)'),
}


class _Fail(Exception):
    pass


def _zyz(A, B, C):
    a, b, c = (math.radians(v) for v in (A, B, C))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc, sb * sc, cb]]


class RobotPort:
    """판별에 필요한 로봇 · 그리퍼 명령 모음 — 전부 robot(DsrIO)으로 (11/17 관제 연동).
    robot 명령에는 관제 정지 확인 · 진행 알림이 들어 있다. 그리퍼(RG2 Modbus)는 Goal 동안만 접속(robot.gripper)."""

    def __init__(self, node, robot, p):
        self.node, self.robot, self.p = node, robot, p

    @property
    def g(self):
        return self.robot.gripper                    # 이번 Goal의 RG2 (처음 부를 때 접속)

    @property
    def pad_total(self):
        return self.g.pad_total

    def close(self):
        pass                                         # 접속 해제는 box_comm이 Goal 끝에 한다

    # ---- 위치 · 이동
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
        """툴 x 방향으로 d [mm] (툴 자세에서 base 방향을 계산해 절대 movel)."""
        p = self.pose()
        R = _zyz(*p[3:6])
        p[0] += R[0][0] * d
        p[1] += R[1][0] * d
        self.movel_abs(p, vel)

    def fz(self):
        return float(self.robot.force_z())

    def mwait(self):
        self.robot.mwait()

    def amovel(self, rel6, vel=None, acc=None, time=None, ref=DR_BASE, mod=DR_MV_MOD_REL):
        """BASE 기준 상대 비동기 이동 (판별·SyncGripper는 이 형태만 씀)"""
        self.robot.amovel_rel(rel6, vel=vel, acc=acc, time_s=time)

    def stop(self, mode=2):
        """지금 이동만 멈춤 (1 빠른 정지, 2 부드러운 정지). 성공 여부 반환"""
        return self.robot.move_stop(mode)

    # ---- 관절
    def j6(self):
        return float(self.robot.joints()[5])

    def turn_j6(self, d):
        if abs(d) > 0.05:
            q = self.robot.joints()
            q[5] += d
            self.robot.movej(q, vel=60, acc=120)

    # ---- 힘 · 힘 제어
    def ftm(self, n, interval=0.03):
        """(base Fz, 툴 Tx, 툴 Ty) 평균, Fz 변동 폭."""
        fz, tx, ty = [], [], []
        for _ in range(n):
            fb = self.robot.force('base')
            ft = self.robot.force('tool')
            fz.append(float(fb[2]))
            tx.append(float(ft[3]))
            ty.append(float(ft[4]))
            time.sleep(interval)
        return (statistics.mean(fz), statistics.mean(tx), statistics.mean(ty)), max(fz) - min(fz)

    def force_down_on(self, push):
        self.robot.compliance_on([2000, 2000, 2000, 50, 50, 50])     # 기준 좌표계는 robot이 BASE로 맞춰 둠
        time.sleep(0.5)

    def force_push(self, push):
        self.robot.force_on([0, 0, -abs(push), 0, 0, 0], [0, 0, 1, 0, 0, 0], time_s=0.0)

    def force_off(self):
        self.robot.stop()                            # 힘 · 순응제어 해제 + 멈춤 확인


class JudgeAlgorithm:
    def __init__(self, node, robot):
        self.node = node
        self.log = node.get_logger()
        self.robot = robot
        self.p = {}
        for k, (v, desc) in PARAMS.items():
            node.declare_parameter(k, v)
            self.p[k] = node.get_parameter(k).value
        p = self.p
        self.tip_clearance = p['grasp_height'] + (p['tcp_z'] - tip_h_mm(p['ref_width']))
        self.io = RobotPort(node, robot, p) if robot is not None else None
        self.sg = None                               # Goal마다 만듦 (그리퍼 접속이 Goal 단위라서, 11/17)
        self.log.info(f'[judge] 준비: 핑거 끝 기본 높이 {self.tip_clearance:.1f} mm')

    def _make_sync_gripper(self):
        p = self.p
        self.sg = SyncGripper(self.io.g, self.io, table_z=p['table_z'], grasp_height=p['grasp_height'],
                              ref_width=p['ref_width'], tcp_z=p['tcp_z'], force=p['grip_force'],
                              cal_gain=p['cal_gain'], cal_offset=p['cal_offset'], put_vel=p['put_vel'],
                              log=self.log.info)
        self.log.info(f'[judge] 그리퍼 접속: 패드 두께 합 {self.io.pad_total:.1f} mm')

    # ------------------------------------------------------------------ 약속된 함수
    def judge(self, specs, poses, report) -> str:
        if self.io is None:
            raise PartError('MEASURE_FAILED', '로봇 객체 없음 (두산 패키지 미설치)')
        self._make_sync_gripper()
        zone = self._measure_zone(poses)
        home = poses['home_j']
        table_z = self._table_ref(zone)
        px, py = zone[0], zone[1]
        io, p = self.io, self.p
        self._report = report
        H = None
        try:
            io.move_z_to(max(io.cur_z(), self._safe_closed_z(table_z)), p['speed'])
            io.move_xy_to(px, py, p['speed'])

            report('measuring_length1')
            H = self._measure_height(table_z)
            report.log(f'높이 {H:.1f} mm')

            report('measuring_length2')
            W, tip = self._measure_width(H)
            self._turn_and_place(px, py, H, W)
            report.log(f'가로 {W:.1f} mm (핑거 끝 {tip:.1f} mm 에서 잡음), J5 일직선으로 돌려 내려놓음')

            report('measuring_length3')
            D, n_avoid = self._avoid_and_depth(px, py, H, tip)
            report.log(f'세로 {D:.1f} mm (회피 {n_avoid}회)')

            report('measuring_weight')
            weight = None  # 미구현
        except _Fail as e:
            raise PartError('MEASURE_FAILED', str(e))
        finally:
            self._cleanup(H, table_z)

        lengths = [H, W, D]
        self.log.info(f'[judge] 측정값 H={H:.1f} W={W:.1f} D={D:.1f} mm')
        report.log(f'측정값 {H:.1f} x {W:.1f} x {D:.1f} mm')

        report('judging')
        result = self.classify(specs, lengths, weight)
        report.log(f'판별 결과 {result}')
        self.robot.movej(home, vel=30, acc=30)
        return result

    # ------------------------------------------------------------------ 판별
    def classify(self, specs, lengths, weight) -> str:
        """세 길이를 크기순으로 정렬해 기준과 비교. 세 변 모두 tol_mm 안인 기준 중 오차 합이 가장 작은 것, 없으면 U."""
        m = sorted(lengths)
        best, best_err = None, None
        for s in specs:
            ref = sorted([s['width_mm'], s['depth_mm'], s['height_mm']])
            errs = [abs(a - b) for a, b in zip(m, ref)]
            if max(errs) <= self.p['tol_mm'] and (best_err is None or sum(errs) < best_err):
                best, best_err = s['class_id'], sum(errs)
        return best if best is not None else UNCLASSIFIED

    # ------------------------------------------------------------------ 측정 단계
    def _measure_height(self, table_z):
        io, p = self.io, self.p
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
                if s in ('contact', 'safety'):        # 같은 자리로 다시 내려가도 같음 → 측정존 비우고 재스캔 (11/18)
                    raise PartError('GRASP_FAILED', f'가로 하강 중 {s} (핑거가 물체 위에 얹힘)')
                raise _Fail(f'가로 하강 중 {s}')

        descend()
        r = self._close_retry('가로', descend)
        return r['width_mm'], tip

    def _close_retry(self, what, redescend):
        """sync 닫기. 빈손이면 같은 자리·자세로 다시 열고 내려가(redescend) GRIP_RETRY번까지 (11/18).
        다 실패하면 GRASP_FAILED → 관제가 측정존을 비우고 재스캔"""
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
        """쥔 채 들어 올린 뒤 놓쳤는지 (잡힘 비트 + 빈손 아님 + 폭 변화 3mm 이내, 폭 범위는 안 봄)"""
        ok, why, _w = holding(self.io.g, width, check_range=False)
        if not ok:
            raise PartError('GRASP_FAILED', f'{where} 놓침: {why}')

    def _turn_and_place(self, px, py, H, W):
        io, p = self.io, self.p
        z_hold = io.cur_z()
        io.move_z_to(z_hold + p['carry_lift'], p['put_vel'])
        self._check_holding(W, '가로 잰 뒤 들어 올리다')
        e = self._finger_err(px, py)
        sgn = 1.0
        for _ in range(4):                                         # 핑거 축(툴 x)을 팔 방향에 수직 = J5 축과 일직선
            if abs(e) <= 0.5:
                break
            io.turn_j6(sgn * e)
            e2 = self._finger_err(px, py)
            if abs(e2) > abs(e):
                sgn = -sgn
            e = e2
        if abs(e) > 0.5:
            raise _Fail(f'J5 일직선 맞춤 실패 ({e:+.2f} deg)')
        io.move_z_to(z_hold, p['put_vel'])
        raw_open = io.g.open(p['open_force'], p['open_width'])
        io.move_z_to(max(io.cur_z(), self._tcp_open_tip(raw_open, H + p['clear_after'])), 30.0)

    def _avoid_and_depth(self, px, py, H, tip):
        io, p = self.io, self.p
        io.turn_j6(90.0)
        if abs(io.j6() - (-1.1)) > 3.0:
            self.log.warn(f'[judge] 회피 자세 J6 {io.j6():+.1f} 가 레버 보정 자세(-1.1)와 다름')
        raw_open = io.g.status()['width_raw_mm']                  # 내려놓을 때 연 폭 그대로
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

        def redescend():                                           # 같은 자리(회피 끝난 위치)에서 다시 열고 내려감
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
            io.stop(1)                                             # quick stop: 덜 눌림 (soft 는 안전 정지에 걸렸다)
        io.mwait()
        return status, hit

    def _force_probe(self, z_target):
        io, p = self.io, self.p
        p0 = io.pose()
        z0 = p0[2]
        io.force_down_on(p['push_force'])
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

    # ------------------------------------------------------------------ 보조
    def _tcp_open_tip(self, raw_open, tip):
        return self.sg.grasp_z_for_open(raw_open) + (tip - self.tip_clearance)

    def _safe_closed_z(self, table_z, H=None):
        top = self.p['obj_h_max'] if H is None else H
        return table_z + top + self.p['close_clear']

    def _finger_err(self, px, py):
        R = _zyz(*self.io.pose()[3:6])
        ang_f = math.degrees(math.atan2(R[1][0], R[0][0]))
        ang_t = math.degrees(math.atan2(py, px)) + 90.0
        return (ang_f - ang_t + 90.0) % 180.0 - 90.0

    def _table_ref(self, zone):
        """측정존 바닥 Z. 기본은 DB 측정존 z(적재와 같은 값), table_ref_file을 주면 그 실측값."""
        path = str(self.p['table_ref_file'] or '').strip()
        if not path:
            self.log.info(f'[judge] 테이블 기준 = DB 측정존 z {zone[2]:.2f} mm')
            return float(zone[2])
        f = os.path.expanduser(path)
        if not os.path.exists(f):
            raise PartError('MEASURE_FAILED', f'테이블 기준 파일 없음 ({f}): 측정존을 비우고 기준을 먼저 잴 것 '
                                              '(또는 table_ref_file을 비워 DB 측정존 z 사용)')
        with open(f) as fh:
            ref = json.load(fh)
        if ref.get('pad_total') is not None and abs(float(ref['pad_total']) - self.io.pad_total) > 0.2:
            self.log.warn('[judge] 테이블 기준을 잴 때와 패드 두께가 다르다 -> 다시 잴 것')
        self.log.info(f'[judge] 테이블 기준 = 파일 {f}: {float(ref["z"]):.2f} mm')
        return float(ref['z'])

    def _cleanup(self, H, table_z):
        """실패 · 정상 모두: 그리퍼를 열고 안전한 높이로. 정지 요청(CANCELED)이면 robot 명령이 예외를 낸다."""
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
