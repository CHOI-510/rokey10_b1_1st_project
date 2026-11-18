#!/usr/bin/env python3
"""
sync_grip.py — RG2 '닫으면서 핑거 끝 높이 유지' 집기 (공용)
================================================================
관리: 관제(정쓰). 내용은 판별 파트 rg2_sync_grip.py(11/14~17 실물 검증)를 그대로 옮긴 것이다 (11/18).
  판별(폭 측정)과 적재(측정존에서 집기)가 같이 쓴다. box_judge/rg2_sync_grip.py는 여기서 가져다 쓴다.

왜 필요한가
  RG2는 닫힐수록 핑거 끝이 아래로 내려간다 (완전히 열림 <-> 닫힘 약 39 mm).
  TCP(GripperDA_v1, z 208.4)는 '완전히 닫힌 핑거 끝' 위치라서, TCP를 바닥 근처에 두고 넓게 벌린 채 닫으면
  핑거 끝은 바닥보다 한참 위에서 박스를 문다 (11/18 실물: A 박스 60 mm 면 → 바닥 위 약 17 mm, 20 mm 박스의 위 3 mm).
  그래서 닫히는 동안 TCP를 '핑거 끝이 내려가는 만큼' 올려, 핑거 끝 높이를 바닥 근처로 일정하게 유지한다.

설정: common.SYNC_GRIP (핑거 끝 높이 2.6, 기준 원시 폭 25, 계획 폭 28, 닫는 힘 10 N) — 판별·적재 같은 값

쓰는 법 (box_comm robot = DsrIO)
  from box_comm.sync_grip import make_sync_gripper, GripMissed
  sg = make_sync_gripper(robot, 바닥z)              # 공용 설정으로 (판별은 자기 PARAMS 기본값이 같은 값)
  raw_open = robot.grip_open()                      # 100 mm
  robot.movel(... z = sg.grasp_z_for_open(raw_open) ...)
  res = sg.close_with_lift(SYNC_GRIP['est_raw_mm'])  # 빈손이면 GripMissed (RuntimeError) → 같은 자리에서 다시 (GRIP_RETRY)
"""

import math
import time

from box_comm.common import GRIPPER, SYNC_GRIP, TCP

DR_BASE = 0
DR_MV_MOD_REL = 1

EMPTY_GRIP_MM = 1.0          # (예전 기준, 쓰지 않음) 원시 폭은 빈손이어도 패드 두께(~10mm)만큼 남아 판정이 안 됨
EMPTY_WIDTH_MM = GRIPPER['empty_width_mm']   # 보정 폭(패드 사이 간격)이 이 이하면 빈손


# ---------------------------------------------------------------- 핑거 끝 높이 (onrobot-ros2 드라이버 링크 모델)
_L1, _L3, _T1, _T3, _DY = 0.108505, 0.055, 1.41371, 0.76794, -0.0144
_J_MIN, _J_MAX = -0.558505, 0.785398
_DZ1, _DZ2 = 0.1095, 0.0427


def _width_mm_to_rad(w_mm):
    arg = (w_mm / 1000.0 / 2.0 - _DY - _L1 * math.cos(_T1)) / _L3
    j = math.acos(max(-1.0, min(1.0, arg))) - _T3
    return max(_J_MIN, min(_J_MAX, j))


def tip_h_mm(raw_width_mm):
    """원시 폭에서 '플랜지부터 핑거 끝까지 거리' [mm]. 폭이 줄수록 커진다 = 닫히면서 핑거 끝이 내려간다."""
    j = _width_mm_to_rad(raw_width_mm)
    return (_L3 * math.sin(j + _T3) + _DZ1 + _DZ2) * 1000.0


# ---------------------------------------------------------------- 닫힘 시간
CLOSE_T0_S = 0.62      # 지연 (명령 · 폴링 포함)
CLOSE_V_MM_S = 31.0    # 10 N 닫힘 속도. 2026-11-14 실측: 25 / 40 / 75 mm 물체 3.0 / 2.43 / 1.26 s, 오차 0.06 s 이하


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])
    return ys[-1]


def _close_speed_mm_s(force_n, width_mm):
    """데이터시트 그리핑 속도 그래프를 눈으로 읽은 근사 (8~12 N 밖에서만 쓴다)."""
    narrow = _interp(force_n, [3, 20, 40], [12.5, 66.5, 129.5])
    wide = _interp(force_n, [3, 20, 40], [4.0, 20.0, 39.0])
    f = max(0.0, min(1.0, (width_mm - 10.0) / 100.0))
    return narrow + (wide - narrow) * f


def est_close_time(force_n, raw_from, raw_to, step=1.0):
    """원시 폭이 raw_from -> raw_to 로 줄어드는 예상 시간 [s]."""
    if 8.0 <= force_n <= 12.0:
        return CLOSE_T0_S + max(0.0, raw_from - raw_to) / CLOSE_V_MM_S
    t, w = 0.0, raw_from
    while w > raw_to + 1e-6:
        dw = min(step, w - raw_to)
        t += dw / _close_speed_mm_s(force_n, w - dw / 2.0)
        w -= dw
    return t


# ---------------------------------------------------------------- sync 닫기
class GripMissed(RuntimeError):
    """close_with_lift가 빈손으로 닫힘 (RuntimeError라 예전 코드도 그대로 잡힘). 재시도 대상"""


class SyncGripper:
    def __init__(self, g, dsr, *, table_z, grasp_height, ref_width, tcp_z=TCP['pos'][2], tip_comp=1.0, force=10.0,
                 cal_gain=1.0, cal_offset=0.30, put_vel=8.0, put_extra=0.0, settle=0.1, samples=10,
                 close_time_scale=1.0, sync_margin=1.0, log=print):
        self.g, self.dsr, self.log = g, dsr, log
        self.table_z, self.grasp_height, self.tcp_z, self.tip_comp = table_z, grasp_height, tcp_z, tip_comp
        self.ref_raw = ref_width
        self.force = force
        self.cal_gain, self.cal_offset = cal_gain, cal_offset
        self.put_vel, self.put_extra, self.settle, self.samples = put_vel, put_extra, settle, samples
        self.close_time_scale, self.sync_margin = close_time_scale, sync_margin

    def tip_clearance(self):
        """잡는 높이에서 핑거 끝 아래면이 책상 위 몇 mm 인가 (sync 가 이 높이를 유지한다)."""
        return self.grasp_height + (self.tcp_z - tip_h_mm(self.ref_raw))

    def grasp_z_for_open(self, raw_open):
        """열린 원시 폭 raw_open 에서 핑거 끝이 기준 높이가 되는 TCP z. 여기까지 내려가서 close_with_lift 를 부른다."""
        return self.table_z + self.grasp_height + self.tip_comp * (tip_h_mm(raw_open) - tip_h_mm(self.ref_raw))

    def calibrated(self, raw_mm):
        return (raw_mm - self.cal_offset) / self.cal_gain - self.g.pad_total

    def close_with_lift(self, est_width=28.0):
        """지금 높이(grasp_z_for_open)에서 닫으면서 TCP 를 올려 폭을 잰다. 그리퍼는 닫힌 채로 끝난다.
        est_width: 예상 원시 폭 [mm] (모르면 28). 결과 dict."""
        g, dsr = self.g, self.dsr
        base0 = dsr.cur_z()
        raw0 = g.status()["width_raw_mm"]
        h0 = tip_h_mm(raw0)
        raw_t = min(raw0, est_width)
        dz = self.tip_comp * (tip_h_mm(raw_t) - h0)                  # 핑거 끝이 내려갈 양 = TCP 를 올릴 양
        t_est = est_close_time(self.force, raw0, raw_t) * self.close_time_scale
        t_up = max(0.2, t_est * self.sync_margin)

        t0 = time.time()
        if dz > 0.3:
            dsr.amovel([0.0, 0.0, dz, 0.0, 0.0, 0.0], time=t_up, ref=DR_BASE, mod=DR_MV_MOD_REL)
        finished = g.grip_until_done(self.force)
        t_close = time.time() - t0
        z_contact = dsr.cur_z()                                         # 물체를 처음 잡은 높이
        stopped = False
        if dz > 0.3 and t_close < t_up:
            stopped = dsr.stop()                                        # 다 닫혔으면 남은 상승을 멈춘다
        dsr.mwait()

        # 접촉 뒤 더 올라간 만큼(물체를 끌어올린 양)을 접촉 높이까지 천천히 내려놓는다. 핑거 끝이 책상 위 1 mm 아래로는 안 간다.
        raw_f = g.status()["width_raw_mm"]
        lifted = dsr.cur_z() - z_contact
        z_floor = self.table_z + 1.0 - (self.tcp_z - tip_h_mm(raw_f))
        z_rel = max(z_contact - self.put_extra, z_floor)
        if dsr.cur_z() > z_rel + 0.3:
            dsr.move_z_to(z_rel, self.put_vel)
        time.sleep(self.settle)

        raw, spread = g.settled_width(self.samples)
        st = g.status()
        if not st["grip"] or self.calibrated(raw) <= EMPTY_WIDTH_MM:   # 11/17: 원시 폭은 빈손이어도 패드 두께만큼 남음
            raise GripMissed(f"물체가 잡히지 않음 (잡힘={st['grip']}, 원시 폭={raw:.1f})")
        width = self.calibrated(raw)
        self.log(f"폭 {width:.2f} mm (원시 {raw:.1f}, 편차 {spread:.1f}) | z {dsr.cur_z() - base0:+.1f} mm "
                 f"(계획 {dz:+.1f}) | 닫힘 {t_close:.2f} s (예상 {t_est:.2f}){' -> 상승 중단' if stopped else ''}")
        return {"width_mm": width, "raw_mm": raw, "spread_mm": spread, "t_close_s": t_close, "t_est_s": t_est,
                "planned_dz_mm": dz, "lifted_after_contact_mm": lifted, "stopped": stopped, "finished": finished}


def make_sync_gripper(robot, floor_z, log=print, **over):
    """공용 설정(common.SYNC_GRIP, GRIPPER 보정값)으로 SyncGripper 만들기 — 판별·적재 공용 (11/18).
    robot = box_comm DsrIO (그리퍼는 robot.gripper, 이동은 DsrZPort). over로 개별 값 덮어쓰기 가능"""
    kw = dict(table_z=floor_z,
              grasp_height=grasp_height_for_tip(SYNC_GRIP['tip_above_floor_mm'], SYNC_GRIP['ref_raw_mm']),
              ref_width=SYNC_GRIP['ref_raw_mm'], force=SYNC_GRIP['close_force_n'],
              cal_gain=GRIPPER['cal_gain'], cal_offset=GRIPPER['cal_offset_mm'], log=log)
    kw.update(over)
    return SyncGripper(robot.gripper, DsrZPort(robot), **kw)


def grasp_height_for_tip(tip_above_floor_mm, ref_width=25.0, tcp_z=None):
    """핑거 끝을 바닥 위 tip_above_floor_mm에 두려면 SyncGripper에 줄 grasp_height"""
    tcp_z = TCP['pos'][2] if tcp_z is None else tcp_z
    return tip_above_floor_mm - (tcp_z - tip_h_mm(ref_width))


class DsrZPort:
    """SyncGripper가 쓰는 로봇 함수(cur_z, move_z_to, amovel, stop, mwait)를 box_comm robot(DsrIO)으로.
    robot 명령이라 관제 정지 확인·진행 알림이 들어 있다."""

    def __init__(self, robot):
        self.robot = robot

    def cur_z(self):
        return float(self.robot.pose()[2])

    def move_z_to(self, z, vel):
        p = [float(v) for v in self.robot.pose()]
        p[2] = z
        self.robot.movel(p, vel=vel, acc=vel * 3)

    def amovel(self, rel6, vel=None, acc=None, time=None, ref=DR_BASE, mod=DR_MV_MOD_REL):
        self.robot.amovel_rel(rel6, vel=vel, acc=acc, time_s=time)

    def stop(self, mode=2):
        return self.robot.move_stop(mode)

    def mwait(self):
        self.robot.mwait()
