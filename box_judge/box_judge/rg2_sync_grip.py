#!/usr/bin/env python3
# rg2_sync_grip.py
# RG2 로 물체 폭을 잴 때 쓰는 '닫히면서 z 축으로 올라가는' 동작(sync 방식)만 떼어 낸 독립 모듈.
#
# 왜 필요한가
#   RG2 는 닫힐수록 핑거 끝이 아래로 내려간다 (완전히 열림 <-> 닫힘 약 39 mm).
#   TCP 높이를 고정한 채 닫으면 핑거 끝이 책상에 닿거나, 반대로 넓게 열린 상태에 맞추면 얇은 물체를 못 잡는다.
#   그래서 닫히는 동안 TCP 를 '핑거 끝이 내려가는 만큼' 올려, 핑거 끝 높이를 일정하게 유지한다.
#
# 동작 (close_with_lift)
#   1) 지금 폭(원시)과 예상 물체 폭으로 '닫히는 동안 핑거 끝이 내려갈 양' dz 와 '닫히는 시간' t 를 계산한다.
#   2) amovel(+dz, time=t) 로 한 번에 올리면서 동시에 닫기 명령을 보낸다.
#      (폭을 읽으며 짧은 이동을 반복하는 방식은 로봇이 심하게 떨려서 쓰지 않는다)
#   3) 그리퍼가 다 닫히면(물체 접촉) 상승이 남아 있어도 MoveStop(soft stop)으로 멈춘다 -> 물체를 끌어올리지 않는다.
#   4) 접촉 뒤 더 올라간 만큼 천천히 내려놓고(바닥 하한 있음), 폭이 안정되면 중앙값을 읽어 보정한다.
#
# 필요한 것
#   - pymodbus (apt: python3-pymodbus), Doosan ROS2 (DSR_ROBOT2, dsr_msgs2)
#   - RG2 Compute Box Modbus TCP 192.168.1.1:502, 장치 주소 65. **동시 접속은 1개뿐**:
#     OnRobot 드라이버(m0609_rg2_bringup real 모드, OnRobotRGControllerServer)가 떠 있으면 내리고 쓴다.
#   - 툴 TCP 'GripperDA_v1' (z 208.4 mm) 이 등록 · 적용되어 있을 것.
#
# 폭의 기준 (중요)
#   - 원시 폭(레지스터 267): 알루미늄 핑거 사이. 핑거 끝 높이 계산은 이 값으로 한다.
#   - 반영 폭(레지스터 275): 원시 폭 - 핑거팁 오프셋 x 2 = 패드 사이 실제 간격. 열기 명령은 이 기준이다.
#   - 결과 폭 = (원시 폭 - cal_offset) / cal_gain - 패드 두께 합.
#     기본 보정 1.0 · 0.30 은 패드 장착 상태에서 젠가 29 mm · 75 mm 두 점으로 구했다(2026-11-17): 실제 = 원시 - 10.30.
#     (패드 없을 때의 값은 1.0401 · 2.09 였다.) 패드 두께 합은 접속할 때 (원시 - 반영) 으로 읽는다.
#     패드를 바꾸면 블록으로 다시 구할 것. 세 번째 크기로는 아직 검증 안 함.
#
# 높이 인자 (rg2_width_robot 과 같은 뜻)
#   table_z      : base 기준 책상 표면 z [mm]
#   grasp_height : 기준 폭(ref_width)에서 TCP 를 책상 위 몇 mm 에 둘지. 실제 핑거 끝 높이 = grasp_height + (tcp_z - tip_h(ref))
#   ref_width    : grasp_height 를 맞춘 원시 폭 [mm]
#   사용 중인 값 예: table_z 30.9, grasp_height 0.5, ref_width 25 -> 핑거 끝이 책상 위 약 2.6 mm
#
# 한계
#   - 닫힘 시간식(0.62 s + 거리 / 31 mm/s)은 힘 8~12 N 에서만 실측했다. 그 밖은 데이터시트 그래프 근사.
#   - 핑거 끝 높이는 드라이버 링크 모델 값(도면과 약 1 mm 차이).
#   - 예상 폭을 모르면 28 mm(원시)로 계획하고, 일찍 닿으면 멈춘다(검증된 기본값).
#
# 단독 실행 (현재 자세 아래 물체 한 번 재기)
#   ros2 run box_judge rg2_sync_grip --table-z 30.9 --grasp-height 0.5 --ref-width 25 --pick-xy 300 -160
#
# 다른 코드에서 쓰기
#   from box_judge.rg2_sync_grip import RG2, DsrHandle, SyncGripper   (RG2는 box_comm.gripper에서 옴)
#   g = RG2("192.168.1.1", 502)
#   dsr = DsrHandle(node)                      # DR_init 설정과 노드 생성 뒤
#   sg = SyncGripper(g, dsr, table_z=30.9, grasp_height=0.5, ref_width=25.0, log=node.get_logger().info)
#   raw_open = g.open(20.0, 100.0)             # 패드 간격 100 mm 로 열기
#   dsr.move_z_to(sg.grasp_z_for_open(raw_open), 30.0)
#   res = sg.close_with_lift(est_width=28.0)   # res["width_mm"] = 보정된 물체 폭
#   g.open(20.0, 100.0)
#   g.close()

import argparse
import math
import sys
import time

# ---------------------------------------------------------------- RG2 Modbus
# 11/17 (관제): RG2 클래스는 box_comm/gripper.py로 옮겼다 (탐지·판별·적재 공용, 내용은 이 파일 원본 그대로).
#   판별 알고리즘은 robot.gripper(Goal 동안 접속)로 받고, 아래 단독 실행(main)은 RG2를 직접 만든다.
from box_comm.gripper import RG2  # noqa: E402,F401

EMPTY_GRIP_MM = 1.0          # 원시 폭이 이보다 작으면 물체 없이 닫힌 것으로 본다


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


# ---------------------------------------------------------------- 로봇 쪽 (DSR_ROBOT2)
DR_BASE = 0
DR_MV_MOD_REL = 1


class DsrHandle:
    """close_with_lift 에 필요한 로봇 함수 묶음. DR_init 에 노드를 넣은 뒤에 만든다."""

    def __init__(self, node):
        import rclpy
        from DSR_ROBOT2 import amovel, movel, mwait, get_current_posx
        from dsr_msgs2.srv import MoveStop
        self._rclpy, self._node = rclpy, node
        self.amovel, self.movel, self.mwait, self._posx = amovel, movel, mwait, get_current_posx
        self._MoveStop = MoveStop
        self._stop_cli = node.create_client(MoveStop, "dsr_controller2/motion/move_stop")

    def cur_z(self):
        p, _sol = self._posx(ref=DR_BASE)
        return float(p[2])

    def move_z_to(self, z, vel):
        self.movel([0.0, 0.0, z - self.cur_z(), 0.0, 0.0, 0.0], vel=vel, acc=vel * 3, ref=DR_BASE, mod=DR_MV_MOD_REL)
        self.mwait()

    def stop(self, mode=2):
        """진행 중인 비동기 이동을 멈춘다. mode 2 = soft stop(DR_SSTOP), 1 = quick stop(DR_QSTOP). 성공 여부 반환."""
        if not self._stop_cli.wait_for_service(timeout_sec=1.0):
            return False
        req = self._MoveStop.Request()
        req.stop_mode = mode
        fut = self._stop_cli.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=2.0)
        return fut.result() is not None and bool(fut.result().success)


# ---------------------------------------------------------------- sync 닫기
class SyncGripper:
    def __init__(self, g, dsr, *, table_z, grasp_height, ref_width, tcp_z=208.4, tip_comp=1.0, force=10.0,
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
        if not st["grip"] or raw <= EMPTY_GRIP_MM:
            raise RuntimeError(f"물체가 잡히지 않음 (잡힘={st['grip']}, 원시 폭={raw:.1f})")
        width = self.calibrated(raw)
        self.log(f"폭 {width:.2f} mm (원시 {raw:.1f}, 편차 {spread:.1f}) | z {dsr.cur_z() - base0:+.1f} mm "
                 f"(계획 {dz:+.1f}) | 닫힘 {t_close:.2f} s (예상 {t_est:.2f}){' -> 상승 중단' if stopped else ''}")
        return {"width_mm": width, "raw_mm": raw, "spread_mm": spread, "t_close_s": t_close, "t_est_s": t_est,
                "planned_dz_mm": dz, "lifted_after_contact_mm": lifted, "stopped": stopped, "finished": finished}


# ---------------------------------------------------------------- 단독 실행
def main(args=None):
    import rclpy
    import DR_init
    from rclpy.utilities import remove_ros_args

    ap = argparse.ArgumentParser(description="RG2 sync 닫기로 폭 한 번 재기")
    ap.add_argument("--table-z", type=float, required=True)
    ap.add_argument("--grasp-height", type=float, default=0.5)
    ap.add_argument("--ref-width", type=float, default=25.0)
    ap.add_argument("--pick-xy", type=float, nargs=2, default=None, help="홈 높이에서 이 XY 로 수평 이동")
    ap.add_argument("--open-width", type=float, default=100.0, help="패드 간격 [mm]")
    ap.add_argument("--open-force", type=float, default=20.0)
    ap.add_argument("--force", type=float, default=10.0, help="닫는 힘 [N]")
    ap.add_argument("--est-width", type=float, default=28.0, help="예상 원시 폭 [mm]")
    ap.add_argument("--tcp", default="GripperDA_v1")
    ap.add_argument("--tool", default="Tool Weight")
    ap.add_argument("--ip", default="192.168.1.1")
    a = ap.parse_args(remove_ros_args(args=sys.argv)[1:])

    DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "m0609"
    rclpy.init(args=args)
    node = rclpy.create_node("rg2_sync_grip", namespace="dsr01")
    DR_init.__dsr__node = node
    from DSR_ROBOT2 import movej, movel, mwait, set_tcp, set_tool, get_current_posx
    from DR_common2 import posj
    log = node.get_logger().info
    g = None
    try:
        set_tcp(a.tcp)
        set_tool(a.tool)
        dsr = DsrHandle(node)
        g = RG2(a.ip)
        sg = SyncGripper(g, dsr, table_z=a.table_z, grasp_height=a.grasp_height, ref_width=a.ref_width,
                         force=a.force, log=log)
        log(f"패드 두께 합 {g.pad_total:.1f} mm | 잡는 높이에서 핑거 끝 책상 위 {sg.tip_clearance():.1f} mm")
        movej(posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0]), vel=30, acc=30)
        if a.pick_xy:
            p, _ = get_current_posx(ref=DR_BASE)
            movel([a.pick_xy[0], a.pick_xy[1], float(p[2])] + [float(v) for v in p[3:6]], vel=100, acc=300, ref=DR_BASE)
            mwait()
        raw_open = g.open(a.open_force, a.open_width)
        z_hold = dsr.cur_z()
        gz = sg.grasp_z_for_open(raw_open)
        input(f"물체를 그리퍼 아래에 놓고 Enter -> TCP z {gz:.1f} 까지 내려가 잽니다 (취소 Ctrl+C): ")
        dsr.move_z_to(gz + 15.0, 100.0)
        dsr.move_z_to(gz, 30.0)
        res = sg.close_with_lift(a.est_width)
        print(f"\n측정 폭: {res['width_mm']:.2f} mm")
        g.open(a.open_force, a.open_width)
        dsr.move_z_to(z_hold, 100.0)
    except KeyboardInterrupt:
        pass
    finally:
        if g is not None:
            g.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
