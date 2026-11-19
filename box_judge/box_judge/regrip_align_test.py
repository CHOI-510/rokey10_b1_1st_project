#!/usr/bin/env python3
# regrip_align_test.py  (2026-11-19) 대각 잡기 보정 단독 시험 — '비틀어 다시 잡기'
#
# 문제: 측정존에 물체가 돌아가 있으면(45° 근처) 평행 그리퍼의 두 패드가 모서리를 대칭으로 눌러
#       돌리는 힘이 거의 없어, 대각인 채로 잡힌다 -> 대각 폭을 재서 미분류(U).
# 시험하는 보정 (시행 한 번):
#   [1] 높이: 닫은 그리퍼로 내려가 윗면 접촉 (5 mm/s · 6 N -> 3 mm 올라가 2 mm/s · 4 N, judge_algorithm 과 같은 값)
#       --obj-h 를 주면 생략
#   [2] 그리퍼 각도 그대로 sync 로 잡아 폭 W1
#   [3] 열고, 핑거 끝이 윗면 위로 올라간 뒤 J6 를 --twist(15°) 돌려, 다시 내려가 sync 로 잡아 폭 W2
#       45° 였다면 30° 가 되어 균형이 깨지고 가까운 면으로 돌아 정렬된다. 정렬돼 있었다면 따라 돌아 W2 ≈ W1.
#   [4] (--check-d) J6 를 90° 더 돌려 같은 식으로 직교 방향 폭 D 도 잰다
#   [5] 열고 올라가 J6 를 처음 각도로 되돌림. 비트는 방향은 시행마다 번갈아 (툴 케이블)
#   --expect W D 를 주면 W2(와 D)가 두 변 중 하나와 ± --tol 안인지로 정렬 여부(O/X)를 판정한다.
#
# 이 코드는 관제 스택 없이 DSR_ROBOT2 를 직접 부른다 (rg2_sync_grip 단독 실행과 같은 방식).
#   - judge_main 등 팀 노드와 동시에 돌리지 않는다.
#   - 그리퍼 Modbus 접속은 1개뿐: 브링업이 띄운 OnRobot 드라이버를 먼저 끈다.
#   - 위치 · 높이 기본값은 box_comm/common.py (측정존 FIXED_POSES, TCP, TOOL, SYNC_GRIP, GRIPPER) 를 그대로 쓴다.
#     측정존 바닥 z 하나를 높이 측정 · sync 집기에 같이 쓴다 (judge_algorithm 11/18 관제 수정과 같음).
#
# 실행 (bringup real 모드 + `pkill -f OnRobotRGControllerServer` 뒤)
#   ros2 run box_judge regrip_align_test --expect 50 40 --check-d --repeat 6 --ask-angle
#   ros2 run box_judge regrip_align_test --twist 25 --expect 50 40 --check-d --repeat 6 --ask-angle
#   ros2 run box_judge regrip_align_test --obj-h 20 --expect 60 40 --repeat 3        (높이를 알면 높이 측정 생략)
#   물체를 측정존 중앙에 0 · 20 · 45° 로 돌려 놓고 두 번씩. 45° 가 정렬되지 않으면 --twist 를 키운다.
# 결과: 터미널 표 + ~/regrip_align_날짜_시각.csv

import argparse
import csv
import os
import statistics
import sys
import time

from box_comm.common import FIXED_POSES, GRIPPER, ROBOT, SYNC_GRIP, TCP, TOOL
from box_comm.gripper import RG2
from box_comm.sync_grip import GripMissed, SyncGripper, grasp_height_for_tip

DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1
FIELDS = ['trial', 'place_angle', 'twist_deg', 'H_mm', 'W1_mm', 'W2_mm', 'W1_minus_W2', 'D_mm', 'aligned',
          'result', 'reason', 't_s']


class Fail(Exception):
    pass


def parse(argv):
    zone = FIXED_POSES['measure_zone']
    ap = argparse.ArgumentParser(description='대각 잡기 보정 시험 (비틀어 다시 잡기)')
    ap.add_argument('--twist', type=float, default=15.0, help='다시 잡기 전 J6 비트는 각 [deg]')
    ap.add_argument('--expect', type=float, nargs=2, default=None, metavar=('W', 'D'),
                    help='물체의 두 수평 변 [mm] (주면 정렬 여부 판정)')
    ap.add_argument('--tol', type=float, default=2.0, help='정렬 판정 허용 오차 [mm]')
    ap.add_argument('--check-d', action='store_true', help='J6 +90 으로 직교 방향 폭도 잰다')
    ap.add_argument('--obj-h', type=float, default=None, help='물체 높이 [mm] (주면 높이 측정 생략)')
    ap.add_argument('--repeat', type=int, default=3, help='시행 횟수')
    ap.add_argument('--ask-angle', action='store_true', help='시행마다 놓은 각도를 입력받아 CSV 에 남김')
    ap.add_argument('--yes', action='store_true', help='Enter 확인 없이 모든 시행 진행')
    ap.add_argument('--pick-xy', type=float, nargs=2, default=list(zone[:2]), help='시험 위치 (측정존)')
    ap.add_argument('--floor-z', type=float, default=zone[2],
                    help='측정존 바닥 z [mm]: 높이 측정 · sync 집기 공통 (judge 와 같은 DB 값)')
    ap.add_argument('--obj-h-max', type=float, default=75.0)
    ap.add_argument('--body-clear', type=float, default=31.0, help='핑거 끝 위로 들어올 수 있는 물체 높이 [mm]')
    ap.add_argument('--open-width', type=float, default=GRIPPER['open_width_mm'])
    ap.add_argument('--open-force', type=float, default=GRIPPER['open_force_n'])
    ap.add_argument('--grip-vel', type=float, default=25.0, help='윗면 아래로 잡으러 내려가는 속도 [mm/s]')
    ap.add_argument('--grip-thr', type=float, default=12.0, help='그때 핑거가 얹힌 것으로 보는 Fz [N]')
    ap.add_argument('--speed', type=float, default=60.0)
    return ap.parse_args(argv)


def main(args=None):
    import rclpy
    import DR_init
    from rclpy.utilities import remove_ros_args
    a = parse(remove_ros_args(args=sys.argv)[1:])

    DR_init.__dsr__id, DR_init.__dsr__model = ROBOT['id'], ROBOT['model']
    rclpy.init(args=args)
    node = rclpy.create_node('regrip_align_test', namespace=ROBOT['id'])
    DR_init.__dsr__node = node
    from DSR_ROBOT2 import movej, movel, amovel, mwait, set_tcp, set_tool, get_tool_force, \
        get_current_posx, get_current_posj
    from DR_common2 import posj
    from box_judge.rg2_sync_grip import DsrHandle
    log = node.get_logger()
    g = None
    rows = []

    def cur_z():
        p, _s = get_current_posx(ref=DR_BASE)
        return float(p[2])

    def j6():
        return float(get_current_posj()[5])

    def turn_j6(d):
        if abs(d) > 0.05:
            q = [float(v) for v in get_current_posj()]
            q[5] += d
            movej(posj(q), vel=60, acc=120)

    def read_fz(n=10):
        vs = []
        for _ in range(n):
            vs.append(float(get_tool_force(ref=DR_BASE)[2]))
            time.sleep(0.03)
        return statistics.mean(vs), max(vs) - min(vs)

    def pos_descent(z_target, vel, thr, ignore_s=0.4, ignore_mm=1.0, safety=25.0):
        """위치 제어로 내려가며 Fz 만 읽는다 (judge_algorithm._pos_descent 와 같은 판정)."""
        time.sleep(0.3)
        fz0, span = read_fz()
        if span > 1.0:
            raise Fail(f'정지 Fz 변동 {span:.2f} N (비정상)')
        z0 = cur_z()
        dist = z0 - z_target
        if dist <= 0.2:
            return 'clear', None
        amovel([0.0, 0.0, -dist, 0.0, 0.0, 0.0], vel=vel, acc=vel * 4, ref=DR_BASE, mod=DR_MV_MOD_REL)
        t0, status, hit = time.time(), 'timeout', None
        while time.time() - t0 < dist / vel + 3.0:
            d = float(get_tool_force(ref=DR_BASE)[2]) - fz0
            z = cur_z()
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
            dsr.stop(1)
        mwait()
        return status, hit

    try:
        dsr = DsrHandle(node)
        if not dsr._stop_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('dsr_controller2 서비스가 없음: 브링업 확인')
        set_tcp(TCP['name'])
        set_tool(TOOL['name'])
        g = RG2()
        sg = SyncGripper(g, dsr, table_z=a.floor_z,
                         grasp_height=grasp_height_for_tip(SYNC_GRIP['tip_above_floor_mm'], SYNC_GRIP['ref_raw_mm']),
                         ref_width=SYNC_GRIP['ref_raw_mm'], tcp_z=TCP['pos'][2], force=SYNC_GRIP['close_force_n'],
                         cal_gain=GRIPPER['cal_gain'], cal_offset=GRIPPER['cal_offset_mm'], log=log.info)
        tip_clear = sg.tip_clearance()
        log.info(f'패드 두께 합 {g.pad_total:.1f} mm | 잡는 높이에서 핑거 끝 바닥 위 {tip_clear:.1f} mm')
        px, py = a.pick_xy
        movej(posj(FIXED_POSES['home_j']), vel=60, acc=60)
        p, _s = get_current_posx(ref=DR_BASE)
        movel([px, py, float(p[2])] + [float(v) for v in p[3:6]], vel=a.speed, acc=a.speed * 3,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)
        mwait()
        j6_home = j6()

        def tcp_tip(raw, tip):
            """열린 원시 폭 raw 에서 핑거 끝이 바닥 위 tip 이 되는 TCP z."""
            return sg.grasp_z_for_open(raw) + (tip - tip_clear)

        def grasp(H, what):
            """열고 핑거 끝이 윗면 + 10 위 -> 잡는 높이까지 (얹히면 멈춤) -> sync 로 닫아 폭."""
            raw_open = g.open(a.open_force, a.open_width)
            tip = max(tip_clear, H - a.body_clear)
            z_above = tcp_tip(raw_open, H + 10.0)
            if cur_z() < z_above:
                dsr.move_z_to(z_above, 30.0)
            s, _z = pos_descent(tcp_tip(raw_open, tip), a.grip_vel, a.grip_thr)
            if s != 'clear':
                dsr.move_z_to(z_above, 30.0)
                raise Fail(f'{what}: 내려가다 {s} (핑거가 물체 위에 얹힘)')
            try:
                return sg.close_with_lift(SYNC_GRIP['est_raw_mm'])['width_mm']
            except GripMissed as e:
                raise Fail(f'{what}: 못 잡음 ({e})')

        def rise_open(H):
            raw = g.open(a.open_force, a.open_width)
            dsr.move_z_to(max(cur_z(), tcp_tip(raw, H + 10.0)), 30.0)

        def measure_height():
            g.grip_until_done(SYNC_GRIP['close_force_n'])          # 최대 물체 위에서 빈 그리퍼 닫기
            dsr.move_z_to(a.floor_z + 78.0, a.speed)
            sA, zA = pos_descent(a.floor_z - 1.0, 5.0, 6.0)
            if sA != 'contact':
                raise Fail(f'높이: 윗면을 못 찾음 ({sA})')
            dsr.move_z_to(zA + 3.0, 30.0)
            sB, zB = pos_descent(a.floor_z - 1.0, 2.0, 4.0, ignore_s=0.6, ignore_mm=0.5)
            if sB != 'contact':
                raise Fail(f'높이: 정밀 하강에서 못 찾음 ({sB})')
            dsr.move_z_to(zB + 3.0, 30.0)
            return zB - a.floor_z

        def match(w):
            return any(abs(w - e) <= a.tol for e in a.expect)

        for t in range(1, a.repeat + 1):
            row = {k: '' for k in FIELDS}
            row['trial'] = t
            if a.ask_angle:
                row['place_angle'] = input(f'[{t}/{a.repeat}] 물체를 놓은 각도(deg, 대략)를 입력하고 Enter: ').strip()
            elif not a.yes:
                input(f'[{t}/{a.repeat}] 물체를 측정존에 (원하는 각도로) 놓고 Enter: ')
            t0 = time.time()
            twist = a.twist if t % 2 else -a.twist                # 비트는 방향을 번갈아 (케이블)
            row['twist_deg'] = twist
            try:
                dsr.move_z_to(max(cur_z(), a.floor_z + a.obj_h_max + 10.0), a.speed)
                H = measure_height() if a.obj_h is None else a.obj_h
                row['H_mm'] = round(H, 2)
                W1 = grasp(H, 'W1')                               # [2] 그대로 잡기
                row['W1_mm'] = round(W1, 2)
                rise_open(H)                                      # [3] 열고 올라가 비틀고 다시 잡기
                turn_j6(twist)
                W2 = grasp(H, 'W2')
                row['W2_mm'] = round(W2, 2)
                row['W1_minus_W2'] = round(W1 - W2, 2)
                log.info(f'[{t}] W1 {W1:.2f} -> 비틀기 {twist:+.0f} deg -> W2 {W2:.2f} (차이 {W1 - W2:+.2f})'
                         + ('  <- 대각이었다가 정렬된 것으로 보임' if W1 - W2 > 2.0 else ''))
                if a.check_d:                                     # [4] 직교 방향
                    rise_open(H)
                    turn_j6(90.0 if j6() < j6_home + 45.0 else -90.0)
                    D = grasp(H, 'D')
                    row['D_mm'] = round(D, 2)
                    log.info(f'[{t}] 직교 방향 D {D:.2f}')
                if a.expect:
                    row['aligned'] = int(match(W2) and (not a.check_d or match(row['D_mm'])))
                row['result'] = 'ok'
            except Fail as e:
                row['result'], row['reason'] = 'failed', str(e)
                log.warn(f'[{t}] 실패: {e}')
            finally:
                try:                                              # [5] 열고 올라가 처음 각도로 (케이블)
                    rise_open(row['H_mm'] or a.obj_h_max)
                    turn_j6(j6_home - j6())
                except Exception as e2:
                    log.error(f'정리 실패: {e2}')
            row['t_s'] = round(time.time() - t0, 1)
            rows.append(row)
    except KeyboardInterrupt:
        log.info('사용자 중단')
    except Exception as e:
        log.error(f'Error: {e}')
    finally:
        if g is not None:
            g.close()
        if rows:
            path = os.path.expanduser(time.strftime('~/regrip_align_%Y%m%d_%H%M%S.csv'))
            with open(path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writeheader()
                w.writerows(rows)
            print(f'\n===== 비틀어 다시 잡기 결과 ({len(rows)}회) =====')
            print(' 시행 | 놓은 각 | 비틀기 |   W1   |   W2   | W1-W2 |   D    | 정렬')
            for r in rows:
                if r['result'] != 'ok':
                    print(f" {r['trial']:4} | 실패: {r['reason']}")
                    continue
                d = f"{r['D_mm']:6.2f}" if r['D_mm'] != '' else '     -'
                ok = {1: 'O', 0: 'X'}.get(r['aligned'], '-')
                print(f" {r['trial']:4} | {r['place_angle'] or '-':>6} | {r['twist_deg']:+5.0f} | {r['W1_mm']:6.2f} | "
                      f"{r['W2_mm']:6.2f} | {r['W1_minus_W2']:+5.2f} | {d} | {ok}")
            print(f'CSV: {path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
