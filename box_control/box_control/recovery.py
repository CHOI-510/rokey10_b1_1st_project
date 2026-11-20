#!/usr/bin/env python3
"""
recovery.py — 관제 복구 동작 모음 (관제 전용)
================================================================
관리: 관제(정쓰). 파트 Goal이 끝난 뒤, 관제가 직접 로봇을 움직여야 하는 일을 여기에 모은다.
  (파트 Goal 안에서는 파트가 로봇을 쓰고, 그 사이 빈 시간에만 관제가 쓴다)

지금 있는 것
  prepare_start(poses): 기존 시작 그리퍼 설정을 유지한다.
  clear_measure_zone(poses): 판별 NOT_FOUND 또는 판별·적재 GRASP_FAILED 뒤 측정존을 쓸고 반납한다.
    1. 현재 XY에서 TCP Z160으로 수직 이동 → 그리퍼 열기
    2. 도구 픽업 상부 Z160 → 픽업 좌표 → 도구 파지
    3. 픽업 상부 Z160 → 밀기 시작 상부 Z160 → 밀기 시작 좌표
    4. 밀기 끝 좌표까지 직선 밀기
    5. 밀기 끝 상부 Z160 → 픽업 상부 Z160 → 픽업 좌표
    6. 그리퍼 열기 → 픽업 상부 Z160에서 종료
  실제 로봇 경로는 TOOL_SWEEP 고정 좌표를 사용한다. 동작 완료 후 잔류 물체는 다음 탐지에서 확인한다.
  도구 파지 실패나 모션 실패 시 RECOVERY_FAILED. 실패 중 도구를 자동 개방/반납하지 않는다.

로봇 연결
  처음 쓸 때 DsrIO('control_dsr')를 만든다 (탐지·판별·적재와 같은 방식, 그리퍼는 Modbus).
  명령마다 관제 정지 요청(비상정지·로봇 이상)을 확인한다 → 정지 중이면 바로 멈추고 정지 코드 반환.
  TCP·툴은 직전 파트 Goal이 맞춰 둔 기본값 그대로 쓴다.

파라미터 (control 노드)
  clear_zone : auto(기본) / robot / fake_ok / fake_fail
    auto → robot_monitor=dsr면 robot(실제 동작), 아니면 fake_ok (로봇 없는 fake 시험)
    fake_ok  → 움직이지 않고 성공으로 침 (fake_scan이 박스를 다시 세도록 BOX_FAKE_RETURN_FILE에 기록)
    fake_fail → 움직이지 않고 실패 (작업 종료 시험)
반환: 성공 None / 실패 'RECOVERY_FAILED' / 정지 요청이면 그 정지 코드
================================================================
"""

import math
import os
import time

from box_comm.common import RECOVERY, PartError

CLEAR_MODES = ('auto', 'robot', 'fake_ok', 'fake_fail')

# 그리퍼 TCP 기준 [X,Y,Z,A,B,C], mm / degree. 검증된 GripperDA_v1 복구 경로.
# 구역 간 이동은 TCP Z160, 작업 구간은 Z101~103이다.
TOOL_SWEEP = {
    'travel_z': 160.0,
    'pickup_pose': [-178.0, -239.0, 101.0, 70.0, 180.0, 70.0],
    'push_start_pose': [300.0, -280.0, 101.0, 26.6, -180.0, -62.0],
    'push_end_pose': [300.0, -44.0, 103.0, 26.6, -180.0, -62.0],
    'travel_speed_mm_s': 60.0,
    'travel_acc_mm_s2': 100.0,
    'work_speed_mm_s': 30.0,
    'work_acc_mm_s2': 60.0,
    'grip_force_n': 10.0,
    'grip_settle_s': 0.5,
    'release_settle_s': 0.5,
}


class _StopGuard:
    """DsrIO.bind()에 넘기는 report 대신: 관제 정지 요청이면 명령 전에 PartError('CANCELED')"""

    def __init__(self, io):
        self._io = io

    def check(self):
        if self._io.stop_reason():
            raise PartError('CANCELED', '관제 정지 요청')

    keepalive = check            # 관제는 자기 자신에게 진행 알림을 보낼 필요 없음


class Recovery:
    def __init__(self, io, log):
        self.io = io                       # ControlInterface (정지 상태 확인, 파라미터)
        self.log = log
        mode = str(io.declare_parameter('clear_zone', 'auto').value).lower()
        if mode not in CLEAR_MODES:
            log.warn(f'[recovery] clear_zone="{mode}" 알 수 없음 → auto')
            mode = 'auto'
        if mode == 'auto':
            mode = 'robot' if getattr(io, '_monitor', 'dsr') == 'dsr' else 'fake_ok'
        self.mode = mode
        self._robot = None
        self.last_error = ''               # 마지막 실패 이유 (로그용)
        self.skipped = False               # 기존 관제 호환용
        self.tool_phase = 'idle'
        self.tool_held = False

    def prepare_start(self, poses):
        """시작 카운트다운 종료 후 그리퍼를 열고 HOME으로 이동한 뒤 닫는다."""
        self.last_error = ''
        if getattr(self.io, '_monitor', 'dsr') != 'dsr':
            self.log.info('[start_setting] fake 로봇 모드 — 그리퍼 초기화 동작 생략')
            return None
        home = (poses or {}).get('home_j')
        if (not isinstance(home, (list, tuple)) or len(home) != 6
                or any(not math.isfinite(float(v)) for v in home)):
            self.last_error = f'DB HOME 관절 좌표가 유효하지 않음 ({home})'
            return 'RECOVERY_FAILED'
        state, name = self.io.robot_state()
        if state != 1:  # DSR STANDBY
            self.last_error = f'그리퍼 초기화 전 로봇이 STANDBY가 아님 ({name})'
            return 'RECOVERY_FAILED'

        def reset_to_home(robot):
            robot.grip_open()                       # 공용 최대 열기 폭
            robot.movej(home, vel=100.0, acc=100.0)
            result = robot.grip_close()
            self.log.info(f'[start_setting] 최대 열기 → HOME 이동 → 닫기 완료 (폭 {result.width_mm:.1f} mm)')

        return self._run('시작 그리퍼 설정', reset_to_home)

    # ------------------------------------------------------------------
    # 로봇 연결 (처음 쓸 때 한 번)
    # ------------------------------------------------------------------
    def _connect(self):
        if self._robot is None:
            from box_comm.dsr_io import DsrIO          # 두산 패키지가 있어야 함
            self._robot = DsrIO('control_dsr')
        return self._robot

    def _run(self, name, work):
        """로봇 동작 실행 틀: 연결 → 정지 확인 연결 → work(robot) → 끝나면 그리퍼 접속 해제.
        반환: None(성공) / 정지 코드 / 'RECOVERY_FAILED'"""
        try:
            robot = self._connect()
        except Exception as e:
            self.last_error = f'관제 로봇 연결 실패: {type(e).__name__}: {e}'
            return 'RECOVERY_FAILED'
        robot.bind(_StopGuard(self.io))
        try:
            work(robot)
            return None
        except PartError as e:
            if e.code == 'CANCELED':
                stop = self.io.stop_reason()
                self.last_error = f'{name} 중 정지 요청'
                self._stop_after_error(robot)
                return stop[0] if stop else 'RECOVERY_FAILED'
            self.last_error = str(e)
            self._stop_after_error(robot)
            return 'RECOVERY_FAILED'
        except Exception as e:
            self.last_error = f'{name} 중 오류: {type(e).__name__}: {e}'
            self._stop_after_error(robot)
            return 'RECOVERY_FAILED'
        finally:
            robot.unbind()                             # 그리퍼(Modbus) 접속 해제 → 다음 파트가 씀

    def _stop_after_error(self, robot):
        if robot.needs_stop():
            try:
                robot.stop(emergency=True)
            except Exception as stop_exc:
                self.last_error += f'; 정지 확인 실패: {stop_exc}'
        if self.tool_held:
            self.log.warn(f'[recovery] 도구 파지 상태에서 중단 (단계={self.tool_phase}). '
                          '자동 개방/반납하지 않습니다. 재시작 전 도구 상태를 확인하세요')

    # ------------------------------------------------------------------
    # 측정존 비우기
    # ------------------------------------------------------------------
    def clear_measure_zone(self, poses):
        self.last_error = ''
        self.skipped = False
        if self.mode == 'fake_ok':
            time.sleep(0.5)
            try:                                       # fake_scan이 밀려온 박스를 다시 세게
                with open(os.environ.get('BOX_FAKE_RETURN_FILE', '/tmp/box_fake_return'), 'a') as f:
                    f.write('1\n')
            except OSError:
                pass
            self.log.info('[recovery] (fake) 측정존 비움')
            return None
        if self.mode == 'fake_fail':
            time.sleep(0.5)
            self.last_error = '(fake) 측정존 비우기 실패'
            return 'RECOVERY_FAILED'
        return self._run('도구로 측정존 비우기', self._sweep_with_tool)

    def _set_tool_phase(self, phase):
        self.tool_phase = phase
        self.log.info(f'[recovery] 도구 복구 단계={phase}')

    @staticmethod
    def _tool_above(pose):
        return [pose[0], pose[1], TOOL_SWEEP['travel_z'], *pose[3:6]]

    @staticmethod
    def _tool_move(robot, pose, work=False):
        cfg = TOOL_SWEEP
        robot.movel(list(pose),
                    vel=cfg['work_speed_mm_s'] if work else cfg['travel_speed_mm_s'],
                    acc=cfg['work_acc_mm_s2'] if work else cfg['travel_acc_mm_s2'])

    def _tool_to_travel_z(self, robot):
        pose = list(robot.pose())
        if len(pose) != 6 or not all(math.isfinite(float(v)) for v in pose):
            raise RuntimeError('도구 복구 시작 TCP 좌표가 유효하지 않습니다')
        if abs(pose[2] - TOOL_SWEEP['travel_z']) > 0.01:
            pose[2] = TOOL_SWEEP['travel_z']
            self._tool_move(robot, pose)

    def _sweep_with_tool(self, robot):
        cfg = TOOL_SWEEP
        pickup = list(cfg['pickup_pose'])
        push_start = list(cfg['push_start_pose'])
        push_end = list(cfg['push_end_pose'])
        pickup_above = self._tool_above(pickup)
        push_start_above = self._tool_above(push_start)
        push_end_above = self._tool_above(push_end)

        if self.tool_held:
            raise RuntimeError('이전 복구의 도구 파지 상태가 남아 있습니다. 도구 반납 상태를 확인하세요')
        self._set_tool_phase('approaching_tool')
        self._tool_to_travel_z(robot)
        robot.grip_open()
        self._tool_move(robot, pickup_above)
        self._tool_move(robot, pickup, work=True)

        self._set_tool_phase('gripping_tool')
        result = robot.grip_close(cfg['grip_force_n'])
        if not result.grabbed:
            raise RuntimeError('쓸기 도구 파지 실패: 그리퍼 잡힘 신호가 없습니다')
        self.tool_held = True
        robot.wait(cfg['grip_settle_s'])
        self.log.info(f'[recovery] 도구 파지 확인: 폭={result.width_mm:.2f}mm, '
                      f'설정 힘={cfg["grip_force_n"]:.1f}N')

        self._set_tool_phase('transporting_tool')
        self._tool_move(robot, pickup_above)
        self._tool_move(robot, push_start_above)
        self._tool_move(robot, push_start, work=True)

        self._set_tool_phase('sweeping')
        self._tool_move(robot, push_end, work=True)

        self._set_tool_phase('returning_tool')
        self._tool_move(robot, push_end_above)
        self._tool_move(robot, pickup_above)
        self._tool_move(robot, pickup, work=True)

        self._set_tool_phase('releasing_tool')
        robot.grip_open()
        self.tool_held = False
        robot.wait(cfg['release_settle_s'])
        self._tool_move(robot, pickup_above)
        self._set_tool_phase('complete')
        self.log.info('[recovery] 도구 밀기·반납 완료. 물체 잔류 여부는 다음 탐지 단계에서 확인합니다')

    def measure_zone_empty(self, robot, zone) -> bool:
        """측정존 가운데를 그리퍼로 잡아 봐서 비었는지 확인. 끝나면 그리퍼 열고 안전 높이."""
        cfg = RECOVERY
        self._rise(robot)
        abc = list(cfg['probe_orientation'] or robot.pose()[3:6])
        robot.grip_open()
        robot.movel([zone[0], zone[1], cfg['safe_z'], *abc], vel=cfg['speed'], acc=cfg['acc'])
        robot.movel([zone[0], zone[1], zone[2] + cfg['probe_above_floor_mm'], *abc],   # 측정존 z = 바닥 (11/18)
                    vel=cfg['speed'] / 2, acc=cfg['acc'])
        result = robot.grip_close()
        robot.grip_open()
        self._rise(robot)
        return not result.grabbed

    @staticmethod
    def _rise(robot):
        cfg = RECOVERY
        pose = robot.pose()
        if pose[2] < cfg['safe_z']:
            pose[2] = cfg['safe_z']
            robot.movel(pose, vel=cfg['speed'], acc=cfg['acc'])
