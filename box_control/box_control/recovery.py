#!/usr/bin/env python3
"""
recovery.py — 관제 복구 동작 모음 (관제 전용)
================================================================
관리: 관제(정쓰). 파트 Goal이 끝난 뒤, 관제가 직접 로봇을 움직여야 하는 일을 여기에 모은다.
  (파트 Goal 안에서는 파트가 로봇을 쓰고, 그 사이 빈 시간에만 관제가 쓴다)

지금 있는 것
  clear_measure_zone(poses)  판별·적재 집기 실패(GRASP_FAILED) 뒤 측정존 비우기 (11/17)
      1. 그리퍼 열고 안전 높이(safe_z)로 수직 상승
      2. push_path 첫 점 위로 이동 → 그리퍼 닫고 push_path를 차례로 movel (박스를 스캔 공간으로 밀어냄)
      3. 안전 높이로 → 측정존이 비었는지 확인
         (측정존 위에서 그리퍼를 열고 파지 높이까지 내려가 닫아 봄 → 잡히면 아직 박스가 있음, 가운데 근처만 확인)
      설정: common.RECOVERY. push_path가 None(미정)이면 움직이지 않고 실패 → 사람이 비움
나중에 붙일 것 (예): 안전 높이 복귀, 그리퍼 열기, 측정존 상태 확인만 하기 ...

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

import os
import time

from box_comm.common import RECOVERY, PartError

CLEAR_MODES = ('auto', 'robot', 'fake_ok', 'fake_fail')


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
                return stop[0] if stop else 'RECOVERY_FAILED'
            self.last_error = str(e)
            return 'RECOVERY_FAILED'
        except Exception as e:
            self.last_error = f'{name} 중 오류: {type(e).__name__}: {e}'
            if robot.needs_stop():
                try:
                    robot.stop(emergency=True)
                except Exception as stop_exc:
                    self.last_error += f'; 정지 확인 실패: {stop_exc}'
            return 'RECOVERY_FAILED'
        finally:
            robot.unbind()                             # 그리퍼(Modbus) 접속 해제 → 다음 파트가 씀

    # ------------------------------------------------------------------
    # 측정존 비우기
    # ------------------------------------------------------------------
    def clear_measure_zone(self, poses):
        self.last_error = ''
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
        if not RECOVERY['push_path']:
            self.last_error = ('측정존 밀어내기 경로 미정 (common.RECOVERY push_path) '
                               '→ 측정존을 사람이 비운 뒤 다시 시작')
            return 'RECOVERY_FAILED'
        zone = (poses or {}).get('measure_zone')
        return self._run('측정존 비우기', lambda robot: self._push_and_check(robot, zone))

    def _push_and_check(self, robot, zone):
        cfg = RECOVERY
        path = cfg['push_path']
        robot.grip_open()
        self._rise(robot)
        first = list(path[0])
        robot.movel([first[0], first[1], cfg['safe_z'], *first[3:6]], vel=cfg['speed'], acc=cfg['acc'])
        robot.grip_close()                                # 닫은 손가락으로 민다 (빈손이 정상)
        for p in path:
            robot.movel(list(p), vel=cfg['speed'], acc=cfg['acc'])
        self._rise(robot)
        self.log.info('[recovery] 밀어내기 완료 → 측정존 확인')
        if zone is not None and not self.measure_zone_empty(robot, zone):
            raise RuntimeError('밀어낸 뒤에도 측정존에 박스가 남아 있음')

    def measure_zone_empty(self, robot, zone) -> bool:
        """측정존 가운데를 그리퍼로 잡아 봐서 비었는지 확인. 끝나면 그리퍼 열고 안전 높이."""
        cfg = RECOVERY
        self._rise(robot)
        abc = list(cfg['probe_orientation'] or robot.pose()[3:6])
        robot.grip_open()
        robot.movel([zone[0], zone[1], cfg['safe_z'], *abc], vel=cfg['speed'], acc=cfg['acc'])
        robot.movel([zone[0], zone[1], zone[2], *abc], vel=cfg['speed'] / 2, acc=cfg['acc'])
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
