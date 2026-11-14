#!/usr/bin/env python3
"""
robot.py — 기본 TCP·툴 확인·적용 (두산 서비스 직접 호출)
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

왜 필요한가
  TCP·툴은 로봇 컨트롤러에 하나만 걸리는 전역 설정이다.
  탐지·판별·적재가 같은 로봇을 번갈아 쓰므로, 앞 파트가 바꾼 TCP가 그대로 남거나
  (특히 가상 모드 에뮬레이터에서는) TCP가 아예 등록돼 있지 않아
  플랜지 기준으로 움직여 바닥에 박는 일이 생긴다.

언제 실행되나
  scan / judge / place interface가 Goal을 받으면 run() 직전에 ensure_defaults()를 호출한다.
  → 관제와 함께 돌리든, 팀원이 혼자 send_goal로 테스트하든 똑같이 적용된다.
  → 팀원 코드에는 아무것도 추가하지 않아도 된다.

동작 (TCP, 툴 각각)
  1) 현재 이름 조회 → 기본값(common.TCP / common.TOOL)과 같으면 끝
  2) 다르면 적용(set_current_*) → 다시 조회해서 확인
  3) 그래도 다르면 (미등록으로 판단) 수동 모드 → 등록(config_create_*) → 적용 → 자동 모드 → 확인
  4) 최종 확인 실패 → (False, 사유) → interface가 ROBOT_SETUP_FAILED로 Goal을 끝낸다

파라미터 (파트 노드에 선언됨)
  robot_setup = 'auto' (기본) : 두산 서비스가 있으면 확인·적용, 없으면 경고 후 건너뜀 (fake 테스트 가능)
                'on'          : 두산 서비스가 없거나 확인 실패면 Goal 실패 (실제 로봇·시연 권장)
                'off'         : 아무것도 안 함
  예: ros2 run box_scan scan --ros-args -p robot_setup:=on

참고: 서비스 정의 확인
  ros2 interface show dsr_msgs2/srv/GetCurrentTcp
================================================================
"""

import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import SingleThreadedExecutor

from box_comm.common import ROBOT, TCP, TOOL

ROBOT_MODE_MANUAL = 0
ROBOT_MODE_AUTONOMOUS = 1

SERVICE_TIMEOUT = 3.0        # 서비스 응답 대기 (초)
PROBE_TIMEOUT = 0.5          # 두산 서비스 존재 확인 대기 (초)
RECHECK_SEC = 30.0           # auto 모드에서 서비스가 없을 때 다시 확인하는 간격 (초)

# 키 → (서비스 이름, dsr_msgs2.srv 타입 이름)
_SERVICES = {
    'set_mode':    ('system/set_robot_mode', 'SetRobotMode'),
    'tcp_create':  ('tcp/config_create_tcp', 'ConfigCreateTcp'),
    'tcp_set':     ('tcp/set_current_tcp', 'SetCurrentTcp'),
    'tcp_get':     ('tcp/get_current_tcp', 'GetCurrentTcp'),
    'tool_create': ('tool/config_create_tool', 'ConfigCreateTool'),
    'tool_set':    ('tool/set_current_tool', 'SetCurrentTool'),
    'tool_get':    ('tool/get_current_tool', 'GetCurrentTool'),
}


class RobotSetupError(Exception):
    pass


class RobotSetup:
    def __init__(self, parent_node, part: str):
        self.part = part
        self.log = parent_node.get_logger()
        # -p robot_setup:=on / off 는 YAML 규칙상 True / False(bool)로 들어오므로
        # 타입을 고정하지 않고 받아서 문자열로 바꾼다.
        parent_node.declare_parameter(
            'robot_setup', 'auto', ParameterDescriptor(dynamic_typing=True))
        value = parent_node.get_parameter('robot_setup').value
        if value is True:
            self.mode = 'on'
        elif value is False:
            self.mode = 'off'
        else:
            self.mode = str(value).lower()
        if self.mode not in ('auto', 'on', 'off'):
            self.log.warn(f'[{part}] robot_setup="{self.mode}" 알 수 없음 → auto로 동작')
            self.mode = 'auto'

        self._node = None            # 두산 서비스 전용 노드
        self._executor = None        # 위 노드 전용 executor (전역 executor와 분리 — 'already spinning' 방지)
        self._types = None
        self._clients = {}
        self._unavailable_at = None  # auto 모드에서 서비스 없음을 확인한 시각
        self.log.info(f'[{part}] robot_setup={self.mode} '
                      f'(TCP={TCP["name"]}, TOOL={TOOL["name"]})')

    # ------------------------------------------------------------------
    # 공개 함수
    # ------------------------------------------------------------------
    def ensure_defaults(self):
        """기본 TCP·툴 확인·적용. 반환 (ok: bool, message: str)"""
        if self.mode == 'off':
            return True, 'robot_setup=off'

        try:
            if not self._is_available():
                msg = '두산 서비스 없음 (로봇 bringup 확인)'
                if self.mode == 'on':
                    return False, msg
                return True, f'건너뜀: {msg}'

            tcp_msg = self._ensure('tcp')
            tool_msg = self._ensure('tool')
            return True, f'{tcp_msg}, {tool_msg}'
        except RobotSetupError as e:
            return False, str(e)
        except Exception as e:                            # 예상 못 한 오류도 ROBOT_SETUP_FAILED로
            return False, f'{type(e).__name__}: {e}'

    # ------------------------------------------------------------------
    # 내부
    # ------------------------------------------------------------------
    def _is_available(self) -> bool:
        now = time.monotonic()
        if self._unavailable_at is not None and now - self._unavailable_at < RECHECK_SEC:
            return False                                  # 최근에 없음을 확인함 → 빠르게 건너뜀

        try:
            self._make_clients()
        except ImportError:
            if self._unavailable_at is None:
                self.log.warn(f'[{self.part}] dsr_msgs2를 import할 수 없음 → TCP·툴 확인 건너뜀')
            self._unavailable_at = now
            return False

        if self._clients['tcp_get'].wait_for_service(timeout_sec=PROBE_TIMEOUT):
            self._unavailable_at = None
            return True

        if self._unavailable_at is None:
            self.log.warn(f'[{self.part}] 두산 서비스가 없어 TCP·툴 확인을 건너뜀 '
                          f'(robot_setup={self.mode})')
        self._unavailable_at = now
        return False

    def _make_clients(self):
        if self._node is not None:
            return
        import dsr_msgs2.srv as dsr_srv                   # 두산 워크스페이스에서만 있음
        self._types = dsr_srv
        self._node = rclpy.create_node(f'{self.part}_robot_setup')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        ns = ROBOT['service_ns'].rstrip('/')
        for key, (name, type_name) in _SERVICES.items():
            self._clients[key] = self._node.create_client(getattr(dsr_srv, type_name), f'{ns}/{name}')

    def _call(self, key, **fields):
        client = self._clients[key]
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RobotSetupError(f'{_SERVICES[key][0]} 서비스 없음')
        req = getattr(self._types, _SERVICES[key][1]).Request()
        for k, v in fields.items():
            if hasattr(req, k):
                setattr(req, k, v)
        fut = client.call_async(req)
        self._executor.spin_until_future_complete(fut, timeout_sec=SERVICE_TIMEOUT)
        if not fut.done() or fut.result() is None:
            raise RobotSetupError(f'{_SERVICES[key][0]} 응답 없음')
        return fut.result()

    def _current(self, kind) -> str:
        res = self._call(f'{kind}_get')
        return str(getattr(res, 'info', '')).strip()

    def _ensure(self, kind) -> str:
        cfg = TCP if kind == 'tcp' else TOOL
        name = cfg['name']
        label = kind.upper()

        # 1) 이미 기본값
        if self._current(kind) == name:
            return f'{label} OK({name})'

        # 2) 적용만 다시
        self._call(f'{kind}_set', name=name)
        if self._current(kind) == name:
            self.log.info(f'[{self.part}] {label} 재적용: {name}')
            return f'{label} 재적용({name})'

        # 3) 등록부터 (수동 모드 필요)
        self.log.warn(f'[{self.part}] {label} "{name}" 미등록으로 판단 → 등록 후 적용')
        self._call('set_mode', robot_mode=ROBOT_MODE_MANUAL)
        try:
            if kind == 'tcp':
                self._call('tcp_create', name=name, pos=[float(v) for v in cfg['pos']])
            else:
                self._call('tool_create', name=name, weight=float(cfg['weight']),
                           cog=[float(v) for v in cfg['cog']],
                           inertia=[float(v) for v in cfg['inertia']])
            self._call(f'{kind}_set', name=name)          # 이미 등록돼 있어서 create가 실패해도 적용은 시도
        finally:
            self._call('set_mode', robot_mode=ROBOT_MODE_AUTONOMOUS)

        current = self._current(kind)
        if current != name:
            raise RobotSetupError(f'{label} 적용 실패: 기대 "{name}", 현재 "{current}"')
        self.log.info(f'[{self.part}] {label} 등록·적용 완료: {name}')
        return f'{label} 등록·적용({name})'
