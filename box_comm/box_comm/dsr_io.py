#!/usr/bin/env python3
"""
dsr_io.py — 두산 로봇 명령 모음
정지 확인·진행 알림·RG2 Modbus 제어 포함.

추가된 진단 로그:
  [MOVEL SEND]     직선 이동 목표 좌표·속도·가속도
  [MOVEL DONE]     이동 후 실제 좌표
  [COMPLIANCE ON]  순응제어 적용 완료
  [FORCE ON]       목표 힘 적용 완료

기존 이동·그리퍼·정지 로직은 유지한다.
"""

import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from box_comm.common import GRIPPER, ROBOT, SAFETY
from box_comm.robot import connect_dsr


KEEPALIVE_SEC = 1.0
STOP_MODE = int(SAFETY['stop_mode'])

_SERVICES = (
    ('pose', 'GetCurrentPosx', 'aux_control/get_current_posx'),
    ('posj', 'GetCurrentPosj', 'aux_control/get_current_posj'),
    ('force', 'GetToolForce', 'aux_control/get_tool_force'),
    ('motion', 'CheckMotion', 'motion/check_motion'),
    ('stop', 'MoveStop', 'motion/move_stop'),
    ('compliance', 'TaskComplianceCtrl', 'force/task_compliance_ctrl'),
    ('target_force', 'SetDesiredForce', 'force/set_desired_force'),
    ('release_force', 'ReleaseForce', 'force/release_force'),
    (
        'release_compliance',
        'ReleaseComplianceCtrl',
        'force/release_compliance_ctrl',
    ),
)

# 처음 사용할 때만 준비를 기다리는 서비스.
_LAZY_SERVICES = (
    (
        'digital_input',
        'GetCtrlBoxDigitalInput',
        'io/get_ctrl_box_digital_input',
    ),
)


class DsrIO:
    def __init__(self, node_name: str):
        import dsr_msgs2.srv as srv

        self._srv = srv
        self.dr, self.dsr_node = connect_dsr(node_name)

        # 위치·힘·정지 서비스는 별도 노드와 executor를 사용한다.
        self._io_node = rclpy.create_node(f'{node_name}_io')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._io_node)

        ns = ROBOT['service_ns'].rstrip('/')

        self._clients = {
            key: self._io_node.create_client(
                getattr(srv, typ),
                f'{ns}/{name}',
            )
            for key, typ, name in _SERVICES
        }

        self._lazy = {
            key: self._io_node.create_client(
                getattr(srv, typ),
                f'{ns}/{name}',
            )
            for key, typ, name in _LAZY_SERVICES
        }

        self.force_active = False
        self.compliance_active = False
        self.moved = False
        self._ready = False

        self._check = None
        self._keepalive = None
        self._last_keepalive = 0.0

        # 첫 그리퍼 명령에서 연결하고 unbind()에서 해제한다.
        self._gripper = None

    # ------------------------------------------------------------------
    # Goal 단위 연결
    # ------------------------------------------------------------------
    def bind(self, report):
        """관제 정지 확인과 진행 알림을 연결한다."""
        self._check = report.check
        self._keepalive = report.keepalive
        self._last_keepalive = time.monotonic()
        self.moved = False

    def unbind(self):
        """Goal 종료: 정지 확인 연결과 그리퍼 접속을 해제한다."""
        self._check = self._keepalive = None
        self.release_gripper()

    def needs_stop(self) -> bool:
        """실패 종료 시 로봇 정지가 필요한지 반환한다."""
        return self.moved or self.force_active or self.compliance_active

    def _guard(self):
        if self._check is not None:
            self._check()

    def _alive(self):
        """읽기·대기 루프에서 정지 확인과 주기적 진행 알림을 수행한다."""
        if self._keepalive is not None:
            now = time.monotonic()
            if now - self._last_keepalive >= KEEPALIVE_SEC:
                self._keepalive()
                self._last_keepalive = now
                return

        self._guard()

    # ------------------------------------------------------------------
    # 서비스 호출
    # ------------------------------------------------------------------
    def _request(
        self,
        key,
        request,
        timeout=2.0,
        allow_failure=False,
    ):
        client = self._clients.get(key) or self._lazy[key]

        if not client.service_is_ready():
            raise RuntimeError(f'두산 서비스 준비 안 됨: {key}')

        future = client.call_async(request)

        self._executor.spin_until_future_complete(
            future,
            timeout_sec=timeout,
        )

        if not future.done():
            future.cancel()
            raise RuntimeError(f'두산 서비스 시간 초과: {key}')

        result = future.result()

        if result is None or (
            not result.success and not allow_failure
        ):
            raise RuntimeError(f'두산 서비스 실패: {key}')

        return result

    def _dr_call(self, name, *args, **kwargs):
        """DSR_ROBOT2 함수를 호출하고 음수 반환을 실패로 처리한다."""
        result = getattr(self.dr, name)(*args, **kwargs)

        if (
            isinstance(result, (int, float))
            and not isinstance(result, bool)
            and result < 0
        ):
            raise RuntimeError(f'{name} 실패: {result}')

        return result

    # ------------------------------------------------------------------
    # 준비·읽기
    # ------------------------------------------------------------------
    def ready(self, timeout=10.0):
        for client in self._clients.values():
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    f'서비스 준비 안 됨: {client.srv_name}'
                )

        self._dr_call('set_ref_coord', self.dr.DR_BASE)

        # 이전 DO 개폐 신호와 Modbus 제어가 충돌하지 않도록 해제한다.
        self._dr_call('set_digital_output', 1, 0)
        self._dr_call('set_digital_output', 2, 0)

        self._ready = True

    def _prepare(self):
        if not self._ready:
            self.ready()

    def ok(self) -> bool:
        return rclpy.ok()

    def pose(self):
        """현재 TCP 위치 [X, Y, Z, A, B, C], BASE 기준."""
        self._prepare()

        req = self._srv.GetCurrentPosx.Request()
        req.ref = 0

        values = self._request('pose', req).task_pos_info

        if not values or len(values[0].data) < 6:
            raise RuntimeError('TCP 좌표 응답 오류')

        pose = list(values[0].data[:6])

        if not all(math.isfinite(v) for v in pose):
            raise RuntimeError('유효하지 않은 TCP 좌표')

        return pose

    def joints(self):
        """현재 관절각 6개를 degree 단위로 반환한다."""
        self._prepare()

        values = list(
            self._request(
                'posj',
                self._srv.GetCurrentPosj.Request(),
            ).pos
        )

        if len(values) != 6 or not all(
            math.isfinite(v) for v in values
        ):
            raise RuntimeError('유효하지 않은 관절각 응답')

        return [float(v) for v in values]

    def force(self, ref='base'):
        """외력 [Fx, Fy, Fz, Tx, Ty, Tz]를 읽는다."""
        self._alive()
        self._prepare()

        req = self._srv.GetToolForce.Request()
        req.ref = {'base': 0, 'tool': 1}[ref]

        values = self._request('force', req).tool_force

        if len(values) != 6 or not all(
            math.isfinite(v) for v in values
        ):
            raise RuntimeError('유효하지 않은 외력 응답')

        return [float(v) for v in values]

    def wait(self, seconds: float):
        """대기 중에도 관제 정지 확인과 진행 알림을 수행한다."""
        if (
            type(seconds) not in (int, float)
            or not math.isfinite(seconds)
            or seconds < 0
        ):
            raise ValueError(
                'wait seconds는 0 이상 유한한 값이어야 합니다'
            )

        until = time.monotonic() + seconds
        self._alive()

        while self.ok() and time.monotonic() < until:
            time.sleep(
                min(
                    0.05,
                    max(0.0, until - time.monotonic()),
                )
            )
            self._alive()

        if not self.ok():
            raise RuntimeError('대기 중 ROS 종료')

    def digital_input(self, index: int, timeout=0.5) -> int:
        """컨트롤러 DI 1~16의 입력값을 읽는다."""
        if type(index) is not int or not 1 <= index <= 16:
            raise ValueError(
                'digital_input index는 1~16 정수여야 합니다'
            )

        self._alive()
        self._prepare()

        client = self._lazy['digital_input']

        if (
            not client.service_is_ready()
            and not client.wait_for_service(timeout_sec=2.0)
        ):
            raise RuntimeError(
                f'서비스 준비 안 됨: {client.srv_name}'
            )

        req = self._srv.GetCtrlBoxDigitalInput.Request()
        req.index = index

        value = self._request(
            'digital_input',
            req,
            timeout=timeout,
        ).value

        if value not in (0, 1):
            raise RuntimeError(f'DI{index} 응답 오류: {value}')

        return int(value)

    def force_z(self) -> float:
        """BASE 기준 툴 외력 Fz."""
        return self.force('base')[2]

    # ------------------------------------------------------------------
    # 이동
    # ------------------------------------------------------------------
    def movel(self, pose, vel, acc, tol=2.0):
        self._guard()
        self._prepare()

        self.dsr_node.get_logger().info(
            f'[MOVEL SEND] BASE 목표={list(pose)}, '
            f'vel={vel}, acc={acc}'
        )

        self.moved = True

        self._dr_call(
            'movel',
            self.dr.posx(list(pose)),
            vel=vel,
            acc=acc,
            ref=self.dr.DR_BASE,
        )

        actual = self.pose()

        if any(
            abs(actual[i] - pose[i]) > tol
            for i in range(3)
        ):
            raise RuntimeError('이동 목표 미도달')

        self.dsr_node.get_logger().info(
            f'[MOVEL DONE] BASE 실제={list(actual)}, '
            f'목표 Z={pose[2]}'
        )

    def movej(self, joints, vel, acc):
        self._guard()
        self._prepare()
        self.moved = True

        self._dr_call(
            'movej',
            self.dr.posj(list(joints)),
            vel=vel,
            acc=acc,
        )

    def amovel_rel(
        self,
        delta,
        vel=None,
        acc=None,
        time_s=None,
    ):
        """BASE 기준 비동기 상대 직선 이동."""
        self._guard()
        self._prepare()
        self.moved = True

        target = self.dr.posx([float(v) for v in delta])

        if time_s is not None:
            self._dr_call(
                'amovel',
                target,
                time=float(time_s),
                ref=self.dr.DR_BASE,
                mod=self.dr.DR_MV_MOD_REL,
            )
        else:
            self._dr_call(
                'amovel',
                target,
                vel=vel,
                acc=acc,
                ref=self.dr.DR_BASE,
                mod=self.dr.DR_MV_MOD_REL,
            )

    def amovel(self, pose, vel, acc):
        """BASE 기준 비동기 절대 직선 이동."""
        self._guard()
        self._prepare()
        self.moved = True

        self._dr_call(
            'amovel',
            self.dr.posx([float(v) for v in pose]),
            vel=vel,
            acc=acc,
            ref=self.dr.DR_BASE,
            mod=self.dr.DR_MV_MOD_ABS,
        )

    def movejx(self, pose, vel, acc, tol=2.0):
        """현재 해 공간을 유지하며 TCP 목표로 관절 보간 이동."""
        self._guard()
        self._prepare()
        self.moved = True

        _, sol = self.dr.get_current_posx(
            ref=self.dr.DR_BASE
        )

        self._dr_call(
            'movejx',
            self.dr.posx([float(v) for v in pose]),
            vel=vel,
            acc=acc,
            sol=int(sol),
            ref=self.dr.DR_BASE,
        )

        actual = self.pose()

        if any(
            abs(actual[i] - pose[i]) > tol
            for i in range(3)
        ):
            raise RuntimeError('관절 보간 이동 목표 미도달')

    def motion_state(self) -> int:
        """0: 정지, 1: 가감속, 2: 이동 중."""
        self._guard()
        self._prepare()

        state = int(
            self._request(
                'motion',
                self._srv.CheckMotion.Request(),
            ).status
        )

        if state not in (0, 1, 2):
            raise RuntimeError(
                f'알 수 없는 이동 상태: {state}'
            )

        return state

    def mwait(self):
        """비동기 이동 완료까지 기다린다."""
        self._prepare()
        self._dr_call('mwait')

    def move_stop(self, mode=2) -> bool:
        """현재 이동만 정지한다. 힘제어 해제와 정지 확인은 하지 않는다."""
        self._prepare()

        req = self._srv.MoveStop.Request()
        req.stop_mode = int(mode)

        try:
            return bool(
                self._request(
                    'stop',
                    req,
                    timeout=2.0,
                ).success
            )
        except RuntimeError:
            return False

    # ------------------------------------------------------------------
    # 그리퍼 — RG2 Modbus
    # ------------------------------------------------------------------
    @property
    def gripper(self):
        """최초 사용 시 연결하고 Goal 종료 시 접속을 해제한다."""
        if self._gripper is None:
            from box_comm.gripper import make_gripper

            self._gripper = make_gripper()

        return self._gripper

    def release_gripper(self):
        if self._gripper is not None:
            try:
                self._gripper.close()
            finally:
                self._gripper = None

    def grip_open(self, width_mm=None, force_n=None):
        """지정 폭으로 열고 원시 폭을 반환한다."""
        self._guard()
        self.moved = True

        return self.gripper.open(
            force_n or GRIPPER['open_force_n'],
            width_mm or GRIPPER['open_width_mm'],
        )

    def grip_close(self, force_n=None):
        """닫고 GripResult(grabbed, width_mm, raw_mm)를 반환한다."""
        self._guard()
        self.moved = True

        return self.gripper.grip(
            force_n or GRIPPER['grip_force_n']
        )

    def gripper_width(self) -> float:
        """보정된 패드 사이 폭을 mm 단위로 반환한다."""
        return self.gripper.width()

    def grip(self, opened: bool, wait_s=None):
        """기존 호출 호환용 열기·닫기 함수."""
        if opened:
            self.grip_open()
            return None

        return self.grip_close()

    # ------------------------------------------------------------------
    # 순응제어·힘제어
    # ------------------------------------------------------------------
    def compliance_on(self, stx):
        self._guard()
        self._prepare()
        self.moved = True
        self.compliance_active = True

        req = self._srv.TaskComplianceCtrl.Request()
        req.stx = [float(v) for v in stx]
        req.ref, req.time = 0, 0.0

        self._request('compliance', req)

        self.dsr_node.get_logger().info(
            f'[COMPLIANCE ON] BASE stx={list(req.stx)}'
        )

    def force_on(self, fd, direction, time_s=0.5):
        self._guard()
        self._prepare()
        self.moved = True
        self.force_active = True

        req = self._srv.SetDesiredForce.Request()
        req.fd = [float(v) for v in fd]
        req.dir = [int(v) for v in direction]

        req.ref, req.time, req.mod = (
            0,
            float(time_s),
            self.dr.DR_FC_MOD_REL,
        )

        self._request('target_force', req)

        self.dsr_node.get_logger().info(
            f'[FORCE ON] BASE REL fd={list(req.fd)}, '
            f'dir={list(req.dir)}, time={req.time}'
        )

    # ------------------------------------------------------------------
    # 힘·순응제어 해제 및 정지 확인
    # ------------------------------------------------------------------
    def stop(self, emergency=False):
        """
        힘·순응제어 해제 후 정지를 확인한다.
        emergency=True이면 move_stop도 전송한다.
        정지 처리 중에는 관제 취소 확인으로 중단하지 않는다.
        """
        self._prepare()
        errors = []

        if self.force_active:
            try:
                req = self._srv.ReleaseForce.Request()
                req.time = 0.0

                self._request('release_force', req)
                self.force_active = False

            except Exception as exc:
                errors.append(exc)

        if self.compliance_active:
            try:
                self._request(
                    'release_compliance',
                    self._srv.ReleaseComplianceCtrl.Request(),
                )
                self.compliance_active = False

            except Exception as exc:
                errors.append(exc)

        if emergency or errors:
            req = self._srv.MoveStop.Request()
            req.stop_mode = STOP_MODE

            self._request(
                'stop',
                req,
                timeout=5.0,
            )

        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            status = self._request(
                'motion',
                self._srv.CheckMotion.Request(),
            ).status

            if status == 0:
                if errors:
                    raise RuntimeError(
                        f'힘·순응제어 해제 실패: {errors}'
                    )
                return

            time.sleep(0.05)

        raise RuntimeError('정지 확인 실패')
