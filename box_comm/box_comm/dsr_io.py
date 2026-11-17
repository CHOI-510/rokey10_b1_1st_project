#!/usr/bin/env python3
"""
dsr_io.py — 파트가 쓰는 두산 로봇 명령 모음 (정지 확인 내장)
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 필요한 명령이 없으면 관제에게 요청.

왜 필요한가
  관제가 정지를 요청하면(비상정지, 로봇 이상, 진행 없음) 관제는 move_stop을 보내고 파트 Goal을 cancel한다.
  그런데 move_stop 뒤 로봇은 STANDBY로 돌아가서, 파트가 다음 movel을 그대로 보내면 다시 움직인다.
  그래서 파트가 '명령을 보내기 직전'에 정지 요청을 확인해야 한다.
  → 이 확인을 파트 코드에 흩어 넣지 않고, 여기 명령 함수 안에 넣어 둔다.
    파트 알고리즘은 이 객체(robot)로만 움직이면 정지 확인·진행 알림이 자동으로 된다.

누가 쓰나
  파트 알고리즘은 robot을 직접 만들지 않는다. box_comm(예: place_interface.start_algorithm)이
  DsrIO를 만들어 알고리즘에 넣어 주고, Goal마다 bind(report)로 정지 확인을 연결한다.
  (box_comm/algorithm.py AlgorithmWork — 탐지·판별·적재 공용)
  두산 서비스 준비 확인은 첫 명령 때 한 번 한다 → 로봇을 안 쓰는 FAKE 코드는 bringup 없이도 돈다.

  정지 확인  : movel / movej / amovel_rel / 그리퍼 / 힘·순응제어 시작 전 → 관제가 취소했으면 PartError('CANCELED')
  진행 알림  : force_z() / force()를 부를 때 1초에 한 번 report.keepalive() (힘 제어·접촉 하강 루프가 매 반복 부름)
  비상 정지  : stop(emergency=True) — 힘·순응제어 해제 + move_stop + 멈춘 것 확인
  그리퍼     : RG2 Modbus (box_comm/gripper.py). 그리퍼 명령을 처음 쓸 때 접속, Goal이 끝나면(unbind) 해제
               (Compute Box 동시 접속이 1개라서 탐지·판별·적재가 번갈아 쓴다). BOX_GRIPPER=fake면 가짜 그리퍼

명령 목록 (알고리즘이 쓰는 것 — 여기 없는 두산 기능이 필요하면 관제에게 요청)
  ready()                         서비스 준비 확인 + 기준 좌표계 BASE + 그리퍼 DO 1·2 끔 (첫 명령 때 자동)
  pose() -> [x, y, z, a, b, c]    현재 TCP 위치 (BASE)
  joints() -> [j1 .. j6]          현재 관절각 [deg]
  force_z() -> float              툴 외력 Fz [N], BASE 기준 (+ 진행 알림)
  force(ref='base') -> [6]        툴 외력 [Fx, Fy, Fz, Tx, Ty, Tz] (+ 진행 알림)
                                  ref='base'|'tool' — 두산 규칙: 힘은 BASE 기준, 모멘트는 TOOL 기준
  movel(pose, vel, acc, tol=2.0)  직선 이동 후 도달 확인 (오차 tol mm 넘으면 RuntimeError)
  movej(joints, vel, acc)         관절 이동
  amovel_rel(delta, vel, acc)     비동기 상대 직선 이동 (BASE). time_s를 주면 시간 기준. 끝까지 기다리지 않음
  mwait()                         비동기 이동이 끝날 때까지 대기
  move_stop(mode=2) -> bool       지금 이동만 멈춤 (1 빠른 정지, 2 부드러운 정지). 접촉 감지 후 하강 끊기용
  grip(opened, wait_s=None)       그리퍼 기본 열기/닫기 (예전 이름 그대로, 닫기는 GripResult 반환)
  grip_open(width_mm, force_n)    지정 폭으로 열기 → 원시 폭 반환
  grip_close(force_n) -> GripResult(grabbed, width_mm, raw_mm)   닫기 + 잡힘 여부·폭 (못 잡아도 예외 없음)
  gripper_width() -> float        지금 폭 (패드 사이, 보정 반영) [mm]
  gripper                         RG2 객체 그대로 (판별 폭 측정처럼 세밀한 제어용)
  compliance_on(stx)              순응제어 시작 (task_compliance_ctrl)
  force_on(fd, direction, time)   목표 힘 (set_desired_force, 상대 모드)
  stop(emergency=False)           힘·순응제어 해제 (+ emergency면 move_stop) → 멈춤 확인
  ok() -> bool                    ROS가 살아 있는지
================================================================
"""

import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from box_comm.common import GRIPPER, ROBOT, SAFETY
from box_comm.robot import connect_dsr

KEEPALIVE_SEC = 1.0     # force_z()에서 관제에 '살아 있음'을 보내는 최소 간격
STOP_MODE = int(SAFETY['stop_mode'])   # move_stop 모드 (관제 정지와 같은 값)

_SERVICES = (           # 키, dsr_msgs2.srv 타입 이름, 서비스 이름 (ROBOT['service_ns'] 아래)
    ('pose', 'GetCurrentPosx', 'aux_control/get_current_posx'),
    ('posj', 'GetCurrentPosj', 'aux_control/get_current_posj'),
    ('force', 'GetToolForce', 'aux_control/get_tool_force'),
    ('motion', 'CheckMotion', 'motion/check_motion'),
    ('stop', 'MoveStop', 'motion/move_stop'),
    ('compliance', 'TaskComplianceCtrl', 'force/task_compliance_ctrl'),
    ('target_force', 'SetDesiredForce', 'force/set_desired_force'),
    ('release_force', 'ReleaseForce', 'force/release_force'),
    ('release_compliance', 'ReleaseComplianceCtrl', 'force/release_compliance_ctrl'),
)


class DsrIO:
    def __init__(self, node_name: str):
        import dsr_msgs2.srv as srv                     # 두산 워크스페이스에서만 있음
        self._srv = srv
        self.dr, self.dsr_node = connect_dsr(node_name)        # movel·movej 등 DSR_ROBOT2 함수용
        # 위치·힘·정지 서비스는 DSR_ROBOT2 노드에 붙이지 않고 전용 노드 + 전용 executor로 부른다.
        # (DSR_ROBOT2 노드에 같은 서비스 클라이언트를 한 벌 더 만들면, 수동 모드 전환 때 컨트롤러가
        #  응답하지 않는 현상 확인 — 11/17. robot.py와 같은 방식)
        self._io_node = rclpy.create_node(f'{node_name}_io')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._io_node)
        ns = ROBOT['service_ns'].rstrip('/')
        self._clients = {key: self._io_node.create_client(getattr(srv, typ), f'{ns}/{name}')
                         for key, typ, name in _SERVICES}
        self.force_active = False
        self.compliance_active = False
        self.moved = False              # 이번 Goal에서 로봇에 동작 명령을 보냈는지 (실패 시 정지 판단용)
        self._ready = False             # 서비스 준비·기준 좌표계 설정을 했는지 (첫 명령 때 한 번)
        self._check = None
        self._keepalive = None
        self._last_keepalive = 0.0
        self._gripper = None            # RG2 (Modbus) — 그리퍼 명령을 처음 쓸 때 접속, unbind에서 해제

    # ------------------------------------------------------------------
    # Goal 단위 연결 (box_comm이 부름)
    # ------------------------------------------------------------------
    def bind(self, report):
        """Goal 시작: 관제 정지 확인·진행 알림 연결"""
        self._check = report.check
        self._keepalive = report.keepalive
        self._last_keepalive = time.monotonic()
        self.moved = False

    def unbind(self):
        """Goal 끝: 정지 확인 해제 + 그리퍼 접속 해제 (다음 파트가 접속할 수 있게)"""
        self._check = self._keepalive = None
        self.release_gripper()

    def needs_stop(self) -> bool:
        """실패로 끝날 때 로봇을 멈춰야 하는지"""
        return self.moved or self.force_active or self.compliance_active

    def _guard(self):
        if self._check is not None:
            self._check()

    def _alive(self):
        """오래 도는 읽기 루프용: 1초에 한 번 진행 알림(정지 확인 포함), 그 사이는 정지 확인만"""
        if self._keepalive is not None:
            now = time.monotonic()
            if now - self._last_keepalive >= KEEPALIVE_SEC:
                self._keepalive()                       # check 포함
                self._last_keepalive = now
                return
        self._guard()

    # ------------------------------------------------------------------
    # 서비스 호출
    # ------------------------------------------------------------------
    def _request(self, key, request, timeout=2.0, allow_failure=False):
        client = self._clients[key]
        if not client.service_is_ready():
            raise RuntimeError(f'두산 서비스 준비 안 됨: {key}')
        future = client.call_async(request)
        self._executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'두산 서비스 시간 초과: {key}')
        result = future.result()
        if result is None or (not result.success and not allow_failure):
            raise RuntimeError(f'두산 서비스 실패: {key}')
        return result

    def _dr_call(self, name, *args, **kwargs):
        """DSR_ROBOT2 함수 호출. 음수 반환은 실패로 본다."""
        result = getattr(self.dr, name)(*args, **kwargs)
        if isinstance(result, (int, float)) and not isinstance(result, bool) and result < 0:
            raise RuntimeError(f'{name} 실패: {result}')
        return result

    # ------------------------------------------------------------------
    # 준비·읽기
    # ------------------------------------------------------------------
    def ready(self, timeout=10.0):
        for client in self._clients.values():
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'서비스 준비 안 됨: {client.srv_name}')
        self._dr_call('set_ref_coord', self.dr.DR_BASE)
        # 그리퍼는 Modbus로 다룬다 (11/17). 예전 DO 잡기/놓기 신호가 켜진 채 남아 Modbus 명령과 부딪히지 않게 끔
        self._dr_call('set_digital_output', 1, 0)
        self._dr_call('set_digital_output', 2, 0)
        self._ready = True

    def _prepare(self):
        if not self._ready:
            self.ready()

    def ok(self) -> bool:
        return rclpy.ok()

    def pose(self):
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
        """현재 관절각 [deg] 6개"""
        self._prepare()
        values = list(self._request('posj', self._srv.GetCurrentPosj.Request()).pos)
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('유효하지 않은 관절각 응답')
        return [float(v) for v in values]

    def force(self, ref='base'):
        """툴 외력 [Fx, Fy, Fz, Tx, Ty, Tz]. 루프가 매 반복 부르므로 여기서 정지 확인 + 진행 알림.
        ref: 'base'(0) / 'tool'(1). 두산 규칙상 힘은 BASE 기준, 모멘트는 ref 기준."""
        self._alive()
        self._prepare()
        req = self._srv.GetToolForce.Request()
        req.ref = {'base': 0, 'tool': 1}[ref]
        values = self._request('force', req).tool_force
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise RuntimeError('유효하지 않은 외력 응답')
        return [float(v) for v in values]

    def force_z(self) -> float:
        """툴 외력 Fz (BASE). 힘 제어 하강 루프용 (정지 확인 + 진행 알림 포함)"""
        return self.force('base')[2]

    # ------------------------------------------------------------------
    # 동작 (모두 명령 전에 정지 확인)
    # ------------------------------------------------------------------
    def movel(self, pose, vel, acc, tol=2.0):
        self._guard()
        self._prepare()
        self.moved = True
        self._dr_call('movel', self.dr.posx(list(pose)), vel=vel, acc=acc, ref=self.dr.DR_BASE)
        actual = self.pose()
        if any(abs(actual[i] - pose[i]) > tol for i in range(3)):
            raise RuntimeError('이동 목표 미도달')

    def movej(self, joints, vel, acc):
        self._guard()
        self._prepare()
        self.moved = True
        self._dr_call('movej', self.dr.posj(list(joints)), vel=vel, acc=acc)

    def amovel_rel(self, delta, vel=None, acc=None, time_s=None):
        """비동기 상대 직선 이동 (BASE 기준 [dx, dy, dz, da, db, dc]). 바로 돌아온다.
        끝까지 기다리려면 mwait(), 중간에 끊으려면 move_stop(). time_s를 주면 vel·acc 대신 시간 기준."""
        self._guard()
        self._prepare()
        self.moved = True
        target = self.dr.posx([float(v) for v in delta])
        if time_s is not None:
            self._dr_call('amovel', target, time=float(time_s), ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)
        else:
            self._dr_call('amovel', target, vel=vel, acc=acc, ref=self.dr.DR_BASE, mod=self.dr.DR_MV_MOD_REL)

    def mwait(self):
        """비동기 이동이 끝날 때까지 대기 (기다리는 동안은 정지 확인을 못 함 — 관제 정지는 move_stop으로 로봇이 멈춤)"""
        self._prepare()
        self._dr_call('mwait')

    def move_stop(self, mode=2) -> bool:
        """지금 하는 이동만 멈춘다 (힘 제어 해제 없음, 정지 확인 없음). 1 빠른 정지 / 2 부드러운 정지.
        접촉을 감지해서 비동기 하강을 끊을 때 쓴다. 성공 여부 반환."""
        self._prepare()
        req = self._srv.MoveStop.Request()
        req.stop_mode = int(mode)
        try:
            return bool(self._request('stop', req, timeout=2.0).success)
        except RuntimeError:
            return False

    # ------------------------------------------------------------------
    # 그리퍼 (RG2 Modbus, box_comm/gripper.py)
    # ------------------------------------------------------------------
    @property
    def gripper(self):
        """RG2 객체 (처음 부를 때 접속). Goal이 끝나면 unbind()가 접속을 끊는다."""
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
        """지정한 패드 간격으로 열기 (기본 common.GRIPPER) → 원시 폭 반환"""
        self._guard()
        self.moved = True
        return self.gripper.open(force_n or GRIPPER['open_force_n'], width_mm or GRIPPER['open_width_mm'])

    def grip_close(self, force_n=None):
        """닫고 잡힘 여부·폭 반환: GripResult(grabbed, width_mm, raw_mm). 못 잡아도 예외 없음
        → 파트가 grabbed를 보고 다시 집거나 PartError('GRASP_FAILED')"""
        self._guard()
        self.moved = True
        return self.gripper.grip(force_n or GRIPPER['grip_force_n'])

    def gripper_width(self) -> float:
        """지금 폭 (패드 사이, 보정 반영) [mm]"""
        return self.gripper.width()

    def grip(self, opened: bool, wait_s=None):
        """예전 이름 그대로 (11/17 이전 DO 방식과 같은 호출). 기본 폭·힘으로 열기/닫기.
        wait_s는 쓰지 않는다 (Modbus는 동작이 끝날 때까지 기다림). 닫기는 GripResult를 돌려준다."""
        if opened:
            self.grip_open()
            return None
        return self.grip_close()

    def compliance_on(self, stx):
        self._guard()
        self._prepare()
        self.moved = True
        self.compliance_active = True
        req = self._srv.TaskComplianceCtrl.Request()
        req.stx = [float(v) for v in stx]
        req.ref, req.time = 0, 0.0
        self._request('compliance', req)

    def force_on(self, fd, direction, time_s=0.5):
        self._guard()
        self._prepare()
        self.moved = True
        self.force_active = True
        req = self._srv.SetDesiredForce.Request()
        req.fd = [float(v) for v in fd]
        req.dir = [int(v) for v in direction]
        req.ref, req.time, req.mod = 0, float(time_s), self.dr.DR_FC_MOD_REL
        self._request('target_force', req)

    def stop(self, emergency=False):
        """힘·순응제어 해제 → (emergency면) move_stop → 동작이 멈출 때까지 최대 5초 확인.
        정지 확인은 하지 않는다 (정지하는 중에 CANCELED로 끊기면 안 되므로)."""
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
                self._request('release_compliance', self._srv.ReleaseComplianceCtrl.Request())
                self.compliance_active = False
            except Exception as exc:
                errors.append(exc)
        if emergency or errors:
            req = self._srv.MoveStop.Request()
            req.stop_mode = STOP_MODE
            self._request('stop', req, timeout=5.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._request('motion', self._srv.CheckMotion.Request()).status == 0:
                if errors:
                    raise RuntimeError(f'힘·순응제어 해제 실패: {errors}')
                return
            time.sleep(0.05)
        raise RuntimeError('정지 확인 실패')
