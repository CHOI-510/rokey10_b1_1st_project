#!/usr/bin/env python3
"""
control_interface.py — 관제 통신 계층
================================================================
관리: 관제(정쓰).

하는 일
  - Action 클라이언트 3개 (/scan, /judge, /place)
  - Service 클라이언트 3개 (/get_job_data, /update_inventory, /set_job_status)
  - ROS 메시지 ↔ 파이썬 값 변환
  - 실행 구조: MultiThreadedExecutor는 별도 스레드에서 spin,
               관제 시퀀스(run)는 메인 스레드에서 위에서 아래로 실행.
               → 호출이 블로킹처럼 보여도 데드락이 생기지 않는다.

관제 시퀀스(control.py)에서 쓰는 함수 — 모두 (값..., err) 형태로 반환
  err가 None이면 성공. 아니면 파트 error_code 또는 관제 내부 코드.

  scan(poses)                         -> (found: bool, err)
  judge(specs, poses)                 -> (result_class: str, err)
  place(result_class, poses)      -> (count: int|None, err)
    count = 적재가 실제로 놓은 위치로 잰 result_class 수량 (v0.4)
    poses = {'home_j': [6], 'measure_zone': [3], 'place_zones': {'A': [x, y], ...}} (DB 값).
            모든 Goal에 실어 보낸다.
  get_job_data()                 -> (specs: list, inventory: dict, poses: dict|None, err)
    poses = 위와 같은 형식 (DB 값), DB에 없으면 None
  update_inventory(box_id, inv)  -> err
  set_job_status(status, reason='') -> err
    status: BEFORE / RUNNING / DONE / ERROR / STOPPED, reason: ERROR·STOPPED일 때 UI 표시 이유

  이벤트 로그 (/event_log 토픽, 보내기만 함)
  log_event(level, code, message, box_id='', class_id='', expected=-1, measured=-1)
    관제 자신의 판단(작업 시작·완료·중단, 재고 불일치 등)을 올린다.
  set_log_context(job_id, box_id='', class_id='')
    지금 처리 중인 박스. 파트 Feedback 로그에 이 값이 붙는다.
  파트 Feedback에 log_msg가 있으면(report.log) 자동으로 /event_log에 넘긴다 (code=PART_LOG).
  진행만 알리는 Feedback(report(stage))은 넘기지 않는다.

  정지 기능 (common.SAFETY)
  stop_reason()                  -> (code, detail) 또는 None
    정지가 요청됐는지. 요청되면 진행 중인 Action은 바로 그 code로 끝나고(err),
    이후 Action 호출도 모두 그 code를 돌려준다. (DB Service는 계속 동작 — 상태 기록용)
  request_stop(code, detail='')  관제 스스로 정지할 때 (첫 요청만 유효)
    → 두산 move_stop(반복) + 진행 중인 파트 Goal cancel
  arm_monitor(on)                작업 중(RUNNING)에만 로봇 상태 이상을 정지로 처리
  robot_state()                  -> (state: int|None, name: str)  사전 점검용

  정지가 일어나는 경우
    /emergency_stop 서비스 요청 (UI 버튼 → DB, 또는 터미널)      → EMERGENCY_STOP (언제든)
    로봇 상태가 STANDBY/MOVING이 아님 (robot_monitor=dsr/fake)   → ROBOT_* (작업 중에만)
    로봇 상태 확인 연속 실패 / robot_disconnection 토픽          → ROBOT_DISCONNECTED (작업 중에만)
    파트 Feedback이 progress_timeout_sec 동안 없음               → NO_PROGRESS (작업 중에만)
  파라미터: robot_monitor (dsr / fake / off), fake_robot_state (int), progress_timeout_sec (float)

  진행 추적 (Feedback 기반)
  progress(name)                 -> (last_stage: str, seconds_since: float)
    name = 'scan' / 'judge' / 'place'
    Goal을 보내면 stage='goal_sent'로 초기화, Feedback이 올 때마다 갱신.
    → "어느 단계에서 몇 초째 멈춰 있는지" 확인용. (개입 로직은 safety에서 추가 예정)

  관제 내부 코드 (파트가 보내는 코드가 아님)
    SERVER_DOWN  서버가 떠 있지 않음
    REJECTED     Goal 거절됨
    TIMEOUT      시간 초과 (timeout을 지정했을 때만)
    NO_ERROR_CODE  파트가 실패했는데 error_code를 비워 보냄
    DB_FAILED    DB 서비스가 success=False

  TODO(나중에): 재시도, 에러 정책
================================================================
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from box_interfaces.action import Judge, Place, Scan
from box_interfaces.msg import ClassSpec, FixedPoses, Inventory, LogEvent, PlaceZone

from box_comm.common import (LOG_LEVELS, LOG_TOPIC, ROBOT, ROBOT_STATE, SAFETY, log_qos,
                             poses_from_msg, robot_stop_code)
from box_interfaces.srv import EmergencyStop, GetJobData, SetJobStatus, UpdateInventory
from rcl_interfaces.msg import ParameterDescriptor

NODE_NAME = 'control'
SERVER_WAIT_SEC = 3.0        # 서버가 떠 있는지 기다리는 시간
SRV_EMERGENCY_STOP = '/emergency_stop'
WAIT_SLICE = 0.05            # Action 결과를 기다리며 정지 요청을 확인하는 간격


# ----------------------------------------------------------------------
# 변환 헬퍼
# ----------------------------------------------------------------------
def _inv_to_dict(inv: Inventory) -> dict:
    return {'A': inv.a, 'B': inv.b, 'C': inv.c, 'D': inv.d}


def _dict_to_inv(d: dict) -> Inventory:
    return Inventory(a=int(d['A']), b=int(d['B']), c=int(d['C']), d=int(d['D']))


def _spec_to_dict(s: ClassSpec) -> dict:
    return {'class_id': s.class_id, 'width_mm': float(s.width_mm),
            'depth_mm': float(s.depth_mm), 'height_mm': float(s.height_mm),
            'weight_g': float(s.weight_g)}


def _dict_to_poses(d) -> FixedPoses:
    msg = FixedPoses()
    if not d or d.get('measure_zone') is None:
        msg.available = False
        return msg
    msg.available = True
    msg.home_j = [float(v) for v in d['home_j']]
    msg.measure_zone = [float(v) for v in d['measure_zone']]
    msg.place_zones = [PlaceZone(class_id=c, x=float(xy[0]), y=float(xy[1]))
                       for c, xy in (d.get('place_zones') or {}).items() if xy is not None]
    return msg


def _dict_to_spec(d: dict) -> ClassSpec:
    return ClassSpec(class_id=str(d['class_id']), width_mm=float(d['width_mm']),
                     depth_mm=float(d['depth_mm']), height_mm=float(d['height_mm']),
                     weight_g=float(d['weight_g']))


class ControlInterface(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        cb = ReentrantCallbackGroup()

        self._scan = ActionClient(self, Scan, '/scan', callback_group=cb)
        self._judge = ActionClient(self, Judge, '/judge', callback_group=cb)
        self._place = ActionClient(self, Place, '/place', callback_group=cb)

        self._get_job = self.create_client(GetJobData, '/get_job_data', callback_group=cb)
        self._update_inv = self.create_client(UpdateInventory, '/update_inventory', callback_group=cb)
        self._set_status = self.create_client(SetJobStatus, '/set_job_status', callback_group=cb)

        # 이벤트 로그 (보내기만 함)
        self._log_pub = self.create_publisher(LogEvent, LOG_TOPIC, log_qos())
        self._log_ctx = {'job_id': '', 'box_id': '', 'class_id': ''}

        # Feedback 진행 추적: name -> (stage, 받은 시각)
        self._progress_lock = threading.Lock()
        self._progress = {}

        # 정지 기능
        self._stop_lock = threading.Lock()
        self._stop = None                  # (code, detail) — 첫 요청만 유효
        self._stop_event = threading.Event()
        self._active = None                # (name, goal_handle) 실행 중인 Action
        self._armed = False                # 작업 중(RUNNING)에만 이상 감지를 정지로 처리
        self._robot_state = None           # 마지막으로 읽은 로봇 상태 (int)
        self._robot_lost = 0               # 상태 확인 연속 실패 횟수
        self._state_pending = False
        self._last_alarm = ''              # 두산 알람 메시지 (reason에 덧붙임)
        self._dsr = None                   # dsr_msgs2 모듈 (없으면 None)

        dyn = ParameterDescriptor(dynamic_typing=True)
        self._monitor = str(self.declare_parameter('robot_monitor', 'off', dyn).value).lower()
        self.declare_parameter('fake_robot_state', 1)            # fake 모드: ros2 param set으로 변경
        self._progress_timeout = float(self.declare_parameter('progress_timeout_sec', 0.0).value)
        if self._monitor not in ('dsr', 'fake', 'off'):
            self.get_logger().warn(f'[control] robot_monitor="{self._monitor}" 알 수 없음 → off')
            self._monitor = 'off'

        self.create_service(EmergencyStop, SRV_EMERGENCY_STOP, self._on_emergency_stop, callback_group=cb)
        self._setup_dsr(cb)
        self.create_timer(SAFETY['monitor_period_sec'], self._on_monitor_tick, callback_group=cb)

        self.get_logger().info(
            f'[control] 클라이언트 준비 (Action 3, Service 3, 로그 토픽 {LOG_TOPIC}) | '
            f'비상정지 {SRV_EMERGENCY_STOP} | robot_monitor={self._monitor} | '
            f'progress_timeout_sec={self._progress_timeout or "off"}')

    # ==================================================================
    # 파트 (Action)
    # ==================================================================
    def scan(self, poses=None, timeout=None):
        res, err = self._call_action(self._scan, Scan.Goal(poses=_dict_to_poses(poses)), 'scan', timeout)
        if err:
            return False, err
        if not res.success:
            return False, res.error_code or 'NO_ERROR_CODE'
        return res.found, None

    def judge(self, specs, poses=None, timeout=None):
        goal = Judge.Goal(specs=[_dict_to_spec(s) for s in specs], poses=_dict_to_poses(poses))
        res, err = self._call_action(self._judge, goal, 'judge', timeout)
        if err:
            return '', err
        if not res.success:
            return '', res.error_code or 'NO_ERROR_CODE'
        return res.result_class, None

    def place(self, result_class, poses=None, timeout=None):
        goal = Place.Goal(result_class=result_class, poses=_dict_to_poses(poses))
        res, err = self._call_action(self._place, goal, 'place', timeout)
        if err:
            return None, err
        if not res.success:
            return None, res.error_code or 'NO_ERROR_CODE'
        return int(res.count), None

    # ==================================================================
    # DB (Service)
    # ==================================================================
    def get_job_data(self, timeout=None):
        res, err = self._call_service(self._get_job, GetJobData.Request(), 'GetJobData', timeout)
        if err:
            return [], {}, None, err
        if not res.success:
            self.get_logger().error(f'[control] GetJobData 실패: {res.message}')
            return [], {}, None, 'DB_FAILED'
        poses = poses_from_msg(res.poses)      # available=False면 None
        return ([_spec_to_dict(s) for s in res.specs], _inv_to_dict(res.inventory),
                poses, None)

    def update_inventory(self, box_id, inventory, timeout=None):
        req = UpdateInventory.Request(box_id=box_id, inventory=_dict_to_inv(inventory))
        res, err = self._call_service(self._update_inv, req, 'UpdateInventory', timeout)
        if err:
            return err
        if not res.success:
            self.get_logger().error(f'[control] UpdateInventory 실패: {res.message}')
            return 'DB_FAILED'
        return None

    def set_job_status(self, status, reason='', timeout=None):
        res, err = self._call_service(
            self._set_status, SetJobStatus.Request(status=status, reason=reason), 'SetJobStatus', timeout)
        if err:
            return err
        if not res.success:
            return 'DB_FAILED'
        return None

    # ==================================================================
    # 이벤트 로그
    # ==================================================================
    def set_log_context(self, job_id=None, box_id=None, class_id=None):
        for k, v in (('job_id', job_id), ('box_id', box_id), ('class_id', class_id)):
            if v is not None:
                self._log_ctx[k] = str(v)

    def log_event(self, level, code, message, box_id=None, class_id=None,
                  expected=-1, measured=-1, source='control', stage=''):
        """/event_log에 한 줄 발행. 실패해도 예외를 밖으로 내보내지 않는다 (작업을 막지 않음)."""
        try:
            level = level if level in LOG_LEVELS else 'INFO'
            msg = LogEvent(
                stamp=time.strftime('%Y-%m-%d %H:%M:%S'),
                job_id=self._log_ctx['job_id'],
                source=str(source), stage=str(stage), level=level, code=str(code),
                box_id=self._log_ctx['box_id'] if box_id is None else str(box_id),
                class_id=self._log_ctx['class_id'] if class_id is None else str(class_id),
                expected=int(expected), measured=int(measured), message=str(message))
            self._log_pub.publish(msg)
        except Exception as e:      # 로그 때문에 작업이 멈추면 안 됨
            self.get_logger().warn(f'[control] 이벤트 로그 발행 실패: {type(e).__name__}: {e}')

    # ==================================================================
    # 정지 기능
    # ==================================================================
    def stop_reason(self):
        with self._stop_lock:
            return self._stop

    def arm_monitor(self, on: bool):
        self._armed = bool(on)
        self._robot_lost = 0

    def robot_state(self):
        """사전 점검용: (state, 이름). 모니터가 꺼져 있거나 아직 못 읽었으면 (None, 이유)"""
        if self._monitor == 'off':
            return None, 'robot_monitor=off'
        if self._monitor == 'fake':
            st = int(self.get_parameter('fake_robot_state').value)
            return st, ROBOT_STATE.get(st, str(st))
        deadline = time.monotonic() + SERVER_WAIT_SEC       # dsr: 첫 응답을 잠깐 기다림
        while self._robot_state is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self._robot_state is None:
            return None, '로봇 상태 응답 없음 (get_robot_state)'
        return self._robot_state, ROBOT_STATE.get(self._robot_state, str(self._robot_state))

    def request_stop(self, code, detail=''):
        """정지 요청 (첫 요청만 유효). 로봇 정지 + 진행 중인 파트 Goal cancel."""
        with self._stop_lock:
            if self._stop is not None:
                return False
            if not detail and self._last_alarm and code.startswith('ROBOT_'):
                detail = self._last_alarm
            self._stop = (code, detail)
        self._stop_event.set()
        self.get_logger().error(f'[control] !!!!! 정지 요청: {code}' + (f' — {detail}' if detail else ''))
        self.log_event('ERROR', 'STOP_REQUEST', f'정지 요청: {code}' + (f' — {detail}' if detail else ''))
        self._robot_move_stop_hold()
        active = self._active
        if active:
            name, handle = active
            self.get_logger().warn(f'[control] /{name} Goal cancel 요청')
            handle.cancel_goal_async()
        return True

    def _on_emergency_stop(self, request, response):
        src = request.source or 'unknown'
        first = self.request_stop('EMERGENCY_STOP', f'by {src}')
        response.success = True
        response.message = '정지 요청 처리' if first else f'이미 정지 중: {self._stop[0]}'
        return response

    # ---- 두산 연결 (dsr_msgs2가 없으면 robot_monitor=dsr와 move_stop을 쓸 수 없음) ----
    def _setup_dsr(self, cb):
        try:
            import dsr_msgs2.msg as dsr_msg
            import dsr_msgs2.srv as dsr_srv
        except ImportError:
            if self._monitor == 'dsr':
                self.get_logger().error('[control] dsr_msgs2를 import할 수 없음 → robot_monitor=dsr 불가, off로 동작')
                self._monitor = 'off'
            self.get_logger().warn('[control] dsr_msgs2 없음 → 비상정지 시 로봇 move_stop 불가 (파트 cancel만)')
            return
        self._dsr = dsr_srv
        ns = ROBOT['service_ns'].rstrip('/')
        self._cli_move_stop = self.create_client(dsr_srv.MoveStop, f'{ns}/{SAFETY["srv_move_stop"]}', callback_group=cb)
        self._cli_robot_state = self.create_client(dsr_srv.GetRobotState, f'{ns}/{SAFETY["srv_robot_state"]}', callback_group=cb)
        self.create_subscription(dsr_msg.RobotError, SAFETY['topic_error'], self._on_robot_error, 10, callback_group=cb)
        self.create_subscription(dsr_msg.RobotDisconnection, SAFETY['topic_disconnect'],
                                 self._on_robot_disconnect, 10, callback_group=cb)

    def _robot_move_stop(self):
        if self._dsr is None or not self._cli_move_stop.service_is_ready():
            return False
        self._cli_move_stop.call_async(self._dsr.MoveStop.Request(stop_mode=int(SAFETY['stop_mode'])))
        return True

    def _robot_move_stop_hold(self):
        """move_stop을 stop_hold_sec 동안 반복 (파트가 cancel을 알아차리기 전에 보낸 동작도 멈춤)"""
        if not self._robot_move_stop():
            self.get_logger().warn('[control] move_stop 서비스 없음 → 로봇 정지 명령 못 보냄 (파트 cancel만)')
            return
        self.get_logger().error(f'[control] 로봇 move_stop(mode={SAFETY["stop_mode"]}) '
                                f'{SAFETY["stop_hold_sec"]}초 동안 반복')
        end = time.monotonic() + SAFETY['stop_hold_sec']

        def loop():
            while time.monotonic() < end:
                time.sleep(SAFETY['stop_resend_sec'])
                self._robot_move_stop()
        threading.Thread(target=loop, daemon=True).start()

    def _on_robot_error(self, msg):
        """두산 알람 (level 1 정보 / 2 경고 / 3 오류). 정보(TCP·툴 설정 안내 등)는 기록만 하고,
        경고·오류만 정지 이유에 덧붙인다. rclpy 로거는 같은 줄에서 레벨을 바꾸면 안 되므로 따로 호출."""
        text = ' / '.join(t for t in (msg.msg1, msg.msg2, msg.msg3) if t)
        alarm = f'alarm {msg.group}-{msg.code}' + (f': {text}' if text else '')
        if msg.level >= 3:
            self._last_alarm = alarm
            self.get_logger().error(f'[control] 두산 알람(오류) {alarm}')
            self.log_event('ERROR', 'ROBOT_ALARM', alarm)
        elif msg.level == 2:
            self._last_alarm = alarm
            self.get_logger().warn(f'[control] 두산 알람(경고) {alarm}')
            self.log_event('WARN', 'ROBOT_ALARM', alarm)
        else:
            self.get_logger().info(f'[control] 두산 알림 {alarm}')

    def _on_robot_disconnect(self, _msg):
        self.get_logger().error('[control] 로봇 연결 끊김 (robot_disconnection)')
        if self._armed:
            self.request_stop('ROBOT_DISCONNECTED')

    # ---- 주기 감시 (로봇 상태 + 진행) ----
    def _on_monitor_tick(self):
        if self._monitor == 'fake':
            self._check_robot_state(int(self.get_parameter('fake_robot_state').value))
        elif self._monitor == 'dsr' and not self._state_pending:
            if not self._cli_robot_state.service_is_ready():
                self._robot_state_failed('get_robot_state 서비스 없음')
            else:
                self._state_pending = True
                fut = self._cli_robot_state.call_async(self._dsr.GetRobotState.Request())
                fut.add_done_callback(self._on_robot_state_response)
        self._check_progress()

    def _on_robot_state_response(self, fut):
        self._state_pending = False
        res = fut.result()
        if res is None or not res.success:
            self._robot_state_failed('get_robot_state 실패')
            return
        self._robot_lost = 0
        self._check_robot_state(int(res.robot_state))

    def _robot_state_failed(self, why):
        self._robot_lost += 1
        if self._armed and self._robot_lost == SAFETY['lost_limit']:
            self.request_stop('ROBOT_DISCONNECTED', why)

    def _check_robot_state(self, state):
        if state != self._robot_state:
            self.get_logger().info(f'[control] 로봇 상태: {ROBOT_STATE.get(state, state)} ({state})')
            self._robot_state = state
        code = robot_stop_code(state)
        if code and self._armed:
            self.request_stop(code, f'robot_state={ROBOT_STATE.get(state, state)}')

    def _check_progress(self):
        active = self._active
        if not (active and self._armed and self._progress_timeout > 0):
            return
        name = active[0]
        stage, sec = self.progress(name)
        if sec > self._progress_timeout:
            self.request_stop('NO_PROGRESS', f'/{name} stage="{stage}" {sec:.0f}초 소식 없음')

    # ==================================================================
    # 진행 추적
    # ==================================================================
    def progress(self, name):
        """(마지막 stage, 그 뒤로 지난 초). 아직 Goal을 보낸 적이 없으면 ('', 0.0)."""
        with self._progress_lock:
            stage, t = self._progress.get(name, ('', None))
        return stage, (time.monotonic() - t) if t is not None else 0.0

    def _set_progress(self, name, stage):
        with self._progress_lock:
            self._progress[name] = (stage, time.monotonic())

    # ==================================================================
    # 공통 호출 (call_skill) — 타임아웃/재시도/cancel은 나중에 여기에만 추가
    # ==================================================================
    def _call_action(self, client, goal, name, timeout):
        if (stop := self.stop_reason()):                     # 이미 정지 요청 → 새 동작을 보내지 않음
            return None, stop[0]
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(f'[control] /{name} 서버 없음')
            return None, 'SERVER_DOWN'

        self._set_progress(name, 'goal_sent')
        send_fut = client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(name, fb))
        if not self._wait(send_fut, SERVER_WAIT_SEC):
            return None, 'TIMEOUT'

        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error(f'[control] /{name} Goal 거절됨')
            return None, 'REJECTED'

        self._active = (name, handle)
        try:
            if (stop := self.stop_reason()):                 # Goal을 보내는 사이에 정지 요청이 들어온 경우
                handle.cancel_goal_async()
                return None, stop[0]
            res_fut = handle.get_result_async()
            done = self._wait(res_fut, timeout, stoppable=True)
            if (stop := self.stop_reason()) and not res_fut.done():
                return None, stop[0]                         # cancel은 request_stop()이 이미 보냄
            if not done:
                handle.cancel_goal_async()
                self.get_logger().error(f'[control] /{name} 시간 초과 → cancel')
                return None, 'TIMEOUT'
            if (stop := self.stop_reason()):                 # 정지와 거의 동시에 끝남 → 정지가 우선
                return None, stop[0]
            return res_fut.result().result, None
        finally:
            self._active = None

    def _call_service(self, client, request, name, timeout):
        if not client.wait_for_service(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(f'[control] {name} 서비스 없음')
            return None, 'SERVER_DOWN'
        fut = client.call_async(request)
        if not self._wait(fut, timeout):
            return None, 'TIMEOUT'
        return fut.result(), None

    def _on_feedback(self, name, feedback_msg):
        fb = feedback_msg.feedback
        stage = fb.stage
        prev, _ = self.progress(name)
        self._set_progress(name, stage)    # 같은 stage가 다시 와도 시각은 갱신 (keepalive)
        if fb.log_msg:                     # report.log()로 보낸 것만 이벤트 로그로 넘김
            self.get_logger().info(f'[control] /{name} log {fb.log_level}: {fb.log_msg}')
            self.log_event(fb.log_level, 'PART_LOG', fb.log_msg, source=name, stage=stage)
        elif stage != prev:                # stage가 바뀔 때만 출력 (keepalive는 조용히)
            self.get_logger().info(f'[control] /{name} feedback: {stage}')

    def _wait(self, fut, timeout, stoppable=False):
        """future가 끝날 때까지 대기. timeout=None이면 무한 대기.
        stoppable=True면 정지 요청이 들어오는 즉시 False로 돌아온다."""
        if fut.done():
            return True
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        if not stoppable:
            return ev.wait(timeout)
        end = None if timeout is None else time.monotonic() + timeout
        while not ev.is_set():
            if self._stop_event.is_set():
                return False
            remain = WAIT_SLICE if end is None else min(WAIT_SLICE, end - time.monotonic())
            if remain <= 0:
                return False
            ev.wait(remain)
        return True

def start(work_cls, args=None):
    """executor는 백그라운드 스레드에서 spin, 관제 시퀀스는 메인 스레드에서 실행."""
    rclpy.init(args=args)
    node = ControlInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        work = work_cls(node)
        work.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)       # spin 스레드가 끝난 뒤에 노드 정리 (종료 시 Aborted 방지)
        node.destroy_node()
        rclpy.try_shutdown()
