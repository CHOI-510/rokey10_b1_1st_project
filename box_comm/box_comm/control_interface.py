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

  scan()                         -> (found: bool, err)
  judge(specs)                   -> (result_class: str, err)
  place(result_class, inventory) -> (inventory_after: dict, err)
  get_job_data()                 -> (specs: list, inventory: dict, poses: dict|None, err)
    poses = {'home_j': [6], 'measure_zone': [3]} (DB 사본), DB에 없으면 None
  update_inventory(box_id, inv)  -> err
  set_job_status(status)         -> err

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

  TODO(나중에): 재시도, 에러 정책, 정지 버튼, Action별 timeout 값
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
from box_interfaces.msg import ClassSpec, Inventory
from box_interfaces.srv import GetJobData, SetJobStatus, UpdateInventory

NODE_NAME = 'control'
SERVER_WAIT_SEC = 3.0        # 서버가 떠 있는지 기다리는 시간


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

        # Feedback 진행 추적: name -> (stage, 받은 시각)
        self._progress_lock = threading.Lock()
        self._progress = {}

        self.get_logger().info('[control] 클라이언트 준비 (Action 3, Service 3)')

    # ==================================================================
    # 파트 (Action)
    # ==================================================================
    def scan(self, timeout=None):
        res, err = self._call_action(self._scan, Scan.Goal(), 'scan', timeout)
        if err:
            return False, err
        if not res.success:
            return False, res.error_code or 'NO_ERROR_CODE'
        return res.found, None

    def judge(self, specs, timeout=None):
        goal = Judge.Goal(specs=[_dict_to_spec(s) for s in specs])
        res, err = self._call_action(self._judge, goal, 'judge', timeout)
        if err:
            return '', err
        if not res.success:
            return '', res.error_code or 'NO_ERROR_CODE'
        return res.result_class, None

    def place(self, result_class, inventory, timeout=None):
        goal = Place.Goal(result_class=result_class, inventory=_dict_to_inv(inventory))
        res, err = self._call_action(self._place, goal, 'place', timeout)
        if err:
            return dict(inventory), err
        if not res.success:
            return _inv_to_dict(res.inventory), res.error_code or 'NO_ERROR_CODE'
        return _inv_to_dict(res.inventory), None

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
        poses = None
        if res.poses.available:
            poses = {'home_j': list(res.poses.home_j),
                     'measure_zone': list(res.poses.measure_zone)}
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

    def set_job_status(self, status, timeout=None):
        res, err = self._call_service(
            self._set_status, SetJobStatus.Request(status=status), 'SetJobStatus', timeout)
        if err:
            return err
        if not res.success:
            return 'DB_FAILED'
        return None

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

        res_fut = handle.get_result_async()
        if not self._wait(res_fut, timeout):
            handle.cancel_goal_async()
            self.get_logger().error(f'[control] /{name} 시간 초과 → cancel')
            return None, 'TIMEOUT'

        return res_fut.result().result, None

    def _call_service(self, client, request, name, timeout):
        if not client.wait_for_service(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(f'[control] {name} 서비스 없음')
            return None, 'SERVER_DOWN'
        fut = client.call_async(request)
        if not self._wait(fut, timeout):
            return None, 'TIMEOUT'
        return fut.result(), None

    def _on_feedback(self, name, feedback_msg):
        stage = feedback_msg.feedback.stage
        self._set_progress(name, stage)
        self.get_logger().info(f'[control] /{name} feedback: {stage}')

    @staticmethod
    def _wait(fut, timeout):
        """future가 끝날 때까지 대기. timeout=None이면 무한 대기."""
        if fut.done():
            return True
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        return ev.wait(timeout)


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
