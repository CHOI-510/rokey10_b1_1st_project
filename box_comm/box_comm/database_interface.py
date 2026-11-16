#!/usr/bin/env python3
"""
database_interface.py — DB 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Service 서버 3개를 띄운다 (box_interfaces/srv)
      /get_job_data      GetJobData       작업 시작 시
      /update_inventory  UpdateInventory  적재완료마다
      /set_job_status    SetJobStatus     작업중 / 작업완료 시점
  - ROS 메시지를 파이썬 값으로 바꿔 작업 클래스의 메서드를 호출한다.
  - 이벤트 로그 토픽 /event_log (LogEvent)를 구독해서 작업 클래스의 on_event(event)를 부른다.
    (관제가 보내기만 하는 로그 — 응답 없음, 누락 가능)
  - 계약 규칙을 여기서 강제한다.
      · 같은 box_id는 한 번만 반영 (중복이면 작업 클래스를 호출하지 않음)
      · status는 BEFORE / RUNNING / DONE만 허용

작업 클래스 약속 (box_database/database.py에서 작성)
  class DatabaseWork:
      def __init__(self, node): ...
      def get_job_data(self) -> tuple[list, dict]:
          # (specs, inventory) 반환
          # specs     = [{'class_id': 'A', 'width_mm': .., 'depth_mm': ..,
          #               'height_mm': .., 'weight_g': ..}, ...]
          # inventory = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
      def update_inventory(self, box_id: str, inventory: dict) -> None:
          # DB에 재고 기록 + UI 갱신
      def set_job_status(self, status: str) -> None:
          # DB에 작업 상태 기록 + UI 갱신
      def get_fixed_poses(self) -> dict | None:      # 필수 (없거나 None이면 관제가 시작하지 않음)
          # DB에 저장된 고정 좌표 — 기준값 (v0.3: DB가 기준, 관제가 이 값으로 덮어씀)
          # {'home_j': [6개, deg], 'measure_zone': [x, y, z, mm],
          #  'place_zones': {'A': [x, y], 'B': [x, y], 'C': [x, y], 'D': [x, y]}}   ← v0.4
          # place_zones = 클래스별 적재 구역 '1단' x, y [mm, base]. 단 간격·z는 적재만 가짐
      def on_event(self, event: dict) -> None:       # 선택 (없으면 로그를 받지 않음)
          # event = {'stamp', 'job_id', 'source', 'stage', 'level', 'code', 'box_id',
          #          'class_id', 'expected', 'measured', 'message'}
          # DB에 저장 + UI 로그 창에 message 표시. code·level로 걸러서 보여 주면 됨
  재고 → UI 적재 이미지: box_comm.common.stack_layout(수량) → 단별 층 수 (예: 6 → [4, 2])
  실패하면 아무 예외나 raise → success=False, message에 사유가 담긴다.

실행
  box_database/database.py의 main()에서 start(DatabaseWork) 호출
================================================================
"""

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from box_interfaces.msg import ClassSpec, FixedPoses, Inventory, LogEvent, PlaceZone
from box_interfaces.srv import GetJobData, SetJobStatus, UpdateInventory

from box_comm.common import LOG_TOPIC, VALID_CLASSES, log_qos

NODE_NAME = 'database'
SRV_GET_JOB_DATA = '/get_job_data'
SRV_UPDATE_INVENTORY = '/update_inventory'
SRV_SET_JOB_STATUS = '/set_job_status'

VALID_STATUS = ('BEFORE', 'RUNNING', 'DONE')


def _inv_to_dict(inv: Inventory) -> dict:
    return {'A': inv.a, 'B': inv.b, 'C': inv.c, 'D': inv.d}


def _dict_to_inv(d: dict) -> Inventory:
    if set(d.keys()) != set(VALID_CLASSES):
        raise ValueError(f'inventory는 {VALID_CLASSES} 키를 모두 가져야 함 (받은 값: {d!r})')
    return Inventory(a=int(d['A']), b=int(d['B']), c=int(d['C']), d=int(d['D']))


def _dict_to_poses(d) -> FixedPoses:
    """{'home_j': [6], 'measure_zone': [3], 'place_zones': {'A': [x, y], ...}} → FixedPoses.
    None이면 available=False. place_zones는 v0.4 (없거나 None인 클래스는 빠짐 → 관제 사전 점검 FAIL)"""
    msg = FixedPoses()
    if not d:
        msg.available = False
        return msg
    home = [float(v) for v in d['home_j']]
    zone = [float(v) for v in d['measure_zone']]
    if len(home) != 6 or len(zone) != 3:
        raise ValueError(f'고정 좌표 길이 오류: home_j {len(home)}개(6), measure_zone {len(zone)}개(3)')
    zones = []
    for c, xy in (d.get('place_zones') or {}).items():
        if xy is None:
            continue
        if c not in VALID_CLASSES or len(xy) != 2:
            raise ValueError(f'적재 구역 형식 오류: {c}={xy} (클래스 {VALID_CLASSES}, [x, y] 2개)')
        zones.append(PlaceZone(class_id=c, x=float(xy[0]), y=float(xy[1])))
    msg.available = True
    msg.home_j = home
    msg.measure_zone = zone
    msg.place_zones = zones
    return msg


def _dict_to_spec(d: dict) -> ClassSpec:
    return ClassSpec(
        class_id=str(d['class_id']),
        width_mm=float(d['width_mm']),
        depth_mm=float(d['depth_mm']),
        height_mm=float(d['height_mm']),
        weight_g=float(d['weight_g']),
    )


class DatabaseInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.work = work_cls(self)
        self._applied_box_ids = set()

        self.create_service(GetJobData, SRV_GET_JOB_DATA, self._on_get_job_data)
        self.create_service(UpdateInventory, SRV_UPDATE_INVENTORY, self._on_update_inventory)
        self.create_service(SetJobStatus, SRV_SET_JOB_STATUS, self._on_set_job_status)
        self._on_event_fn = getattr(self.work, 'on_event', None)
        if self._on_event_fn:
            self.create_subscription(LogEvent, LOG_TOPIC, self._on_log_event, log_qos())
        self.get_logger().info(
            f'[database] Service 서버 준비: {SRV_GET_JOB_DATA}, '
            f'{SRV_UPDATE_INVENTORY}, {SRV_SET_JOB_STATUS} | '
            f'로그 구독 {LOG_TOPIC}: {"on" if self._on_event_fn else "off (on_event 없음)"}')

    def _on_get_job_data(self, request, response):
        try:
            specs, inventory = self.work.get_job_data()
            getter = getattr(self.work, 'get_fixed_poses', None)
            poses = getter() if getter else None
            response.specs = [_dict_to_spec(s) for s in specs]
            response.inventory = _dict_to_inv(inventory)
            response.poses = _dict_to_poses(poses)
            response.success = True
            response.message = ''
            self.get_logger().info(
                f'[database] GetJobData → specs {len(specs)}개, inventory={inventory}, '
                f'poses={"있음" if poses else "없음"}, '
                f'적재 구역 {len(response.poses.place_zones)}개')
        except Exception as e:
            response.success = False
            response.message = f'{type(e).__name__}: {e}'
            self.get_logger().error(f'[database] GetJobData 실패: {response.message}')
        return response

    def _on_update_inventory(self, request, response):
        box_id = request.box_id
        try:
            if box_id in self._applied_box_ids:
                response.success = True
                response.message = f'duplicate box_id {box_id}, ignored'
                self.get_logger().warn(f'[database] UpdateInventory 중복 무시: {box_id}')
                return response

            inventory = _inv_to_dict(request.inventory)
            self.work.update_inventory(box_id, inventory)
            self._applied_box_ids.add(box_id)
            response.success = True
            response.message = ''
            self.get_logger().info(f'[database] UpdateInventory {box_id} → {inventory}')
        except Exception as e:
            response.success = False
            response.message = f'{type(e).__name__}: {e}'
            self.get_logger().error(f'[database] UpdateInventory 실패: {response.message}')
        return response

    def _on_log_event(self, msg):
        event = {k: getattr(msg, k) for k in (
            'stamp', 'job_id', 'source', 'stage', 'level', 'code', 'box_id',
            'class_id', 'expected', 'measured', 'message')}
        try:
            self._on_event_fn(event)
        except Exception as e:      # 로그 처리 오류가 DB 서비스를 멈추지 않게
            self.get_logger().error(f'[database] on_event 실패: {type(e).__name__}: {e}')

    def _on_set_job_status(self, request, response):
        try:
            if request.status not in VALID_STATUS:
                raise ValueError(f'알 수 없는 status: "{request.status}" (사용 가능: {VALID_STATUS})')
            self.work.set_job_status(request.status)
            response.success = True
            self.get_logger().info(f'[database] SetJobStatus → {request.status}')
        except Exception as e:
            response.success = False
            self.get_logger().error(f'[database] SetJobStatus 실패: {type(e).__name__}: {e}')
        return response


def start(work_cls, args=None):
    rclpy.init(args=args)
    node = DatabaseInterface(work_cls)
    # 전용 executor로 spin한다 (rclpy.spin은 전역 executor를 씀).
    # → run() 안에서 DSR_ROBOT2 함수(movej 등)가 전역 executor로 응답을 기다려도
    #   'Executor is already spinning' 충돌이 나지 않는다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
