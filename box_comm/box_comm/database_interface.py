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
  실패하면 아무 예외나 raise → success=False, message에 사유가 담긴다.

실행
  box_database/database.py의 main()에서 start(DatabaseWork) 호출
================================================================
"""

import rclpy
from rclpy.node import Node

from box_interfaces.msg import ClassSpec, Inventory
from box_interfaces.srv import GetJobData, SetJobStatus, UpdateInventory

NODE_NAME = 'database'
SRV_GET_JOB_DATA = '/get_job_data'
SRV_UPDATE_INVENTORY = '/update_inventory'
SRV_SET_JOB_STATUS = '/set_job_status'

VALID_CLASSES = ('A', 'B', 'C', 'D')
VALID_STATUS = ('BEFORE', 'RUNNING', 'DONE')


def _inv_to_dict(inv: Inventory) -> dict:
    return {'A': inv.a, 'B': inv.b, 'C': inv.c, 'D': inv.d}


def _dict_to_inv(d: dict) -> Inventory:
    if set(d.keys()) != set(VALID_CLASSES):
        raise ValueError(f'inventory는 {VALID_CLASSES} 키를 모두 가져야 함 (받은 값: {d!r})')
    return Inventory(a=int(d['A']), b=int(d['B']), c=int(d['C']), d=int(d['D']))


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
        self.get_logger().info(
            f'[database] Service 서버 준비: {SRV_GET_JOB_DATA}, '
            f'{SRV_UPDATE_INVENTORY}, {SRV_SET_JOB_STATUS}')

    def _on_get_job_data(self, request, response):
        try:
            specs, inventory = self.work.get_job_data()
            response.specs = [_dict_to_spec(s) for s in specs]
            response.inventory = _dict_to_inv(inventory)
            response.success = True
            response.message = ''
            self.get_logger().info(
                f'[database] GetJobData → specs {len(specs)}개, inventory={inventory}')
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
