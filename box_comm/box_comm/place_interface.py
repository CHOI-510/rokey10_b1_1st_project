#!/usr/bin/env python3
"""
place_interface.py — 적재 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /place (box_interfaces/action/Place)를 띄운다.
  - Goal의 result_class와 inventory를 파이썬 값으로 바꿔 작업 클래스의 run()에 넘긴다.
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.
  - 실패하면 적재 전 inventory를 그대로 돌려준다.

작업 클래스 약속 (box_place/place.py에서 작성)
  class PlaceWork:
      def __init__(self, node): ...
      def run(self, result_class, inventory, report) -> dict:
          # result_class = 'A' / 'B' / 'C' / 'D'
          # inventory    = {'A': 1, 'B': 0, 'C': 2, 'D': 0}   (적재 전)
          # 적재 후 inventory dict를 같은 형식으로 반환
          # 슬롯 배정·좌표는 이 안에서만 처리 (인터페이스에 나오지 않음)
          # 실패하면 raise PartError('PLACE_FAILED', '이유')
          # 단계가 바뀔 때마다 report(stage) 호출 (필수, stage 목록은 common.STAGES['place'])

실행
  box_place/place.py의 main()에서 start(PlaceWork) 호출
================================================================
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from box_interfaces.action import Place
from box_interfaces.msg import Inventory

from box_comm.common import ERROR_CODES, PartError, STAGES, VALID_CLASSES  # noqa: F401

NODE_NAME = 'place'
ACTION_NAME = '/place'
DEFAULT_ERROR = 'PLACE_FAILED'   # 예상 못 한 예외일 때 보내는 코드


def _inv_to_dict(inv: Inventory) -> dict:
    return {'A': inv.a, 'B': inv.b, 'C': inv.c, 'D': inv.d}


def _dict_to_inv(d: dict) -> Inventory:
    if set(d.keys()) != set(VALID_CLASSES):
        raise ValueError(f'inventory는 {VALID_CLASSES} 키를 모두 가져야 함 (받은 값: {d!r})')
    return Inventory(a=int(d['A']), b=int(d['B']), c=int(d['C']), d=int(d['D']))


class PlaceInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.work = work_cls(self)
        self._server = ActionServer(self, Place, ACTION_NAME, self._execute)
        self.get_logger().info(f'[place] Action 서버 {ACTION_NAME} 준비')

    def _execute(self, goal_handle):
        req = goal_handle.request
        result_class = req.result_class
        inv_before = _inv_to_dict(req.inventory)
        self.get_logger().info(
            f'[place] Goal 수신: class={result_class}, inventory(전)={inv_before}')
        result = Place.Result()

        def report(stage: str):
            stage = str(stage)
            if stage not in STAGES['place']:
                self.get_logger().warn(
                    f'[place] 등록되지 않은 stage: "{stage}" (사용 가능: {STAGES["place"]})')
            goal_handle.publish_feedback(Place.Feedback(stage=stage))
            self.get_logger().info(f'[place] stage={stage}')

        try:
            if result_class not in VALID_CLASSES:
                raise PartError('PLACE_FAILED', f'알 수 없는 result_class: "{result_class}"')
            inv_after = self.work.run(result_class, dict(inv_before), report)
            result.success = True
            result.inventory = _dict_to_inv(inv_after)
            result.error_code = ''
            goal_handle.succeed()
            self.get_logger().info(f'[place] Result: inventory(후)={inv_after}')

        except PartError as e:
            self.get_logger().error(f'[place] 실패 {e.code}: {e}')
            result.success = False
            result.inventory = req.inventory
            result.error_code = e.code
            goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f'[place] 예외: {type(e).__name__}: {e}')
            result.success = False
            result.inventory = req.inventory
            result.error_code = DEFAULT_ERROR
            goal_handle.abort()

        return result


def start(work_cls, args=None):
    rclpy.init(args=args)
    node = PlaceInterface(work_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
