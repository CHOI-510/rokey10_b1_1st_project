#!/usr/bin/env python3
"""
scan_interface.py — 탐지 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /scan (box_interfaces/action/Scan)을 띄운다.
  - Goal(시작명령)을 받으면 작업 클래스의 run(report)을 호출한다.
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.
    (어떤 경우에도 Result 없이 끝나지 않는다)

작업 클래스 약속 (box_scan/scan.py에서 작성)
  class ScanWork:
      def __init__(self, node): ...
      def run(self, report) -> bool:
          # report('scanning') 처럼 진행 단계 알림 (선택)
          # 물품을 측정존에 두었으면 True, 물품이 없으면 False
          # 실패하면 raise PartError('GRASP_FAILED', '이유')

실행
  box_scan/scan.py의 main()에서 start(ScanWork) 호출
================================================================
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from box_interfaces.action import Scan

NODE_NAME = 'scan'
ACTION_NAME = '/scan'

# 인터페이스 계약 2.4 — 새 코드는 관제가 등록 후 배포
ERROR_CODES = ('NOT_FOUND', 'GRASP_FAILED', 'OVER_FORCE',
               'MEASURE_FAILED', 'NO_SLOT', 'PLACE_FAILED')
DEFAULT_ERROR = 'GRASP_FAILED'   # 예상 못 한 예외일 때 보내는 코드


class PartError(Exception):
    """작업 실패를 관제에 알릴 때 사용. 등록된 코드만 허용."""

    def __init__(self, code: str, message: str = ''):
        if code not in ERROR_CODES:
            raise ValueError(f'등록되지 않은 error_code: "{code}" (사용 가능: {ERROR_CODES})')
        super().__init__(message or code)
        self.code = code


class ScanInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.work = work_cls(self)
        self._server = ActionServer(self, Scan, ACTION_NAME, self._execute)
        self.get_logger().info(f'[scan] Action 서버 {ACTION_NAME} 준비')

    def _execute(self, goal_handle):
        self.get_logger().info('[scan] Goal 수신: 시작명령')
        result = Scan.Result()

        def report(stage: str):
            goal_handle.publish_feedback(Scan.Feedback(stage=str(stage)))
            self.get_logger().info(f'[scan] stage={stage}')

        try:
            found = self.work.run(report)
            if not isinstance(found, bool):
                raise TypeError(f'run()은 bool을 반환해야 함 (받은 값: {found!r})')
            result.success = True
            result.found = found
            result.error_code = ''
            goal_handle.succeed()
            self.get_logger().info(f'[scan] Result: success=True, found={found}')

        except PartError as e:
            self.get_logger().error(f'[scan] 실패 {e.code}: {e}')
            result.success = False
            result.found = False
            result.error_code = e.code
            goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f'[scan] 예외: {type(e).__name__}: {e}')
            result.success = False
            result.found = False
            result.error_code = DEFAULT_ERROR
            goal_handle.abort()

        return result


def start(work_cls, args=None):
    rclpy.init(args=args)
    node = ScanInterface(work_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
