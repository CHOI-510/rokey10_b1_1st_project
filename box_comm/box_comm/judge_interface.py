#!/usr/bin/env python3
"""
judge_interface.py — 판별 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /judge (box_interfaces/action/Judge)를 띄운다.
  - Goal의 specs(ClassSpec[])를 파이썬 dict 리스트로 바꿔 작업 클래스의 run()에 넘긴다.
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.

작업 클래스 약속 (box_judge/judge.py에서 작성)
  class JudgeWork:
      def __init__(self, node): ...
      def run(self, specs, report) -> str:
          # specs = [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
          #           'height_mm': 40.0, 'weight_g': 100.0}, ...]
          # 판별 결과 'A' / 'B' / 'C' / 'D' 중 하나를 반환
          # 실패하면 raise PartError('MEASURE_FAILED', '이유')

실행
  box_judge/judge.py의 main()에서 start(JudgeWork) 호출
================================================================
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from box_interfaces.action import Judge

NODE_NAME = 'judge'
ACTION_NAME = '/judge'

ERROR_CODES = ('NOT_FOUND', 'GRASP_FAILED', 'OVER_FORCE',
               'MEASURE_FAILED', 'NO_SLOT', 'PLACE_FAILED')
DEFAULT_ERROR = 'MEASURE_FAILED'
VALID_CLASSES = ('A', 'B', 'C', 'D')


class PartError(Exception):
    """작업 실패를 관제에 알릴 때 사용. 등록된 코드만 허용."""

    def __init__(self, code: str, message: str = ''):
        if code not in ERROR_CODES:
            raise ValueError(f'등록되지 않은 error_code: "{code}" (사용 가능: {ERROR_CODES})')
        super().__init__(message or code)
        self.code = code


def _spec_to_dict(s) -> dict:
    return {
        'class_id': s.class_id,
        'width_mm': float(s.width_mm),
        'depth_mm': float(s.depth_mm),
        'height_mm': float(s.height_mm),
        'weight_g': float(s.weight_g),
    }


class JudgeInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.work = work_cls(self)
        self._server = ActionServer(self, Judge, ACTION_NAME, self._execute)
        self.get_logger().info(f'[judge] Action 서버 {ACTION_NAME} 준비')

    def _execute(self, goal_handle):
        specs = [_spec_to_dict(s) for s in goal_handle.request.specs]
        self.get_logger().info(
            f'[judge] Goal 수신: specs {len(specs)}개 '
            f'({", ".join(s["class_id"] for s in specs)})')
        result = Judge.Result()

        def report(stage: str):
            goal_handle.publish_feedback(Judge.Feedback(stage=str(stage)))
            self.get_logger().info(f'[judge] stage={stage}')

        try:
            if not specs:
                raise PartError('MEASURE_FAILED', 'specs가 비어 있음')
            result_class = self.work.run(specs, report)
            if result_class not in VALID_CLASSES:
                raise ValueError(f'run()은 {VALID_CLASSES} 중 하나를 반환해야 함 '
                                 f'(받은 값: {result_class!r})')
            result.success = True
            result.result_class = result_class
            result.error_code = ''
            goal_handle.succeed()
            self.get_logger().info(f'[judge] Result: result_class={result_class}')

        except PartError as e:
            self.get_logger().error(f'[judge] 실패 {e.code}: {e}')
            result.success = False
            result.result_class = ''
            result.error_code = e.code
            goal_handle.abort()

        except Exception as e:
            self.get_logger().error(f'[judge] 예외: {type(e).__name__}: {e}')
            result.success = False
            result.result_class = ''
            result.error_code = DEFAULT_ERROR
            goal_handle.abort()

        return result


def start(work_cls, args=None):
    rclpy.init(args=args)
    node = JudgeInterface(work_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
