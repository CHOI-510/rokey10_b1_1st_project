#!/usr/bin/env python3
"""
judge_interface.py — 판별 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /judge (box_interfaces/action/Judge)를 띄운다.
  - Goal의 specs(ClassSpec[])를 파이썬 dict 리스트로 바꿔 작업 클래스의 run()에 넘긴다.
  - Goal의 고정 좌표(poses, DB 값)를 작업 클래스의 self.poses에 넣는다.
    Goal에 좌표가 없으면(단독 테스트) common.py 기본값을 쓰고 경고를 남긴다.
  - run() 직전에 기본 TCP·툴을 확인·적용한다 (box_comm/robot.py, stage 'robot_setup').
    실패하면 run()을 부르지 않고 ROBOT_SETUP_FAILED로 끝낸다. 파라미터 robot_setup=auto/on/off
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.

작업 클래스 약속 (box_judge/judge.py에서 작성)
  class JudgeWork:
      def __init__(self, node): ...
      # self.poses = {'home_j': [6], 'measure_zone': [x, y, z]}  ← interface가 Goal마다 넣어 줌 (DB 값)
      def run(self, specs, report) -> str:
          # specs = [{'class_id': 'A', 'width_mm': 40.0, 'depth_mm': 40.0,
          #           'height_mm': 40.0, 'weight_g': 100.0}, ...]
          # 판별 결과 'A' / 'B' / 'C' / 'D' 중 하나를 반환
          # 실패하면 raise PartError('MEASURE_FAILED', '이유')
          # 단계가 바뀔 때마다 report(stage) 호출 (필수, stage 목록은 common.STAGES['judge'])

실행
  box_judge/judge.py의 main()에서 start(JudgeWork) 호출
================================================================
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from box_interfaces.action import Judge

from box_comm.robot import RobotSetup
from box_comm.common import SETUP_STAGE, ERROR_CODES, PartError, STAGES, default_poses, poses_from_msg, VALID_CLASSES  # noqa: F401

NODE_NAME = 'judge'
ACTION_NAME = '/judge'
DEFAULT_ERROR = 'MEASURE_FAILED'   # 예상 못 한 예외일 때 보내는 코드


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
        self.robot = RobotSetup(self, NODE_NAME)   # 기본 TCP·툴 확인 (box_comm/robot.py)
        self.work = work_cls(self)
        self.work.poses = default_poses()      # Goal을 받으면 Goal의 좌표(DB 값)로 바뀜
        self._server = ActionServer(self, Judge, ACTION_NAME, self._execute)
        self.get_logger().info(f'[judge] Action 서버 {ACTION_NAME} 준비')

    def _execute(self, goal_handle):
        specs = [_spec_to_dict(s) for s in goal_handle.request.specs]
        self.get_logger().info(
            f'[judge] Goal 수신: specs {len(specs)}개 '
            f'({", ".join(s["class_id"] for s in specs)})')
        result = Judge.Result()

        def report(stage: str):
            stage = str(stage)
            if stage not in STAGES['judge']:
                self.get_logger().warn(
                    f'[judge] 등록되지 않은 stage: "{stage}" (사용 가능: {STAGES["judge"]})')
            goal_handle.publish_feedback(Judge.Feedback(stage=stage))
            self.get_logger().info(f'[judge] stage={stage}')

        try:
            if not specs:
                raise PartError('MEASURE_FAILED', 'specs가 비어 있음')
            # 고정 좌표: 관제가 Goal에 실어 보낸 DB 값 → 작업 클래스의 self.poses
            poses = poses_from_msg(goal_handle.request.poses)
            if poses is None:
                poses = default_poses()
                self.get_logger().warn('[judge] Goal에 고정 좌표 없음 → common.py 기본값 사용 (단독 테스트용)')
            self.work.poses = poses

            # 기본 TCP·툴 확인·적용 → 실패하면 run()을 부르지 않고 Result로 끝냄
            report(SETUP_STAGE)
            ok, setup_msg = self.robot.ensure_defaults()
            if not ok:
                raise PartError('ROBOT_SETUP_FAILED', setup_msg)
            self.get_logger().info(f'[judge] robot_setup: {setup_msg}')

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
