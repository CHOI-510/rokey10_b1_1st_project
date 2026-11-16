#!/usr/bin/env python3
"""
scan_interface.py — 탐지 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /scan (box_interfaces/action/Scan)을 띄운다.
  - Goal(시작명령)을 받으면 작업 클래스의 run(report)을 호출한다.
  - Goal의 고정 좌표(poses, DB 값)를 작업 클래스의 self.poses에 넣는다.
    Goal에 좌표가 없으면(단독 테스트) common.py 기본값을 쓰고 경고를 남긴다.
  - run() 직전에 기본 TCP·툴을 확인·적용한다 (box_comm/robot.py, stage 'robot_setup').
    실패하면 run()을 부르지 않고 ROBOT_SETUP_FAILED로 끝낸다. 파라미터 robot_setup=auto/on/off
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.
    (어떤 경우에도 Result 없이 끝나지 않는다)

작업 클래스 약속 (box_scan/scan.py에서 작성)
  class ScanWork:
      def __init__(self, node): ...
      # self.poses = {'home_j': [6], 'measure_zone': [x, y, z]}  ← interface가 Goal마다 넣어 줌 (DB 값)
      def run(self, report) -> bool:
          # 단계가 바뀔 때마다 report(stage) 호출 (필수, stage 목록은 common.STAGES['scan'])
          # 남길 일이 있으면 report.log('집기 실패, 재시도 2/3', 'WARN') → 관제가 /event_log로 넘김
          # 물품을 측정존에 두었으면 True, 물품이 없으면 False
          # 실패하면 raise PartError('GRASP_FAILED', '이유')

실행
  box_scan/scan.py의 main()에서 start(ScanWork) 호출
================================================================
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from box_interfaces.action import Scan

from box_comm.robot import RobotSetup
from box_comm.common import Reporter, SETUP_STAGE, ERROR_CODES, PartError, STAGES, default_poses, poses_from_msg  # noqa: F401  (PartError는 scan.py가 여기서 import)

NODE_NAME = 'scan'
ACTION_NAME = '/scan'
DEFAULT_ERROR = 'GRASP_FAILED'   # 예상 못 한 예외일 때 보내는 코드


class ScanInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.robot = RobotSetup(self, NODE_NAME)   # 기본 TCP·툴 확인 (box_comm/robot.py)
        self.work = work_cls(self)
        self.work.poses = default_poses()      # Goal을 받으면 Goal의 좌표(DB 값)로 바뀜
        self._server = ActionServer(self, Scan, ACTION_NAME, self._execute)
        self.get_logger().info(f'[scan] Action 서버 {ACTION_NAME} 준비')

    def _execute(self, goal_handle):
        self.get_logger().info('[scan] Goal 수신: 시작명령')
        result = Scan.Result()

        report = Reporter(
            'scan',
            lambda stage, level, msg: goal_handle.publish_feedback(
                Scan.Feedback(stage=stage, log_level=level, log_msg=msg)),
            self.get_logger())

        try:
            # 고정 좌표: 관제가 Goal에 실어 보낸 DB 값 → 작업 클래스의 self.poses
            poses = poses_from_msg(goal_handle.request.poses)
            if poses is None:
                poses = default_poses()
                self.get_logger().warn('[scan] Goal에 고정 좌표 없음 → common.py 기본값 사용 (단독 테스트용)')
            self.work.poses = poses

            # 기본 TCP·툴 확인·적용 → 실패하면 run()을 부르지 않고 Result로 끝냄
            report(SETUP_STAGE)
            ok, setup_msg = self.robot.ensure_defaults()
            if not ok:
                raise PartError('ROBOT_SETUP_FAILED', setup_msg)
            self.get_logger().info(f'[scan] robot_setup: {setup_msg}')

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
