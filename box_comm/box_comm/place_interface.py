#!/usr/bin/env python3
"""
place_interface.py — 적재 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

하는 일
  - Action 서버 /place (box_interfaces/action/Place)를 띄운다.
  - Goal의 result_class를 작업 클래스의 run()에 넘긴다. (v0.4: Goal에 inventory 없음)
  - Goal의 고정 좌표(poses, DB 값 — HOME, 측정존, 적재 구역 1단 x,y)를 작업 클래스의 self.poses에 넣는다.
    Goal에 좌표가 없으면(단독 테스트) common.py 기본값을 쓰고 경고를 남긴다.
  - run() 직전에 기본 TCP·툴을 확인·적용한다 (box_comm/robot.py, stage 'robot_setup').
    실패하면 run()을 부르지 않고 ROBOT_SETUP_FAILED로 끝낸다. 파라미터 robot_setup=auto/on/off
  - run()의 반환값/예외를 Result로 바꿔 관제에 보낸다.
  - 반환값(수량)을 검사한다 (정수, 1 ~ 구역 최대 수량). 어긋나면 PLACE_FAILED.

작업 클래스 약속 (box_place/place.py에서 작성)
  class PlaceWork:
      def __init__(self, node): ...
      # self.poses = {'home_j': [6], 'measure_zone': [x, y, z],
      #               'place_zones': {'A': [x, y], ...}}   ← interface가 Goal마다 넣어 줌 (DB 값)
      def run(self, result_class, report) -> dict:
          # result_class = 'A' / 'B' / 'C' / 'D'
          # 1단부터 내려가며 실제로 놓을 자리를 찾고, 놓은 위치로 잰 수량(int)을 반환
          #   수량 = common.count_from_position(단, 층)   예: 2단 2층 → 6
          # 단·층·z·단 간격·박스 높이는 이 안에서만 처리 (관제에는 수량만 감)
          # 실패하면 raise PartError('PLACE_FAILED' 또는 'NO_SLOT', '이유')
          # 단계가 바뀔 때마다 report(stage) 호출 (필수, stage 목록은 common.STAGES['place'])
          # 남길 일이 있으면 report.log('집기 실패, 재시도 2/3', 'WARN') → 관제가 /event_log로 넘김

실행
  box_place/place.py의 main()에서 start(PlaceWork) 호출
================================================================
"""

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from box_interfaces.action import Place

from box_comm.robot import RobotSetup
from box_comm.common import (Reporter, SETUP_STAGE, ERROR_CODES, PartError, STACK, STAGES,  # noqa: F401
                             count_from_position, default_poses, poses_from_msg,
                             stack_capacity, VALID_CLASSES)

NODE_NAME = 'place'
ACTION_NAME = '/place'
DEFAULT_ERROR = 'PLACE_FAILED'   # 예상 못 한 예외일 때 보내는 코드


def _check_count(count) -> int:
    """run() 반환값(측정 수량) 검사"""
    if isinstance(count, bool) or not isinstance(count, int):
        raise ValueError(f'run()은 수량(int)을 반환해야 함 (받은 값: {count!r})')
    if not 1 <= count <= stack_capacity():
        raise ValueError(f'수량 범위 밖: {count} (1~{stack_capacity()})')
    return count


class PlaceInterface(Node):
    def __init__(self, work_cls):
        super().__init__(NODE_NAME)
        self.robot = RobotSetup(self, NODE_NAME)   # 기본 TCP·툴 확인 (box_comm/robot.py)
        self.work = work_cls(self)
        self.work.poses = default_poses()      # Goal을 받으면 Goal의 좌표(DB 값)로 바뀜
        # 정지 기능: 실행 중에도 cancel을 받을 수 있게 Reentrant 그룹 + MultiThreadedExecutor.
        # 한 번에 Goal 하나만 실행 (실행 중 새 Goal은 거절).
        self._busy = threading.Lock()
        self._server = ActionServer(
            self, Place, ACTION_NAME, self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'[place] Action 서버 {ACTION_NAME} 준비')

    def _on_goal(self, _goal_request):
        if self._busy.locked():
            self.get_logger().warn(f'[{NODE_NAME}] 이미 실행 중 → 새 Goal 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._busy:
            return self._execute_locked(goal_handle)

    def _execute_locked(self, goal_handle):
        req = goal_handle.request
        result_class = req.result_class
        self.get_logger().info(f'[place] Goal 수신: class={result_class}')
        result = Place.Result()

        report = Reporter(
            'place',
            lambda stage, level, msg: goal_handle.publish_feedback(
                Place.Feedback(stage=stage, log_level=level, log_msg=msg)),
            self.get_logger(),
            is_canceled=lambda: goal_handle.is_cancel_requested)

        try:
            if result_class not in VALID_CLASSES:
                raise PartError('PLACE_FAILED', f'알 수 없는 result_class: "{result_class}"')
            # 고정 좌표: 관제가 Goal에 실어 보낸 DB 값 → 작업 클래스의 self.poses
            poses = poses_from_msg(goal_handle.request.poses)
            if poses is None:
                poses = default_poses()
                self.get_logger().warn('[place] Goal에 고정 좌표 없음 → common.py 기본값 사용 (단독 테스트용)')
            self.work.poses = poses

            # 기본 TCP·툴 확인·적용 → 실패하면 run()을 부르지 않고 Result로 끝냄
            report(SETUP_STAGE)
            ok, setup_msg = self.robot.ensure_defaults()
            if not ok:
                raise PartError('ROBOT_SETUP_FAILED', setup_msg)
            self.get_logger().info(f'[place] robot_setup: {setup_msg}')

            try:
                count = _check_count(self.work.run(result_class, report))
            except ValueError as e:
                raise PartError('PLACE_FAILED', f'run() 반환값 오류: {e}')
            result.success = True
            result.count = count
            result.error_code = ''
            goal_handle.succeed()
            self.get_logger().info(f'[place] Result: {result_class} 측정 수량 {count}')

        except PartError as e:
            self.get_logger().error(f'[place] 실패 {e.code}: {e}')
            result.success = False
            result.error_code = e.code
            _finish_failed(goal_handle)

        except Exception as e:
            self.get_logger().error(f'[place] 예외: {type(e).__name__}: {e}')
            result.success = False
            result.error_code = DEFAULT_ERROR
            _finish_failed(goal_handle)

        return result


def _finish_failed(goal_handle):
    """실패로 끝낼 때: 관제가 취소(정지)를 요청한 상태면 canceled, 아니면 abort"""
    if goal_handle.is_cancel_requested:
        goal_handle.canceled()
    else:
        goal_handle.abort()


def start(work_cls, args=None):
    rclpy.init(args=args)
    node = PlaceInterface(work_cls)
    # 전용 executor로 spin한다 (rclpy.spin은 전역 executor를 씀). Multi: run() 실행 중에도 cancel 수신.
    # → run() 안에서 DSR_ROBOT2 함수(movej 등)가 전역 executor로 응답을 기다려도
    #   'Executor is already spinning' 충돌이 나지 않는다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
