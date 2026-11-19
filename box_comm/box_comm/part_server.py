#!/usr/bin/env python3
"""
part_server.py — 탐지·판별·적재 Action 서버 공통 틀 (11/18 통합)
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

예전에는 scan_interface / judge_interface / place_interface가 같은 코드를 세 벌 가지고 있었다
(서버 띄우기, Goal 하나만 받기, Reporter, 좌표 넣기, TCP·툴 확인, 실패 마무리, executor).
이제 그 흐름은 여기 한 곳에 있고, 각 interface에는 파트마다 다른 것만 남는다 (PartSpec):
  - Goal → run()에 넘길 입력       (scan: 없음 / judge: specs / place: result_class)
  - run() 반환값 검사 → Result 채우기 (scan: found bool / judge: 결과 클래스 / place: 수량)
  - 실패 Result 기본값, 예상 못 한 예외의 기본 코드

Goal 한 번의 흐름 (세 파트 공통)
  1. 이미 실행 중이면 새 Goal 거절 (한 번에 하나)
  2. 입력 확인 (spec.inputs) → Goal 좌표(DB 값)와 Place 재고 스냅샷을 work에 전달
  3. report('robot_setup') → 기본 TCP·툴 확인 (box_comm/robot.py). 실패면 ROBOT_SETUP_FAILED
  4. work.run(*입력, report) → spec.fill로 반환값 검사·Result 채움 → succeed
  5. 실패: PartError면 그 코드, 그 밖의 예외면 spec.default_error.
     원인을 ERROR 로그로 관제에 보내고(report.log), 관제가 취소했으면 canceled, 아니면 abort
     어떤 경우에도 Result 없이 끝나지 않는다.

실행 방식 (11/18부터 하나)
  start_algorithm(spec, algo_cls, method, dsr_name, use_robot=True)
    알고리즘 약속은 box_comm/algorithm.py. 실제 파트 코드·fake·test 노드 모두 이 방식.
    use_robot=False: 로봇을 쓰지 않는 fake 노드 (DsrIO를 만들지 않음, robot=None)
================================================================
"""

import threading
from typing import Callable, NamedTuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from box_comm.common import SETUP_STAGE, PartError, Reporter, default_poses, poses_from_msg
from box_comm.convert import inv_to_dict
from box_comm.robot import RobotSetup


class PartSpec(NamedTuple):
    name: str                       # 'scan' / 'judge' / 'place' (노드 이름, stage 목록 키)
    action: type                    # box_interfaces.action.Scan ...
    default_error: str              # 예상 못 한 예외일 때 보내는 코드
    inputs: Callable                # inputs(goal_request) -> tuple (run에 넘길 값). 잘못되면 PartError
    describe: Callable              # describe(inputs) -> str  (Goal 수신 로그)
    fill: Callable                  # fill(result, value, inputs, log, report) -> str (Result 채움, 로그 문구). 잘못되면 ValueError
    fail: Callable                  # fail(result) — 실패 Result의 값 칸 기본값


class PartServer(Node):
    def __init__(self, spec: PartSpec, work_factory):
        super().__init__(spec.name)
        self.spec = spec
        self.robot = RobotSetup(self, spec.name)       # 기본 TCP·툴 확인 (box_comm/robot.py)
        self.work = work_factory(self)
        self.work.poses = default_poses()               # Goal을 받으면 Goal의 좌표(DB 값)로 바뀜
        # 정지 기능: 실행 중에도 cancel을 받을 수 있게 Reentrant 그룹 + MultiThreadedExecutor. Goal은 한 번에 하나.
        self._busy = threading.Lock()
        self._server = ActionServer(
            self, spec.action, f'/{spec.name}', self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'[{spec.name}] Action 서버 /{spec.name} 준비')   # fake_run.sh가 이 줄로 준비 확인

    def _on_goal(self, _goal_request):
        if self._busy.locked():
            self.get_logger().warn(f'[{self.spec.name}] 이미 실행 중 → 새 Goal 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        with self._busy:
            return self._execute_locked(goal_handle)

    def _execute_locked(self, goal_handle):
        spec, log = self.spec, self.get_logger()
        name = spec.name
        result = spec.action.Result()
        report = Reporter(
            name,
            lambda stage, level, msg: goal_handle.publish_feedback(
                spec.action.Feedback(stage=stage, log_level=level, log_msg=msg)),
            log,
            is_canceled=lambda: goal_handle.is_cancel_requested)

        try:
            inputs = tuple(spec.inputs(goal_handle.request))
            log.info(f'[{name}] Goal 수신: {spec.describe(inputs)}')

            poses = poses_from_msg(goal_handle.request.poses)
            if poses is None:
                poses = default_poses()
                log.warn(f'[{name}] Goal에 고정 좌표 없음 → common.py 기본값 사용 (단독 테스트용)')
            self.work.poses = poses
            if hasattr(goal_handle.request, 'inventory'):
                self.work.inventory = inv_to_dict(goal_handle.request.inventory)

            report(SETUP_STAGE)
            ok, setup_msg = self.robot.ensure_defaults()
            if not ok:
                raise PartError('ROBOT_SETUP_FAILED', setup_msg)
            log.info(f'[{name}] robot_setup: {setup_msg}')

            value = self.work.run(*inputs, report)
            try:
                done_msg = spec.fill(result, value, inputs, log, report)
            except ValueError as e:
                raise PartError(spec.default_error, f'run() 반환값 오류: {e}')
            result.success = True
            result.error_code = ''
            goal_handle.succeed()
            log.info(f'[{name}] Result: {done_msg}')

        except PartError as e:
            log.error(f'[{name}] 실패 {e.code}: {e}')
            self._failed(goal_handle, result, report, e.code, f'{e.code}: {e}')

        except Exception as e:
            log.error(f'[{name}] 예외: {type(e).__name__}: {e}')
            self._failed(goal_handle, result, report, spec.default_error, f'{type(e).__name__}: {e}')

        return result

    def _failed(self, goal_handle, result, report, code, message):
        try:                                   # 원인을 ERROR 로그로 관제에 → 관제 터미널·DB 로그에 남음
            report.log(message, 'ERROR')
        except Exception:
            pass
        result.success = False
        result.error_code = code
        try:                                   # 실패 Result 값 칸 채우기가 실패해도(예: 메시지 정의가 옛 빌드)
            self.spec.fail(result)             #   success=False·error_code는 관제에 꼭 가야 함 (11/19)
        except Exception as e:
            self.get_logger().error(f'[{self.spec.name}] 실패 Result 채우기 오류 {type(e).__name__}: {e} '
                                    '— box_interfaces를 다시 빌드했는지 확인')
        if goal_handle.is_cancel_requested:    # 관제가 취소(정지)를 요청했으면 canceled
            goal_handle.canceled()
        else:
            goal_handle.abort()


def run_server(spec: PartSpec, work_factory, args=None):
    """전용 MultiThreadedExecutor로 spin (rclpy.spin은 전역 executor를 씀).
    → run() 안에서 DSR_ROBOT2 함수가 응답을 기다려도 'Executor is already spinning' 충돌이 없다."""
    rclpy.init(args=args)
    node = PartServer(spec, work_factory)
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


def start_algorithm(spec: PartSpec, algo_cls, method: str, dsr_name: str, use_robot=True, args=None):
    """알고리즘 클래스를 서버에 연결해 실행 (box_comm/algorithm.py AlgorithmWork)"""
    from box_comm.algorithm import AlgorithmWork
    run_server(spec, lambda node: AlgorithmWork(node, algo_cls, method, dsr_name, spec.default_error,
                                                use_robot=use_robot), args)
