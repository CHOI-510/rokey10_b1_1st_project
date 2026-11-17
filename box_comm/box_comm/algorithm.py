#!/usr/bin/env python3
"""
algorithm.py — 파트 알고리즘 실행 어댑터 (탐지·판별·적재 공용)
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

파트 알고리즘(담당자 코드)과 관제 연동(정지·실패 처리)을 나누기 위한 연결부.
각 interface의 start_algorithm()이 이 클래스로 알고리즘을 감싼다.

  알고리즘 (담당자, 관제 코드 없음)
    class XxxAlgorithm:
        def __init__(self, node, robot): ...      # robot = box_comm.dsr_io.DsrIO (여기서 만들어 넣어 줌)
        def scan(self, poses, report) -> bool            # 탐지
        def judge(self, specs, poses, report) -> str     # 판별
        def place(self, result_class, poses, report) -> int   # 적재

  여기서 하는 일 (관제)
    - DsrIO를 만들어 알고리즘에 넣는다. 두산 패키지가 없으면 robot=None으로 넣고 경고 (FAKE 전용)
    - Goal마다 robot.bind(report): 로봇 명령 전 정지 확인, force_z()의 keepalive 연결
    - 알고리즘 메서드에 Goal의 고정 좌표(poses, DB 값)를 넘긴다
    - 실패(예외)하면: 로봇이 움직였거나 힘 제어 중이었으면 stop(emergency=True)
                     PartError는 코드 그대로(CANCELED 포함), 그 밖의 예외는 파트 기본 코드로
================================================================
"""

from box_comm.common import PartError


class AlgorithmWork:
    def __init__(self, node, algo_cls, method: str, dsr_name: str, default_error: str):
        self.log = node.get_logger()
        self.method = method
        self.default_error = default_error
        self.robot = None
        try:
            from box_comm.dsr_io import DsrIO            # 두산 워크스페이스가 있을 때만
            self.robot = DsrIO(dsr_name)
        except Exception as e:
            self.log.warn(f'[{method}] 두산 로봇 연결 없이 실행 (robot=None, FAKE 전용): '
                          f'{type(e).__name__}: {e}')
        self.algo = algo_cls(node, self.robot)
        if not callable(getattr(self.algo, method, None)):
            raise TypeError(f'{algo_cls.__name__}에 {method}() 메서드가 없습니다')
        self.poses = None                                 # interface가 Goal마다 넣어 줌

    def run(self, *args):
        """interface가 부르는 형태: run(report) / run(specs, report) / run(result_class, report)
        → 알고리즘 메서드(..., poses, report)"""
        *inputs, report = args
        robot = self.robot
        if robot is not None:
            robot.bind(report)
        try:
            return getattr(self.algo, self.method)(*inputs, self.poses, report)
        except BaseException as exc:
            if robot is not None and robot.needs_stop():     # 움직였거나 힘 제어 중이었으면 멈춤
                try:
                    robot.stop(emergency=True)
                except Exception as stop_exc:
                    code = exc.code if isinstance(exc, PartError) else self.default_error
                    raise PartError(code, f'{exc}; 정지 확인 실패: {stop_exc}') from exc
            if isinstance(exc, (KeyboardInterrupt, SystemExit, PartError)):
                raise
            raise PartError(self.default_error, f'{type(exc).__name__}: {exc}') from exc
        finally:
            if robot is not None:
                robot.unbind()
