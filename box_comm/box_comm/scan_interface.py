#!/usr/bin/env python3
"""
scan_interface.py — 탐지 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

서버 흐름(Goal 하나만, 좌표 넣기, TCP·툴 확인, 실패 마무리)은 세 파트 공통으로 box_comm/part_server.py에 있다.
여기에는 탐지만 다른 것만 있다.
  - Goal 입력 없음 (시작 명령 + 고정 좌표)
  - 반환값: 물품을 측정존에 두었으면 True, 물품이 없으면 False (bool이 아니면 SCAN_FAILED)
  - 예상 못 한 예외 → SCAN_FAILED (11/17: GRASP_FAILED는 '재스캔' 신호라 분리)

알고리즘 약속 (box_scan/scan_algorithm.py, 담당자 소유)
  class ScanAlgorithm:
      def __init__(self, node, robot): ...     # robot = box_comm.dsr_io.DsrIO (관제가 만들어 줌)
      def scan(self, poses, report) -> bool:
          # poses = {'home_j': [6], 'measure_zone': [x, y, z]}   (DB 값, Goal마다)
          # 로봇은 robot.movel / movej / grip_open / grip_close / force_z ... 로만 움직인다 (목록: box_comm/dsr_io.py)
          #   → 정지 확인·진행 알림이 자동
          # 단계가 바뀔 때마다 report(stage) (common.STAGES['scan']), 남길 일은 report.log(msg, 'WARN')
          # 집기 최종 실패면 raise PartError('GRASP_FAILED', '이유') → 관제가 공간 스캔부터 다시

실행
  box_scan/scan_main.py → start_algorithm(ScanAlgorithm)
  단독 테스트: ros2 action send_goal /scan box_interfaces/action/Scan "{}" --feedback
================================================================
"""

from box_interfaces.action import Scan

from box_comm import part_server
from box_comm.common import ERROR_CODES, STAGES, PartError  # noqa: F401  (파트 코드가 여기서 import)

NODE_NAME = 'scan'
ACTION_NAME = '/scan'
DEFAULT_ERROR = 'SCAN_FAILED'


def _fill(result, found, _inputs, _log, _report):
    if not isinstance(found, bool):
        raise ValueError(f'scan()은 bool을 반환해야 함 (받은 값: {found!r})')
    result.found = found
    return f'success=True, found={found}'


def _fail(result):
    result.found = False


SPEC = part_server.PartSpec(
    name=NODE_NAME, action=Scan, default_error=DEFAULT_ERROR,
    inputs=lambda _req: (),
    describe=lambda _inputs: '시작명령',
    fill=_fill, fail=_fail)


def start_algorithm(algo_cls, args=None, use_robot=True):
    """탐지 알고리즘 실행 (box_scan/scan_main.py, fake·test 노드). 약속은 이 파일 머리말과 box_comm/algorithm.py"""
    part_server.start_algorithm(SPEC, algo_cls, 'scan', 'scan_dsr', use_robot=use_robot, args=args)
