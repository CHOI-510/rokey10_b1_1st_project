#!/usr/bin/env python3
"""
judge_interface.py — 판별 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

서버 흐름(Goal 하나만, 좌표 넣기, TCP·툴 확인, 실패 마무리)은 세 파트 공통으로 box_comm/part_server.py에 있다.
여기에는 판별만 다른 것만 있다.
  - Goal 입력: specs(ClassSpec[]) → dict 리스트. 비어 있으면 MEASURE_FAILED
  - 반환값: 'A' / 'B' / 'C' / 'D' / 'U'(미분류) 중 하나. 아니면 MEASURE_FAILED
  - 예상 못 한 예외 → MEASURE_FAILED

알고리즘 약속 (box_judge/judge_algorithm.py, 담당자 소유)
  class JudgeAlgorithm:
      def __init__(self, node, robot): ...     # robot = box_comm.dsr_io.DsrIO
      def judge(self, specs, poses, report) -> str:
          # specs = [{'class_id': 'A', 'width_mm': .., 'depth_mm': .., 'height_mm': .., 'weight_g': ..}, ...]
          # poses = {'home_j': [6], 'measure_zone': [x, y, z]}   (DB 값, Goal마다)
          # 실패하면 raise PartError('MEASURE_FAILED' 또는 'GRASP_FAILED', '이유')

실행
  box_judge/judge_main.py → start_algorithm(JudgeAlgorithm)
================================================================
"""

from box_interfaces.action import Judge

from box_comm import part_server
from box_comm.common import ERROR_CODES, RESULT_CLASSES, STAGES, VALID_CLASSES, PartError  # noqa: F401
from box_comm.convert import spec_to_dict

NODE_NAME = 'judge'
ACTION_NAME = '/judge'
DEFAULT_ERROR = 'MEASURE_FAILED'


def _inputs(req):
    specs = [spec_to_dict(s) for s in req.specs]
    if not specs:
        raise PartError('MEASURE_FAILED', 'specs가 비어 있음')
    return (specs,)


def _fill(result, result_class, _inputs, _log, _report):
    if result_class not in RESULT_CLASSES:
        raise ValueError(f'judge()는 {RESULT_CLASSES} 중 하나를 반환해야 함 (받은 값: {result_class!r})')
    result.result_class = result_class
    return f'result_class={result_class}'


def _fail(result):
    result.result_class = ''


SPEC = part_server.PartSpec(
    name=NODE_NAME, action=Judge, default_error=DEFAULT_ERROR,
    inputs=_inputs,
    describe=lambda inputs: f'specs {len(inputs[0])}개 ({", ".join(s["class_id"] for s in inputs[0])})',
    fill=_fill, fail=_fail)


def start_algorithm(algo_cls, args=None, use_robot=True):
    """판별 알고리즘 실행 (box_judge/judge_main.py, fake·test 노드). 약속은 이 파일 머리말과 box_comm/algorithm.py"""
    part_server.start_algorithm(SPEC, algo_cls, 'judge', 'judge_dsr', use_robot=use_robot, args=args)
