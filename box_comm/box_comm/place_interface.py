#!/usr/bin/env python3
"""
place_interface.py — 적재 파트 통신 계층
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다. 수정이 필요하면 관제에게 요청.

서버 흐름(Goal 하나만, 좌표 넣기, TCP·툴 확인, 실패 마무리)은 세 파트 공통으로 box_comm/part_server.py에 있다.
여기에는 적재만 다른 것만 있다.
  - Goal 입력: result_class ('A'~'D', 'U'), poses, 현재 관제 재고 스냅샷
  - 반환값: 놓은 위치로 잰 그 클래스 수량 (int). 미분류 U는 0
      구역이 꽉 차서 미분류로 보냈으면 ('U', 그 구역 수량) → Result placed_class='U', count=그 구역 수량 (11/18)
      A~D인데 1 ~ 구역 최대 수량(common.stack_capacity) 밖이면 (11/18)
        → 박스는 이미 놓였으므로 작업을 실패시키지 않는다. count = 0('수량 확인 불가')으로 보내고 WARN 로그
        → 관제는 0을 받으면 재고를 덮어쓰지 않고 '적재 현황 확인 불가'로 기록 (다음 같은 클래스 적재 때 바로잡힘)
      int가 아니면 PLACE_FAILED (코드 오류)
  - 예상 못 한 예외 → PLACE_FAILED

알고리즘 약속 (box_place/place_algorithm.py, 담당자 소유)
  class PlaceAlgorithm:
      def __init__(self, node, robot): ...     # robot = box_comm.dsr_io.DsrIO
      def place(self, result_class, poses, report) -> int:
          # poses = {'home_j': [6], 'measure_zone': [x, y, z], 'place_zones': {'A': [x, y], ...}}  (DB 값)
          # self.inventory = Goal에 실린 현재 관제 재고 (알고리즘 입력, 결과 재고 결정은 관제가 함)
          # 1단부터 내려가며 실제로 놓을 자리를 찾고, 수량 = common.count_from_position(단, 층)
          # 단·층·z·단 간격·박스 높이는 이 안에서만 (관제에는 수량만)
          # 실패하면 raise PartError('NO_SLOT' / 'GRASP_FAILED' / 'PLACE_FAILED', '이유')

실행
  box_place/place_main.py → start_algorithm(PlaceAlgorithm)
================================================================
"""

from box_interfaces.action import Place

from box_comm import part_server
from box_comm.common import (ERROR_CODES, RESULT_CLASSES, STACK, STAGES, UNCLASSIFIED,  # noqa: F401
                             VALID_CLASSES, PartError, count_from_position, stack_capacity)

NODE_NAME = 'place'
ACTION_NAME = '/place'
DEFAULT_ERROR = 'PLACE_FAILED'


def check_count(count, result_class=''):
    """place() 반환값 → (보낼 수량, 경고 문구 또는 ''). int가 아니면 ValueError.
    A~D 범위 밖은 0 + 경고 (박스는 놓였으니 실패로 만들지 않음, 관제가 '확인 불가'로 처리)."""
    if isinstance(count, bool) or not isinstance(count, int):
        raise ValueError(f'place()는 수량(int)을 반환해야 함 (받은 값: {count!r})')
    if result_class == UNCLASSIFIED:
        if count != 0:
            return 0, f'미분류(U)는 0을 반환해야 함 (받은 값: {count}) → 0으로 보냄'
        return 0, ''
    if not 1 <= count <= stack_capacity():
        return 0, f'{result_class} 수량 범위 밖 {count} (1~{stack_capacity()}) → 수량 확인 불가(0)로 보냄'
    return count, ''


def _inputs(req):
    result_class = req.result_class
    if result_class not in RESULT_CLASSES:
        raise PartError('PLACE_FAILED', f'알 수 없는 result_class: "{result_class}"')
    return (result_class,)


def _fill(result, value, inputs, log, report):
    result_class = inputs[0]
    placed_class = result_class
    if isinstance(value, tuple):                    # ('U', 정원): 구역이 꽉 차서 미분류로 보냄 (11/18)
        if len(value) != 2 or value[0] not in (result_class, UNCLASSIFIED):
            raise ValueError(f"place()는 수량 또는 ('U', 수량)을 반환해야 함 (받은 값: {value!r})")
        placed_class, value = value
    count, warn = check_count(value, result_class)
    if warn:
        log.warn(f'[place] {warn}')
        try:
            report.log(warn, 'WARN')
        except Exception:
            pass
    result.count = count
    result.placed_class = placed_class
    if placed_class != result_class:
        return f'{result_class} 구역 만재({count}) → 미분류로 보냄'
    return f'{result_class} 측정 수량 {count}'


def _fail(result):
    result.count = 0
    result.placed_class = ''


SPEC = part_server.PartSpec(
    name=NODE_NAME, action=Place, default_error=DEFAULT_ERROR,
    inputs=_inputs,
    describe=lambda inputs: f'class={inputs[0]}',
    fill=_fill, fail=_fail)


def start_algorithm(algo_cls, args=None, use_robot=True):
    """적재 알고리즘 실행 (box_place/place_main.py, fake·test 노드). 약속은 이 파일 머리말과 box_comm/algorithm.py"""
    part_server.start_algorithm(SPEC, algo_cls, 'place', 'place_dsr', use_robot=use_robot, args=args)
