#!/usr/bin/env python3
"""Place Action 재사용, /inventory_check 전용. count=0은 정상 빈 자리다."""
from box_interfaces.action import Place
from box_comm import part_server
from box_comm.common import STAGES, VALID_CLASSES, PartError, stack_capacity


def _inputs(req):
    if req.result_class not in VALID_CLASSES:
        raise PartError('PLACE_FAILED', f'점검 분류 오류: {req.result_class}')
    return (req.result_class,)


def _fill(result, value, inputs, log, report):
    if value == ('CALIBRATED', 0):
        result.count, result.placed_class = 0, ''
        return f'{inputs[0]} 빈 바닥 교정 완료 — 재고 갱신 결과가 아님'
    if type(value) is not int or not 0 <= value <= stack_capacity():
        raise ValueError(f'구역 점검 수량은 정수 0~{stack_capacity()} 필요: {value!r}')
    result.count = value
    result.placed_class = inputs[0]
    return f'{inputs[0]} 구역 재고 점검 수량={value}'


def _fail(result):
    result.count, result.placed_class = 0, ''


SPEC = part_server.PartSpec(
    name='inventory_check', action=Place, default_error='PLACE_FAILED',
    inputs=_inputs, describe=lambda inputs: f'class={inputs[0]}, 구역 재고 점검',
    fill=_fill, fail=_fail)


def start_algorithm(algo_cls, args=None):
    if 'inventory_check' not in STAGES:
        raise RuntimeError('common.STAGES에 inventory_check 단계 등록이 필요합니다 (제공 수정본 참고)')
    part_server.start_algorithm(SPEC, algo_cls, 'measure', 'inventory_check_dsr', args=args)
