#!/usr/bin/env python3
"""
common.py — box_comm 공통 정의
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

여기에 한 번만 정의하고 모든 *_interface.py가 import해서 쓴다.
  - ERROR_CODES   : 등록된 error_code 목록 (인터페이스 계약 2.4)
  - VALID_CLASSES : result_class / inventory 키
  - PartError     : 파트가 실패를 알릴 때 raise하는 예외

새 error_code가 필요하면 ERROR_CODES에만 추가한다.
→ 탐지·판별·적재 interface에 동시에 적용된다.

팀원 코드는 지금처럼 자기 interface에서 import하면 된다.
  from box_comm.scan_interface import start, PartError
어느 interface에서 가져오든 같은 PartError 클래스다.
================================================================
"""

ERROR_CODES = (
    'NOT_FOUND',        # 물품을 찾지 못함
    'GRASP_FAILED',     # 집기 실패
    'OVER_FORCE',       # 과도한 힘 감지
    'MEASURE_FAILED',   # 측정 실패
    'NO_SLOT',          # 적재할 칸 없음
    'PLACE_FAILED',     # 적재 실패
)

VALID_CLASSES = ('A', 'B', 'C', 'D')

# ----------------------------------------------------------------------
# Feedback stage 목록 — 각 파트는 단계가 바뀔 때마다 report(stage)를 반드시 호출
# 관제는 마지막 stage와 받은 시각으로 진행 상황을 추적한다.
# 목록에 없는 stage를 보내면 경고 로그가 남는다 (전송은 됨).
# ----------------------------------------------------------------------
STAGES = {
    'scan': (
        'space_scan',            # 작업공간 스캔·물품 감지
        'picking',               # 감지한 물품 집기
        'moving',                # 측정존으로 이동
        'placing',               # 측정존에 내려놓기
    ),
    'judge': (
        'measuring_length1',     # 첫 번째 길이 측정 (물품 방향에 따라 가로/세로/높이가 달라질 수 있어 번호로 구분)
        'measuring_length2',
        'measuring_length3',
        'measuring_weight',      # 무게 측정
        'measuring_materiality', # 강성 추정 (옵션)
        'judging',               # 분류 판별
    ),
    'place': (
        'picking',               # 측정존 물품 집기
        'moving',                # 적재 위치로 이동
        'placing',               # 적재
    ),
}


class PartError(Exception):
    """작업 실패를 관제에 알릴 때 사용. 등록된 코드만 허용.

    사용: raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
    """

    def __init__(self, code: str, message: str = ''):
        if code not in ERROR_CODES:
            raise ValueError(f'등록되지 않은 error_code: "{code}" (사용 가능: {ERROR_CODES})')
        super().__init__(message or code)
        self.code = code
