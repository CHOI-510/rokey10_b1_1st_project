#!/usr/bin/env python3
"""
common.py — box_comm 공통 정의
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

여기에 한 번만 정의하고 모든 *_interface.py와 파트 코드가 import해서 쓴다.
  - ERROR_CODES   : 등록된 error_code 목록 (인터페이스 계약 2.4)
  - VALID_CLASSES : result_class / inventory 키
  - STAGES        : 파트별 Feedback stage 목록
  - ROBOT         : 로봇 ID, 두산 서비스 네임스페이스
  - TCP, TOOL     : 기본 TCP·툴 (각 Goal 시작 전에 box_comm/robot.py가 확인·적용)
  - FIXED_POSES   : 파트 공통 고정 좌표 (원본. DB는 사본, 관제가 시작 전에 비교)
  - PartError     : 파트가 실패를 알릴 때 raise하는 예외

새 error_code·stage·좌표가 필요하면 관제에게 요청한다.

팀원 코드에서 쓰는 법
  from box_comm.scan_interface import start, PartError
  from box_comm.common import FIXED_POSES
  HOME_J = FIXED_POSES['home_j']
================================================================
"""

ERROR_CODES = (
    'NOT_FOUND',            # 물품을 찾지 못함
    'GRASP_FAILED',         # 집기 실패
    'OVER_FORCE',           # 과도한 힘 감지
    'MEASURE_FAILED',       # 측정 실패
    'NO_SLOT',              # 적재할 칸 없음
    'PLACE_FAILED',         # 적재 실패
    'ROBOT_SETUP_FAILED',   # 기본 TCP·툴 적용 실패 (box_comm이 Goal 시작 전에 보냄)
)

VALID_CLASSES = ('A', 'B', 'C', 'D')

# ----------------------------------------------------------------------
# Feedback stage 목록 — 각 파트는 단계가 바뀔 때마다 report(stage)를 반드시 호출
# 관제는 마지막 stage와 받은 시각으로 진행 상황을 추적한다.
# 목록에 없는 stage를 보내면 경고 로그가 남는다 (전송은 됨).
# ----------------------------------------------------------------------
SETUP_STAGE = 'robot_setup'     # box_comm이 run() 전에 TCP·툴을 확인할 때 보내는 stage

STAGES = {
    'scan': (
        SETUP_STAGE,
        'space_scan',            # 작업공간 스캔·물품 감지
        'picking',               # 감지한 물품 집기
        'moving',                # 측정존으로 이동
        'placing',               # 측정존에 내려놓기
    ),
    'judge': (
        SETUP_STAGE,
        'measuring_length1',     # 첫 번째 길이 측정 (물품 방향에 따라 가로/세로/높이가 달라질 수 있어 번호로 구분)
        'measuring_length2',
        'measuring_length3',
        'measuring_weight',      # 무게 측정
        'measuring_materiality', # 강성 추정 (옵션)
        'judging',               # 분류 판별
    ),
    'place': (
        SETUP_STAGE,
        'picking',               # 측정존 물품 집기
        'moving',                # 적재 위치로 이동
        'placing',               # 적재
    ),
}

# ----------------------------------------------------------------------
# 로봇 설정 — TCP·툴은 로봇 컨트롤러에 하나만 걸리는 전역 설정
# 각 파트 Goal 시작 전에 box_comm/robot.py가 아래 기본값을 확인하고, 다르면 적용한다.
# 파트는 자기 run() 안에서 필요하면 다른 TCP로 바꿔도 된다 (다음 Goal에서 기본값으로 복귀).
# 값 출처: 티치 펜던트 툴 설정 화면 (2026-11-12 확인)
# ----------------------------------------------------------------------
ROBOT = {
    'id': 'dsr01',
    'model': 'm0609',
    'service_ns': '/dsr01/dsr_controller2',
}

TCP = {
    'name': 'GripperDA_v1',
    'pos': [0.0, 0.0, 208.4, 0.0, 0.0, 0.0],      # x, y, z [mm], a, b, c [deg]
}

TOOL = {
    'name': 'Tool Weight',
    'weight': 1.530,                               # [kg]
    'cog': [13.520, 79.740, 0.120],                # 무게중심 Cx, Cy, Cz [mm]
    'inertia': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

# ----------------------------------------------------------------------
# 고정 좌표 — 파트 공통 (원본은 여기, DB에는 사본)
# 관제가 작업 시작 전(preflight)에 DB 사본과 비교하고, 다르면 시작하지 않는다.
# None은 아직 미설정 (비교를 건너뛰고 경고만 남김)
# ----------------------------------------------------------------------
FIXED_POSES = {
    'home_j': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],   # HOME 관절 각도 [deg]
    'measure_zone': None,                          # 측정존 x, y, z [mm, base 기준]  TODO: 실측 후 입력
}

POSE_TOLERANCE = 0.01   # 좌표 비교 허용 오차 (mm 또는 deg)


class PartError(Exception):
    """작업 실패를 관제에 알릴 때 사용. 등록된 코드만 허용.

    사용: raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
    """

    def __init__(self, code: str, message: str = ''):
        if code not in ERROR_CODES:
            raise ValueError(f'등록되지 않은 error_code: "{code}" (사용 가능: {ERROR_CODES})')
        super().__init__(message or code)
        self.code = code
