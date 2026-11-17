#!/usr/bin/env python3
"""
common.py — box_comm 공통 정의
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

여기에 한 번만 정의하고 모든 *_interface.py와 파트 코드가 import해서 쓴다.
  - ERROR_CODES   : 등록된 error_code 목록 (인터페이스 계약 2.4)
  - JOB_STATUS    : DB 작업 상태 5개 + ERROR_TEXT(에러 코드 → UI 표시 이유) + status_for()
  - VALID_CLASSES : 정상 분류 A~D (분류 기준·적재 구역·적재 수량 측정 대상)
  - UNCLASSIFIED  : 미분류 'U' (v0.5) + RESULT_CLASSES(판별 결과·적재 Goal) / INVENTORY_KEYS(재고 키)
  - STAGES        : 파트별 Feedback stage 목록
  - ROBOT         : 로봇 ID, 두산 서비스 네임스페이스
  - TCP, TOOL     : 기본 TCP·툴 (각 Goal 시작 전에 box_comm/robot.py가 확인·적용)
  - FIXED_POSES   : 고정 좌표 기본값 (실제 작업은 DB 값을 관제가 Goal로 보내 줌)
  - STACK         : 적재 규칙 (구역당 층 수·단 수) + 수량 ↔ 단·층 변환 함수
  - LOG_*         : 이벤트 로그 (/event_log 토픽) 레벨·코드·QoS
  - GRIPPER       : RG2 그리퍼 Modbus 설정 (세 파트 공통, 11/17) + GRIP_RETRY(집기 재시도 횟수)
  - RECOVERY      : 집기 실패 복구 (측정존 비우기 경로·확인) + RECOVER_LIMIT(관제 연속 재스캔 한도)
  - SAFETY        : 정지 기능 설정 (로봇 상태 감시 주기, 정지 모드, 진행 감시) + ROBOT_STATE
  - Reporter      : 파트 run()에 넘기는 report — report(stage), report.log(msg), report.check()
  - PartError     : 파트가 실패를 알릴 때 raise하는 예외

새 error_code·stage·좌표가 필요하면 관제에게 요청한다.

팀원 코드에서 쓰는 법
  from box_comm.scan_interface import start, PartError
  고정 좌표는 run() 안에서 self.poses로 읽는다 (interface가 Goal마다 넣어 줌)
    self.poses['home_j'], self.poses['measure_zone'], self.poses['place_zones']['A']
================================================================
"""

ERROR_CODES = (
    'NOT_FOUND',            # 물품을 찾지 못함
    'GRASP_FAILED',         # 집기 실패 (파트가 GRIP_RETRY번 다시 집어 보고 포기) → 관제가 공간 스캔부터 다시
    'OVER_FORCE',           # 과도한 힘 감지
    'MEASURE_FAILED',       # 측정 실패
    'NO_SLOT',              # 적재할 칸 없음
    'SCAN_FAILED',          # 탐지 실패 (집기 말고 예상 못 한 오류 — 탐지 기본 코드, 11/17)
    'PLACE_FAILED',         # 적재 실패
    'ROBOT_SETUP_FAILED',   # 기본 TCP·툴 적용 실패 (box_comm이 Goal 시작 전에 보냄)
    'CANCELED',             # 관제가 작업을 취소함 (정지 요청) — box_comm이 report()에서 자동으로 냄
)

# 집기 실패 처리 (11/17)
#   파트: 정해진 방식으로 GRIP_RETRY번까지 다시 집기 (시도마다 report.log(..., 'WARN'))
#         → 그래도 못 집으면 raise PartError('GRASP_FAILED')
#         (파트 Goal은 여기서 끝. 그리퍼는 열고 안전 높이로 올라간 상태로 끝낸다)
#   관제: GRASP_FAILED를 받으면 작업을 끝내지 않음
#         탐지 → 공간 스캔부터 다시 / 판별·적재 → 측정존 비우기(관제가 로봇을 움직임,
#         box_control/recovery.py) → 공간 스캔부터 다시. 재고·박스 번호 그대로
#         연속 RECOVER_LIMIT번을 넘으면 RETRY_LIMIT, 측정존을 못 비우면 RECOVERY_FAILED → 작업 종료(ERROR)

VALID_CLASSES = ('A', 'B', 'C', 'D')     # 정상 분류: 분류 기준(specs)·적재 구역(place_zones)·적재 수량 측정 대상

# ----------------------------------------------------------------------
# 미분류 (v0.5) — 판별이 A~D 어디에도 맞지 않는다고 판단한 물품
#   - 판별은 'U'를 반환할 수 있다. 관제는 적재에 'U'로 Goal을 보낸다.
#   - 적재는 미분류 투하 위치(place_config.json의 unclassified)에 놓기만 하고 수량을 재지 않는다.
#     (쌓지 않으므로 높이로 셀 필요가 없음) → Place Result count = 0
#   - 미분류 수량은 관제가 센다: 적재 성공 시 재고 U = 이전 + 1 → DB UpdateInventory
#   - 미분류 위치는 DB 고정 좌표가 아니라 적재 설정에 있다 (투하 자세까지 포함한 한 점이라서)
# ----------------------------------------------------------------------
UNCLASSIFIED = 'U'
RESULT_CLASSES = VALID_CLASSES + (UNCLASSIFIED,)   # 판별 결과 / 적재 Goal에 허용
INVENTORY_KEYS = RESULT_CLASSES                    # 재고(Inventory) 키: A, B, C, D, U

# ----------------------------------------------------------------------
# 작업 상태 (DB SetJobStatus) — UI 표시: f"{status} : {reason}"
#   ERROR   : 공정이 실패해서 관제가 판단해 멈춤. 로봇은 정상 → 로그 보고 원인 해결 후 재시작
#   STOPPED : 로봇이 물리적으로 멈춤 (비상정지 버튼, 두산 안전 정지 등) → 현장 확인 후 복구
# ----------------------------------------------------------------------
JOB_STATUS = ('BEFORE', 'RUNNING', 'DONE', 'ERROR', 'STOPPED')

# STOPPED가 되는 코드 (감지·기록 경로는 정지 기능 단계에서 추가 — 지금은 정의만)
STOP_CODES = (
    'EMERGENCY_STOP',         # UI 비상정지 버튼
    'ROBOT_SAFETY_STOP',      # 두산 안전 정지 (충돌 감지, 토크 초과, 보호 정지) — robot_state SAFE_STOP 등
    'ROBOT_EMERGENCY_STOP',   # 티치 펜던트·하드웨어 비상정지 — robot_state EMERGENCY_STOP
    'ROBOT_DISCONNECTED',     # 로봇 연결 끊김 (robot_disconnection 토픽, 상태 확인 연속 실패)
    'ROBOT_NOT_READY',        # 작업 중 로봇이 STANDBY/MOVING이 아닌 상태 (수동 모드, 복구 중 등)
)

# 에러 코드 → UI에 보일 이유 (영어, 짧게). 없는 코드는 코드 이름 그대로 표시
ERROR_TEXT = {
    # 파트 Result error_code (ERROR_CODES)
    'NOT_FOUND': 'Item not found',
    'GRASP_FAILED': 'Fail to pick',
    'OVER_FORCE': 'Over force detected',
    'MEASURE_FAILED': 'Fail to measure',
    'NO_SLOT': 'Zone full',
    'PLACE_FAILED': 'Fail to place',
    'SCAN_FAILED': 'Fail to scan',
    'ROBOT_SETUP_FAILED': 'Robot setup failed',
    'CANCELED': 'Canceled',
    'NO_PROGRESS': 'No progress',
    # 관제 복구 (box_control/recovery.py, 11/17)
    'RECOVERY_FAILED': 'Measure zone not cleared',   # 집기 실패 뒤 측정존을 비우지 못함 → 측정존 사용 불가
    'RETRY_LIMIT': 'Too many pick failures',         # 집기 실패 재스캔 연속 한도 초과
    # 관제 쪽 통신 오류 (control_interface)
    'SERVER_DOWN': 'No response',
    'REJECTED': 'Goal rejected',
    'TIMEOUT': 'Timeout',
    'NO_ERROR_CODE': 'Unknown failure',
    'DB_FAILED': 'DB failed',
    # STOPPED (STOP_CODES)
    'EMERGENCY_STOP': 'EMERGENCY STOP',
    'ROBOT_SAFETY_STOP': 'Robot safety stop',
    'ROBOT_EMERGENCY_STOP': 'Pendant emergency stop',
    'ROBOT_DISCONNECTED': 'Robot disconnected',
    'ROBOT_NOT_READY': 'Robot not ready',
}


def status_for(code: str) -> str:
    """중단 원인 코드 → DB status ('STOPPED' 또는 'ERROR')"""
    return 'STOPPED' if code in STOP_CODES else 'ERROR'


def error_text(code: str, part: str = '', class_id: str = '') -> str:
    """중단 원인 코드 → UI 표시 이유. part·class를 알면 덧붙인다.
    예: ('SERVER_DOWN', 'place') → 'No response (place)', ('NO_SLOT', '', 'B') → 'Zone full (B)'"""
    text = ERROR_TEXT.get(code, code or 'Unknown failure')
    if code == 'NO_SLOT' and class_id:
        return f'{text} ({class_id})'
    if code in ('SERVER_DOWN', 'REJECTED', 'TIMEOUT', 'NO_ERROR_CODE', 'NO_PROGRESS',
                'GRASP_FAILED') and part:
        return f'{text} ({part})'
    return text

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
# 고정 좌표 — 기준은 DB (인터페이스 v0.3, 적재 구역은 v0.4)
#   관제가 GetJobData로 DB 값을 받아 자기 값을 무조건 덮어쓰고, 모든 Action Goal에 실어 보낸다.
#   파트 interface가 Goal의 값으로 작업 클래스의 self.poses를 덮어쓴다 (Goal마다).
# 아래 FIXED_POSES는 '기본값'이다. 다르다고 멈추거나 경고하지 않는다.
#   - Goal에 좌표가 없을 때(팀원 단독 send_goal 테스트 등)만 쓰인다 (경고 로그).
# None은 아직 미설정
#
# place_zones: 클래스별 적재 구역의 '1단(첫 자리)' x, y [mm, base 기준]
#   다음 단(자리)의 위치는 적재 설정이 정한다 (box_place/config/place_config.json의 slot_dx_mm,
#   현재 −x 방향 80mm, y는 같음). 단 간격·z·박스 높이는 적재 파트만 가진다.
# ----------------------------------------------------------------------
FIXED_POSES = {
    'home_j': [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],   # HOME 관절 각도 [deg]
    'measure_zone': [300.0, -160.0, 30.0],         # 측정존 x, y, z [mm, base 기준]  z = 박스 파지 TCP 높이 (적재 1층 첫 접촉 Z ≈ 30)
    'place_zones': {                               # 적재 구역 1단 x, y [mm, base 기준]  TODO: 실측 후 입력
        'A': [480.0, -160.0],   # 실측 (11/16)
        'B': [650.0, -160.0],   # 실측 (11/16)
        'C': [580.0, 300.0],    # 실측 (11/16)
        'D': None,              # 미정 (D 수치 대기)
    },
    # 미분류(U) 투하 위치는 여기(DB 고정 좌표)가 아니라 box_place/config/place_config.json의 unclassified
    #   release_pose [670, -43, 144, 160, 180, 162], approach_z 160 (11/16)
}

POSE_TOLERANCE = 0.01   # 좌표 비교 허용 오차 (mm 또는 deg)


def default_poses() -> dict:
    """FIXED_POSES 기본값의 복사본
    {'home_j': [6], 'measure_zone': [3] 또는 None, 'place_zones': {'A': [x, y] 또는 None, ...}}"""
    zone = FIXED_POSES['measure_zone']
    return {'home_j': list(FIXED_POSES['home_j']),
            'measure_zone': list(zone) if zone is not None else None,
            'place_zones': {c: (list(v) if v is not None else None)
                            for c, v in FIXED_POSES['place_zones'].items()}}


def poses_from_msg(msg):
    """FixedPoses 메시지 → dict. available=False면 None.
    place_zones에 없는 클래스는 None (적재 파트가 그 클래스를 받으면 PLACE_FAILED)."""
    if not getattr(msg, 'available', False):
        return None
    zones = {c: None for c in VALID_CLASSES}
    for z in getattr(msg, 'place_zones', []):
        if z.class_id in zones:
            zones[z.class_id] = [float(z.x), float(z.y)]
    return {'home_j': [float(v) for v in msg.home_j],
            'measure_zone': [float(v) for v in msg.measure_zone],
            'place_zones': zones}


# ----------------------------------------------------------------------
# 적재 규칙 (v0.4) — 관제·DB·UI·적재가 모두 같은 규칙으로 수량 ↔ 위치를 해석한다
#   - 구역 하나에 같은 클래스만 쌓는다.
#   - 1단(첫 자리)에 1층부터 levels층까지 쌓고, 다 차면 다음 자리(2단)에 다시 1층부터.
#     다음 자리 방향·간격은 적재 설정(slot_dx_mm)이 정한다.
#   - 수량 n → n번째 박스 위치 = (n-1)//levels + 1 단, (n-1)%levels + 1 층
#     예: 5개 → 1단 3개 + 2단 2개
#   - 적재 실제 코드(box_place/place_algorithm.py)는 이 값을 그대로 쓴다 (적재 설정에는 적지 않음).
#     바꾸면 적재의 자리별 첫 접촉 Z(place_config.json)도 다시 재야 한다 → 적재 담당과 합의 후 변경.
#   - 적재는 매번 1단부터 내려가며 실제로 확인한다 (재고 숫자로 위치를 정하지 않음).
# ----------------------------------------------------------------------
STACK = {
    'levels': 3,        # 단(자리) 하나에 쌓는 층 수
    'max_tiers': 2,     # 구역당 단(자리) 수 (넘으면 NO_SLOT)
}


def stack_capacity() -> int:
    """구역 하나에 쌓을 수 있는 최대 수량"""
    return STACK['levels'] * STACK['max_tiers']


def count_from_position(tier: int, level: int) -> int:
    """박스를 놓은 (단, 층) → 그 구역의 수량. 앞 단은 모두 찼다고 본다."""
    return (int(tier) - 1) * STACK['levels'] + int(level)


def position_from_count(count: int):
    """수량 n → n번째(마지막) 박스의 (단, 층). n=0이면 (0, 0)."""
    n = int(count)
    if n <= 0:
        return 0, 0
    return (n - 1) // STACK['levels'] + 1, (n - 1) % STACK['levels'] + 1


def stack_layout(count: int) -> list:
    """수량 → 단별 층 수 리스트 (UI 적재 이미지용). 예: 5 → [3, 2], 0 → []"""
    n, lv = max(int(count), 0), STACK['levels']
    return [min(lv, n - i) for i in range(0, n, lv)]


# ----------------------------------------------------------------------
# 그리퍼 (RG2, Modbus TCP) — 11/17 세 파트 공통으로 디지털 출력(DO) → Modbus
#   box_comm/gripper.py가 이 값으로 접속하고, robot(DsrIO)이 Goal 동안만 접속을 연다 (동시 접속 1개).
#   OnRobot 드라이버(m0609_rg2_bringup mode:=real)가 떠 있으면 접속 실패 → pkill -f OnRobotRGControllerServer
#   환경변수 BOX_GRIPPER=fake → 가짜 그리퍼 (에뮬레이터). 기본 modbus.
#   보정값은 판별 실측 (11/17, 패드 장착, 젠가 29·75mm): 실제 폭 = 원시 폭 − 10.30
#   박스별로 다른 여는 폭·잡는 힘은 각 파트 설정에서 정해 robot.grip_open/grip_close에 넘긴다.
# ----------------------------------------------------------------------
GRIPPER = {
    'ip': '192.168.1.1',
    'port': 502,
    'slave': 65,                 # Compute Box 장치 주소
    'open_width_mm': 100.0,      # 기본 여는 패드 간격 (grip(opened=True))
    'open_force_n': 20.0,        # 여는 힘
    'grip_force_n': 20.0,        # 기본 잡는 힘 (grip(opened=False)). RG2 최대 40N
    'empty_raw_mm': 1.0,         # 닫은 뒤 원시 폭이 이 이하면 빈손
    'cal_gain': 1.0,             # 폭 보정 기울기
    'cal_offset_mm': 0.30,       # 폭 보정 오프셋
    'timeout_s': 5.0,            # 닫기 대기 최대 시간
}
GRIP_RETRY = 3          # 파트가 못 집었을 때 다시 집는 최대 횟수 (첫 시도 포함)

# ----------------------------------------------------------------------
# 집기 실패 복구 — 측정존 비우기 (관제 box_control/recovery.py, 판별·적재 GRASP_FAILED 뒤 관제가 실행)
#   1. 그리퍼 열고 안전 높이로
#   2. push_path를 차례로 movel (그리퍼 닫고 박스를 스캔 공간으로 밀어냄)
#   3. 측정존 확인: 측정존 위에서 그리퍼를 열고 파지 높이까지 내려가 닫아 봄 → 잡히면 아직 박스가 있음
#   push_path가 None(미정)이면 밀지 않고 RECOVERY_FAILED → 작업 종료 (사람이 측정존을 비움)
# ----------------------------------------------------------------------
RECOVERY = {
    'push_path': None,           # [[x, y, z, a, b, c], ...] [mm, deg, base] — 판별·적재와 실물 보고 정할 것
    'safe_z': 160.0,             # 옆으로 움직이기 전 TCP 높이 [mm]
    'speed': 60.0,               # 직선 이동 속도 [mm/s]
    'acc': 100.0,
    'probe_orientation': None,   # 측정존 확인 때 그리퍼 자세 [a, b, c] (None이면 지금 자세 유지)
}
RECOVER_LIMIT = 3       # 관제: 집기 실패 → 재스캔을 연속 몇 번까지 허용 (넘으면 RETRY_LIMIT로 종료)

# ----------------------------------------------------------------------
# 정지 기능 (관제 control_interface가 사용)
#   멈추게 하기 : /emergency_stop 요청 또는 이상 감지 → 두산 move_stop + 파트 Goal cancel
#   알아차리기   : 두산 get_robot_state를 주기적으로 확인 (이 버전 컨트롤러는 상태 토픽을 안 냄)
#   관제 파라미터
#     robot_monitor        : 'dsr'(기본, 실제·에뮬레이터) / 'fake'(테스트) / 'off'(로봇 없이 테스트할 때만)
#       fake 모드에서는 'ros2 param set /control fake_robot_state 5' 로 상태를 바꿔 시험한다
#     progress_timeout_sec : 파트 Feedback이 이 시간 동안 없으면 정지 (0 = 끔)
# ----------------------------------------------------------------------
SAFETY = {
    'monitor_period_sec': 0.5,     # 로봇 상태 확인 주기
    'lost_limit': 6,               # 상태 확인 연속 실패 횟수 → ROBOT_DISCONNECTED (0.5초 x 6 = 3초)
    'stop_mode': 1,                # move_stop 모드: 0 빠른 정지(STO 없음) / 1 빠른 정지 / 2 부드러운 정지 / 3 HOLD
    'stop_hold_sec': 3.0,          # 정지 후 이 시간 동안 move_stop을 반복 (파트가 다음 동작을 보내도 막음)
    'stop_resend_sec': 0.2,        # 위 반복 간격
    'srv_robot_state': 'system/get_robot_state',   # ROBOT['service_ns'] 아래
    'srv_move_stop': 'motion/move_stop',
    'topic_error': '/dsr01/error',                  # 두산 알람 (RobotError) — 이름은 'ros2 topic list'로 확인
    'topic_disconnect': '/dsr01/robot_disconnection',
}

# 두산 GetRobotState.robot_state 값 (dsr_msgs2/srv/GetRobotState.srv)
ROBOT_STATE = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
    5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
    9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY',
}
ROBOT_OK_STATES = (1, 2)            # 작업 가능: STANDBY, MOVING


def robot_stop_code(state: int):
    """로봇 상태 → 중단 코드. 정상(STANDBY/MOVING)이면 None"""
    if state in ROBOT_OK_STATES:
        return None
    if state == 6:
        return 'ROBOT_EMERGENCY_STOP'
    if state in (3, 5, 9, 10):
        return 'ROBOT_SAFETY_STOP'
    return 'ROBOT_NOT_READY'


# ----------------------------------------------------------------------
# 이벤트 로그 (/event_log 토픽, box_interfaces/msg/LogEvent)
#   보내는 쪽은 관제 하나. 파트는 report.log()로 Feedback에 실어 보내고 관제가 넘긴다.
#   받는 쪽은 DB(저장 + UI). 보내기만 하고 응답을 기다리지 않는다 (누락 가능, 작업은 안 막힘).
#   확인: ros2 topic echo /event_log    기록: ros2 bag record /event_log
# ----------------------------------------------------------------------
LOG_TOPIC = '/event_log'
LOG_LEVELS = ('INFO', 'WARN', 'ERROR')
LOG_CODES = (
    'PART_LOG',         # 파트가 report.log()로 보낸 로그 (재시도, 정렬, 스캔 결과 등)
    'JOB_START',        # 작업 시작 (RUNNING)
    'JOB_DONE',         # 작업 완료 (요약 포함)
    'JOB_CANCELED',     # 시작 전 취소 (데이터 실패, 사전 점검 실패, q)
    'JOB_ERROR',        # 공정 오류로 중단 (status ERROR)
    'JOB_STOPPED',      # 로봇 정지로 중단 (status STOPPED)
    'BOX_DONE',         # 박스 하나 적재·재고 반영 완료
    'COUNT_MISMATCH',   # 재고 기대값과 적재 측정값이 다름 (측정값으로 반영)
    'BOX_RETRY',        # 집기 실패 → (측정존 비움) → 공간 스캔부터 다시
    'ZONE_CLEAR',       # 관제가 측정존 비우기 시작·결과 (box_control/recovery.py)
    'STOP_REQUEST',     # 정지 요청 (비상정지 버튼, 로봇 이상 감지, 진행 없음)
    'ROBOT_ALARM',      # 두산 알람 (error 토픽)
)
LOG_QOS_DEPTH = 100     # reliable + transient_local: DB가 조금 늦게 켜져도 최근 100개는 받음


def log_qos():
    """/event_log용 QoS (발행·구독 양쪽이 같은 값을 써야 transient_local이 동작)"""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=LOG_QOS_DEPTH, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Reporter:
    """파트 run()에 넘기는 report.

    report('picking')                              진행 단계 알림 (필수) → 로그 아님
    report.log('집기 실패, 재시도 2/3', 'WARN')     지금 단계 그대로 + 이벤트 로그로 올라감
    report.check()                                 관제가 정지를 요청했으면 여기서 멈춤 (로봇 명령 전에)
    report.keepalive()                             오래 걸리는 단계 중 '살아 있음' 알림 (check 포함, 로그 없음)
                                                   → 관제 진행 감시(progress_timeout_sec)가 멈춘 것으로 오인하지 않게

    정지: 관제가 Goal을 cancel하면 다음 report(stage) 또는 report.check()에서
    PartError('CANCELED')가 나서 run()을 빠져나온다. 잡지(except) 말고 그대로 두면 된다.
    """

    def __init__(self, part, publish, logger, is_canceled=None):
        self._part = part                # 'scan' / 'judge' / 'place'
        self._publish = publish          # publish(stage, log_level, log_msg)
        self._log = logger
        self._is_canceled = is_canceled or (lambda: False)
        self.stage = ''

    def check(self):
        """관제가 취소(정지)를 요청했으면 PartError('CANCELED')"""
        if self._is_canceled():
            self._log.warn(f'[{self._part}] 관제가 정지를 요청함 → 작업 중단 (stage={self.stage})')
            raise PartError('CANCELED', f'관제 정지 요청 (stage={self.stage})')

    def keepalive(self):
        """지금 단계 그대로 Feedback만 다시 보냄 (로그 안 남김). 1초에 한 번 정도면 충분"""
        self.check()
        self._publish(self.stage, '', '')

    def __call__(self, stage: str):
        self.check()
        stage = str(stage)
        if stage not in STAGES[self._part]:
            self._log.warn(f'[{self._part}] 등록되지 않은 stage: "{stage}" '
                           f'(사용 가능: {STAGES[self._part]})')
        self.stage = stage
        self._publish(stage, '', '')
        self._log.info(f'[{self._part}] stage={stage}')

    def log(self, message: str, level: str = 'INFO'):
        level = str(level).upper()
        if level not in LOG_LEVELS:
            self._log.warn(f'[{self._part}] 알 수 없는 log level "{level}" → INFO')
            level = 'INFO'
        self._publish(self.stage, level, str(message))
        self._log.info(f'[{self._part}] log {level}: {message}')


class PartError(Exception):
    """작업 실패를 관제에 알릴 때 사용. 등록된 코드만 허용.

    사용: raise PartError('GRASP_FAILED', '두 번 시도했지만 못 집음')
    """

    def __init__(self, code: str, message: str = ''):
        if code not in ERROR_CODES:
            raise ValueError(f'등록되지 않은 error_code: "{code}" (사용 가능: {ERROR_CODES})')
        super().__init__(message or code)
        self.code = code
