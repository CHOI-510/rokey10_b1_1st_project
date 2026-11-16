#!/usr/bin/env python3
"""
common.py — box_comm 공통 정의
================================================================
관리: 관제(정쓰). 팀원은 수정하지 않는다.

여기에 한 번만 정의하고 모든 *_interface.py와 파트 코드가 import해서 쓴다.
  - ERROR_CODES   : 등록된 error_code 목록 (인터페이스 계약 2.4)
  - JOB_STATUS    : DB 작업 상태 5개 + ERROR_TEXT(에러 코드 → UI 표시 이유) + status_for()
  - VALID_CLASSES : result_class / inventory 키
  - STAGES        : 파트별 Feedback stage 목록
  - ROBOT         : 로봇 ID, 두산 서비스 네임스페이스
  - TCP, TOOL     : 기본 TCP·툴 (각 Goal 시작 전에 box_comm/robot.py가 확인·적용)
  - FIXED_POSES   : 고정 좌표 기본값 (실제 작업은 DB 값을 관제가 Goal로 보내 줌)
  - STACK         : 적재 규칙 (구역당 층 수·단 수) + 수량 ↔ 단·층 변환 함수
  - LOG_*         : 이벤트 로그 (/event_log 토픽) 레벨·코드·QoS
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
    'GRASP_FAILED',         # 집기 실패
    'OVER_FORCE',           # 과도한 힘 감지
    'MEASURE_FAILED',       # 측정 실패
    'NO_SLOT',              # 적재할 칸 없음
    'PLACE_FAILED',         # 적재 실패
    'ROBOT_SETUP_FAILED',   # 기본 TCP·툴 적용 실패 (box_comm이 Goal 시작 전에 보냄)
    'CANCELED',             # 관제가 작업을 취소함 (정지 요청) — box_comm이 report()에서 자동으로 냄
)

VALID_CLASSES = ('A', 'B', 'C', 'D')

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
    'ROBOT_SETUP_FAILED': 'Robot setup failed',
    'CANCELED': 'Canceled',
    'NO_PROGRESS': 'No progress',
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
    if code in ('SERVER_DOWN', 'REJECTED', 'TIMEOUT', 'NO_ERROR_CODE', 'NO_PROGRESS') and part:
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
        'A': None,
        'B': None,
        'C': None,
        'D': None,
    },
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
#   - 적재 실제 코드(box_place/place_main.py)는 시작할 때 자기 설정(max_layers, slot_count)이
#     이 값과 같은지 확인한다. 바꿀 때는 두 곳을 같이 바꾼다.
#   - 적재는 매번 1단부터 내려가며 실제로 확인한다 (재고 숫자로 위치를 정하지 않음).
# ----------------------------------------------------------------------
STACK = {
    'levels': 3,        # 단(자리) 하나에 쌓는 층 수   = place_config.json max_layers
    'max_tiers': 2,     # 구역당 단(자리) 수 (넘으면 NO_SLOT) = place_config.json slot_count
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
    """수량 → 단별 층 수 리스트 (UI 적재 이미지용). 예: 6 → [4, 2], 0 → []"""
    n, lv = max(int(count), 0), STACK['levels']
    return [min(lv, n - i) for i in range(0, n, lv)]


# ----------------------------------------------------------------------
# 정지 기능 (관제 control_interface가 사용)
#   멈추게 하기 : /emergency_stop 요청 또는 이상 감지 → 두산 move_stop + 파트 Goal cancel
#   알아차리기   : 두산 get_robot_state를 주기적으로 확인 (이 버전 컨트롤러는 상태 토픽을 안 냄)
#   관제 파라미터
#     robot_monitor        : 'dsr'(실제·에뮬레이터) / 'fake'(테스트) / 'off'(기본, 감시 안 함)
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
