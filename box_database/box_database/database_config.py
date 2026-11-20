#!/usr/bin/env python3
"""
database_config.py — DB 노드 설정 (관제·DB: 정쓰)
================================================================
환경 변수 (터미널에서 export, 코드·git에 키를 넣지 않는다)
  SUPABASE_URL           https://xxxx.supabase.co
  SUPABASE_SERVICE_KEY   service_role 키 (DB 노드 전용. 웹 코드에 절대 넣지 말 것)
  BOX_LOG_DIR            이벤트 로그 파일 폴더 (기본 ~/box_logs)
  BOX_DB_PORT            버튼 창구 포트 (기본 8765, 127.0.0.1에서만 받음)
  BOX_START_TOKEN        설정하면 /start·/cancel·/inventory에 헤더 X-Box-Token이 같아야 받음 (기본: 안 씀)

ROS 파라미터 (ros2 run box_database database_main --ros-args -p store:=file)
  store     'auto'(기본: 키가 있으면 supabase, 없으면 file) / 'supabase' / 'file'
  fake_dir  파일 모드 폴더 (기본 ~/box_logs/fake_db). 테이블마다 JSON 파일 하나:
            product.json, loc.json (처음엔 아래 FILE_* 값으로 만들어짐, 직접 고쳐도 됨),
            state.json (DB 노드가 씀), state_updates.jsonl (state에 쓴 기록)
            → 노드를 껐다 켜도 재고가 남음. 폴더를 지우면 초기값으로 시작.
================================================================
"""
import os

# ---- 연결·경로 ----
SUPABASE_URL = os.environ.get('SUPABASE_URL', '')
SUPABASE_KEY = os.environ.get('SUPABASE_SERVICE_KEY', '')
LOG_DIR = os.path.expanduser(os.environ.get('BOX_LOG_DIR', '~/box_logs'))
HTTP_HOST = '127.0.0.1'                     # 노트북 안에서만 받음 (Next.js가 같은 노트북에서 넘김)
HTTP_PORT = int(os.environ.get('BOX_DB_PORT', '8765'))
START_TOKEN = os.environ.get('BOX_START_TOKEN', '')
DEFAULT_FAKE_DIR = os.path.join(LOG_DIR, 'fake_db')

# ---- 주기·시간 ----
DB_TIMEOUT_SEC = 3          # Supabase 호출 한 번 최대 대기 (관제는 응답을 기다리므로 짧게)
SYNC_PERIOD_SEC = 0.5       # 동기화 스레드: 모아 둔 관제 상태를 쓰는 주기
HEARTBEAT_SEC = 2.0         # heartbeat_at 갱신 주기 → UI: 5초 넘게 안 바뀌면 'DB 노드 꺼짐'
SEEN_WRITE_SEC = 5.0        # 바뀐 게 없을 때 control_seen_at만 쓰는 주기

# ---- 테이블 (docs/DB_UI_CONTRACT.md) ----
TABLE_PRODUCT = 'product'
TABLE_LOC = 'loc'
TABLE_STATE = 'state'
STATE_ID = 1
LOC_MEASURE_ZONE = 'measure_zone'

# /control_status JSON 키 → state 열 (관제 상태에서 오는 열만)
STATUS_TO_COLUMN = {
    'state': 'stage',
    'job_id': 'job_id',
    'box_id': 'box_id',
    'box_seq': 'box_seq',
    'robot_state_name': 'robot_state',
    'robot_ok': 'robot_ok',
    'robot_connected': 'robot_connected',
    'robot_hint': 'robot_hint',
}
BOOL_COLUMNS = ('robot_ok', 'robot_connected')
INT_COLUMNS = ('box_seq',)
FINISHED_STATUS = ('DONE', 'ERROR', 'STOPPED')
SYNC_ONLY_COLUMNS = ('heartbeat_at', 'control_seen_at')   # 동기화 스레드가 매번 새로 쓰는 열

# ---- 파일 모드 처음 값 — product.json·loc.json이 없을 때 이 값으로 만든다. 실제 값은 Supabase product·loc ----
FILE_PRODUCT = [
    {'class_id': 'A', 'name': '알루미늄 묶음 A', 'width_mm': 60.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 130.0},
    {'class_id': 'B', 'name': '젠가 묶음 B', 'width_mm': 75.0, 'depth_mm': 29.0, 'height_mm': 25.0, 'weight_g': 0.0},
    {'class_id': 'C', 'name': '알루미늄 묶음 C', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 108.0},
    {'class_id': 'D', 'name': '지우개 D', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 55.3},
]
FILE_LOC = [
    {'name': 'A', 'x': 480.0, 'y': -200.0, 'z': None},
    {'name': 'B', 'x': 630.0, 'y': -200.0, 'z': None},
    {'name': 'C', 'x': 635.0, 'y': -30.0, 'z': None},
    {'name': 'D', 'x': 635.0, 'y': 115.0, 'z': None},           # 11/18 입력
    {'name': LOC_MEASURE_ZONE, 'x': 300.0, 'y': -160.0, 'z': 30.9},   # z = 측정존 바닥 (판별 실측, 11/18 30 → 30.9)
]
FILE_STATE = {        # Supabase state 테이블 기본값과 같게
    'id': STATE_ID, 'a': 0, 'b': 0, 'c': 0, 'd': 0, 'u': 0,
    'status': 'BEFORE', 'reason': '', 'started_at': None, 'finished_at': None,
    'stage': '', 'job_id': '', 'box_id': '', 'box_seq': 0, 'start_at': None,
    'robot_state': '', 'robot_ok': False, 'robot_connected': False, 'robot_hint': '',
    'control_seen_at': None, 'heartbeat_at': None, 'updated_at': None,
}
