#!/usr/bin/env python3
"""
database_config.py — DB 노드 설정 (관제·DB: 정쓰)
================================================================
무엇을 어디에 두나 (11/19)
  DB(Supabase)   기록할 것만: product(기준값) · loc(좌표) · inventory(재고 1행)
  DB 노드 메모리  실시간 상태(작업·로봇·연결) → GET /state 로 웹에 보여 주기만 (휘발)
  로그 파일       이벤트 전부 → ~/box_logs/{job_id}.jsonl (작업 이력도 여기: JOB_* 줄)

환경 변수 (터미널에서 export, 코드·git에 키를 넣지 않는다)
  SUPABASE_URL           https://xxxx.supabase.co
  SUPABASE_SERVICE_KEY   service_role 키 (DB 노드 전용. 웹 코드에 절대 넣지 말 것)
  BOX_LOG_DIR            이벤트 로그 파일 폴더 (기본 ~/box_logs)
  BOX_DB_PORT            창구 포트 (기본 8765, 127.0.0.1에서만 받음)
  BOX_START_TOKEN        설정하면 /start·/cancel·/inventory에 헤더 X-Box-Token이 같아야 받음

ROS 파라미터 (ros2 run box_database database_main --ros-args -p store:=file)
  store     'auto'(기본: 키가 있으면 supabase, 없으면 file) / 'supabase' / 'file'
  fake_dir  파일 모드 폴더 (기본 ~/box_logs/fake_db): product.json · loc.json · inventory.json
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

# ---- 시간 ----
DB_TIMEOUT_SEC = 3          # Supabase 호출 한 번 최대 대기 (관제가 응답을 기다리므로 짧게)
RETRY_SEC = 2.0             # 재고 저장 실패 시 다시 보내는 주기
CONTROL_ALIVE_SEC = 5.0     # 관제 상태를 이 시간 넘게 못 받으면 control_ok = False (관제는 2초마다 보냄)

# ---- 테이블 (docs/DB_UI_CONTRACT.md) ----
TABLE_PRODUCT = 'product'
TABLE_LOC = 'loc'
TABLE_INVENTORY = 'inventory'
INVENTORY_ID = 1
LOC_MEASURE_ZONE = 'measure_zone'

# ---- 관제 단계 → 화면 status (/state) ----
#   STARTING = 점검·카운트다운, RUNNING = 작업 중. 그 밖(READY 등)은 마지막 작업 결과를 그대로 보여 줌
STARTING_STAGES = ('BEFORE', 'PREFLIGHT', 'STANDBY')
RUNNING_STAGES = ('RUNNING', 'SCANNING', 'SCAN_DONE', 'JUDGING', 'JUDGE_DONE',
                  'PLACING', 'PLACE_DONE', 'CLEARING')
FINISHED_STATUS = ('DONE', 'ERROR', 'STOPPED')

# ---- 파일 모드 처음 값 — 파일이 없을 때 이 값으로 만든다. 실제 값은 Supabase ----
FILE_PRODUCT = [
    {'class_id': 'A', 'name': '알루미늄 묶음 A', 'width_mm': 60.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 130.0},
    {'class_id': 'B', 'name': '젠가 묶음 B', 'width_mm': 75.0, 'depth_mm': 29.0, 'height_mm': 25.0, 'weight_g': 0.0},
    {'class_id': 'C', 'name': '알루미늄 묶음 C', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 108.0},
    {'class_id': 'D', 'name': '지우개 D', 'width_mm': 50.0, 'depth_mm': 40.0, 'height_mm': 20.0, 'weight_g': 55.3},
]
FILE_LOC = [
    {'name': 'A', 'x': 480.0, 'y': -160.0, 'z': None},
    {'name': 'B', 'x': 650.0, 'y': -160.0, 'z': None},
    {'name': 'C', 'x': 580.0, 'y': 300.0, 'z': None},
    {'name': 'D', 'x': 300.0, 'y': 300.0, 'z': None},           # 11/18 입력
    {'name': LOC_MEASURE_ZONE, 'x': 300.0, 'y': -160.0, 'z': 30.9},   # z = 측정존 바닥 (판별 실측, 11/18 30 → 30.9)
]
FILE_INVENTORY = {'id': INVENTORY_ID, 'a': 0, 'b': 0, 'c': 0, 'd': 0, 'u': 0, 'updated_at': None}
