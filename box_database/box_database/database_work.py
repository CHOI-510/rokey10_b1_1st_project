#!/usr/bin/env python3
"""
database_work.py — DB 노드가 하는 일 (관제·DB: 정쓰)
================================================================
ROS 통신은 box_comm/database_interface.py가 하고, 이 클래스의 함수를 부른다.
저장소(Supabase / 파일)는 database_store.py, 설정은 database_config.py.

  관제 → DB 노드 (ROS)                      DB 노드가 하는 일
  -----------------------------------------------------------------------------
  get_job_data()        작업 시작 때          product(분류 기준) + state(재고) 읽기
  get_fixed_poses()     작업 시작 때          loc(측정존·적재 구역) 읽기, HOME은 코드 값
  update_inventory()    박스 하나 끝날 때     state 재고 열 쓰기 (관제가 응답을 기다림)
  set_job_status()      RUNNING·DONE 등      state 작업 결과 열 쓰기 (관제가 응답을 기다림)
  on_state()            /control_status      바뀐 열만 모아 두고 동기화 스레드가 씀 (관제를 안 막음)
  on_event()            /event_log           ~/box_logs/{job_id}.jsonl에 한 줄 (DB에는 안 올림)

  웹 UI 버튼 → DB 노드 (같은 노트북, HTTP 127.0.0.1:8765)
    POST /estop   → 관제 /emergency_stop     (언제든, 인터넷 없어도 동작)
    POST /start   → 관제 /job_command start  (관제 단계가 READY일 때만)
    GET  /health  → {ok, stage, store}
    GET  /state   → 지금 state 행 (시험·디버그용)
================================================================
"""
import json
import os
import threading
import time
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from box_comm.common import FIXED_POSES, INVENTORY_KEYS, VALID_CLASSES
from box_database import database_config as cfg
from box_database.database_store import make_stores


def now_iso(offset_sec=0.0):
    return (datetime.now(timezone.utc) + timedelta(seconds=offset_sec)).isoformat()


class DatabaseWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        mode = str(node.declare_parameter('store', 'auto').value).lower()
        fake_dir = str(node.declare_parameter('fake_dir', cfg.DEFAULT_FAKE_DIR).value)
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        self.store, self._sync_store = make_stores(mode, fake_dir, self.log)

        self._lock = threading.Lock()
        self._pending = {}           # 아직 안 쓴 관제 상태 열
        self._written = {}           # 마지막으로 쓴 관제 상태 열 (같은 값 다시 쓰지 않게)
        self._seen_wall = None       # 관제 상태를 마지막으로 받은 시각 (ISO)
        self._seen_mono = 0.0
        self._last_seen_write = 0.0
        self.stage = ''              # 지금 관제 단계 (/start 판단용)

        self._log_job = None         # 로그 파일 중복 거르기 (작업별 마지막 seq)
        self._log_seq = 0
        self._log_lock = threading.Lock()

        threading.Thread(target=self._sync_loop, name='db_sync', daemon=True).start()
        self._start_http()
        where = (f'파일 {self.store.folder}' if self.store.kind == 'file'
                 else f'Supabase {cfg.SUPABASE_URL}')
        self.log.info(f'[database] 저장소: {where} | 로그 {cfg.LOG_DIR} | '
                      f'버튼 창구 http://{cfg.HTTP_HOST}:{cfg.HTTP_PORT}'
                      + (' (start 토큰 사용)' if cfg.START_TOKEN else ''))

    # ------------------------------------------------------------------
    # 관제가 부르는 함수 (응답을 기다림 → 짧게)
    # ------------------------------------------------------------------
    def get_job_data(self):
        specs = []
        for r in self.store.products():
            if r['class_id'] not in VALID_CLASSES:
                continue
            specs.append({'class_id': r['class_id'],
                          'width_mm': float(r['width_mm']), 'depth_mm': float(r['depth_mm']),
                          'height_mm': float(r['height_mm']),
                          'weight_g': float(r.get('weight_g') or 0.0)})     # NULL → 0 (미측정)
        missing = set(VALID_CLASSES) - {s['class_id'] for s in specs}
        if missing:
            raise ValueError(f'{cfg.TABLE_PRODUCT}에 클래스 {sorted(missing)} 행이 없음')
        st = self.store.get_state()
        inventory = {c: int(st.get(c.lower()) or 0) for c in INVENTORY_KEYS}
        return specs, inventory

    def get_fixed_poses(self):
        locs = {r['name']: r for r in self.store.locs()}
        mz = locs.get(cfg.LOC_MEASURE_ZONE)
        if mz is None or mz.get('z') is None:
            self.log.error(f'[database] {cfg.TABLE_LOC}에 {cfg.LOC_MEASURE_ZONE}(x, y, z) 없음 '
                           '→ 관제 사전 점검 FAIL')
            return None
        zones = {}
        for c in VALID_CLASSES:
            r = locs.get(c)
            zones[c] = [float(r['x']), float(r['y'])] if r else None      # 빠지면 관제가 사전 점검 FAIL
        return {'home_j': list(FIXED_POSES['home_j']),                     # HOME은 코드가 원본 (11/18)
                'measure_zone': [float(mz['x']), float(mz['y']), float(mz['z'])],
                'place_zones': zones}

    def update_inventory(self, box_id, inventory):
        patch = {c.lower(): int(inventory.get(c, 0)) for c in INVENTORY_KEYS}
        patch['updated_at'] = now_iso()
        self.store.update_state(patch)
        self.log.info(f'[database] 재고 ← {box_id}: {inventory}')

    def set_job_status(self, status, reason):
        patch = {'status': status, 'reason': reason or '', 'updated_at': now_iso()}
        if status == 'RUNNING':
            patch.update(started_at=now_iso(), finished_at=None)
        elif status in cfg.FINISHED_STATUS:
            patch['finished_at'] = now_iso()
        self.store.update_state(patch)

    # ------------------------------------------------------------------
    # 관제 상태 (/control_status) — 모아 두기만 하고 쓰기는 동기화 스레드
    # ------------------------------------------------------------------
    def on_state(self, st):
        cols = {}
        for key, col in cfg.STATUS_TO_COLUMN.items():
            if key in st:
                v = st[key]
                cols[col] = (int(v or 0) if col in cfg.INT_COLUMNS
                             else bool(v) if col in cfg.BOOL_COLUMNS
                             else '' if v is None else str(v))
        with self._lock:
            stage = cols.get('stage', self.stage)
            if stage != self.stage:             # 카운트다운 시작·끝 → UI 팝업 남은 시간용
                sec = float(st.get('standby_sec') or 0)
                cols['start_at'] = now_iso(sec) if stage == 'STANDBY' and sec > 0 else None
                self.log.info(f'[database] 관제 단계 {self.stage or "-"} → {stage}')
            self.stage = stage
            known = {**self._written, **self._pending}
            self._pending.update({k: v for k, v in cols.items() if known.get(k, object()) != v})
            self._seen_wall = now_iso()
            self._seen_mono = time.monotonic()

    def _sync_loop(self):
        last_hb, warned = 0.0, 0.0
        while True:
            time.sleep(cfg.SYNC_PERIOD_SEC)
            t = time.monotonic()
            with self._lock:
                patch, self._pending = self._pending, {}
                fresh = self._seen_mono > self._last_seen_write
                if self._seen_wall and (patch or (fresh and t - self._last_seen_write >= cfg.SEEN_WRITE_SEC)):
                    patch['control_seen_at'] = self._seen_wall
            if t - last_hb >= cfg.HEARTBEAT_SEC:
                patch['heartbeat_at'] = now_iso()
            if not patch:
                continue
            try:
                self._sync_store.update_state(patch)
                if 'heartbeat_at' in patch:
                    last_hb = t
                if 'control_seen_at' in patch:
                    self._last_seen_write = t
                with self._lock:
                    self._written.update({k: v for k, v in patch.items() if k not in cfg.SYNC_ONLY_COLUMNS})
            except Exception as e:      # 끊김 → 다음 주기에 최신값으로 다시 (오래된 값이 새 값을 덮지 않게)
                with self._lock:
                    for k, v in patch.items():
                        if k not in cfg.SYNC_ONLY_COLUMNS:
                            self._pending.setdefault(k, v)
                if t - warned > 10.0:
                    self.log.warn(f'[database] state 동기화 실패 (재시도 중): {type(e).__name__}: {e}')
                    warned = t

    # ------------------------------------------------------------------
    # 이벤트 로그 (/event_log) → 파일 (UI는 Next.js API가 최근 N줄을 읽음)
    # ------------------------------------------------------------------
    def on_event(self, event):
        job = event.get('job_id') or 'no_job'
        seq = int(event.get('seq') or 0)
        with self._log_lock:
            if job != self._log_job:
                self._log_job, self._log_seq = job, self._last_seq_in_file(job)
            if 0 < seq <= self._log_seq:
                return                   # DB 노드 재시작으로 다시 온 줄
            if seq > 0:
                self._log_seq = seq
            self._append_log(job, event)

    def _log_path(self, job):
        return os.path.join(cfg.LOG_DIR, f'{job}.jsonl')

    def _last_seq_in_file(self, job):
        try:
            with open(self._log_path(job), 'rb') as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 4096))
                lines = f.read().decode('utf-8', 'ignore').strip().splitlines()
        except FileNotFoundError:
            return 0
        for line in reversed(lines):
            try:
                s = int(json.loads(line).get('seq') or 0)
            except ValueError:
                continue
            if s > 0:
                return s
        return 0

    def _append_log(self, job, event):
        line = dict(event, received_at=now_iso())
        with open(self._log_path(job), 'a', encoding='utf-8') as f:
            f.write(json.dumps(line, ensure_ascii=False) + '\n')

    def _log_ui_command(self, command, ok, message):
        """UI 버튼 기록 (seq 0 = 관제가 보낸 줄이 아님)"""
        event = {'seq': 0, 'stamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                 'job_id': self._log_job or '', 'source': 'ui', 'stage': self.stage,
                 'level': 'WARN' if command == 'estop' else 'INFO', 'code': 'UI_COMMAND',
                 'box_id': '', 'class_id': '', 'expected': -1, 'measured': -1,
                 'message': f'UI {command} → {"OK" if ok else "거절"} {message}'.strip()}
        with self._log_lock:
            self._append_log(self._log_job or 'no_job', event)

    # ------------------------------------------------------------------
    # UI 버튼 창구 (HTTP, 127.0.0.1 전용)
    # ------------------------------------------------------------------
    def handle_command(self, command, headers):
        if command == 'estop':
            ok = self.node.request_emergency_stop('ui')
            msg = '' if ok else '관제 연결 안 됨 — 펜던트 비상정지 버튼을 쓰세요'
        elif command == 'start':
            if cfg.START_TOKEN and headers.get('X-Box-Token', '') != cfg.START_TOKEN:
                return 403, {'ok': False, 'message': '시작 권한 없음'}
            if self.stage != 'READY':
                ok, msg = False, f'준비 안 됨 (지금 관제 단계: {self.stage or "알 수 없음"})'
            else:
                ok, msg = self.node.request_job_command('start', 'ui')
        else:
            return 404, {'ok': False, 'message': f'알 수 없는 명령: {command}'}
        self.log.warn(f'[database] UI {command} → {"OK" if ok else "거절"} {msg}')
        self._log_ui_command(command, ok, msg)
        return 200, {'ok': bool(ok), 'message': msg}

    def handle_get(self, path):
        if path == '/health':
            return 200, {'ok': True, 'stage': self.stage, 'store': self.store.kind,
                         'folder': getattr(self.store, 'folder', ''), 'log_dir': cfg.LOG_DIR}
        if path == '/state':
            try:
                return 200, self.store.get_state()
            except Exception as e:
                return 500, {'ok': False, 'message': f'{type(e).__name__}: {e}'}
        return 404, {'ok': False, 'message': 'not found'}

    def _start_http(self):
        work = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, code, body):
                data = json.dumps(body, ensure_ascii=False, default=str).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self._send(*work.handle_get(self.path))

            def do_POST(self):
                n = int(self.headers.get('Content-Length') or 0)
                if n:
                    self.rfile.read(n)          # 본문은 쓰지 않음
                self._send(*work.handle_command(self.path.strip('/'), self.headers))

            def log_message(self, fmt, *args):  # 기본 출력(stderr) 끄기
                pass

        try:
            server = ThreadingHTTPServer((cfg.HTTP_HOST, cfg.HTTP_PORT), Handler)
        except OSError as e:
            self.log.error(f'[database] 버튼 창구를 못 엶 ({cfg.HTTP_HOST}:{cfg.HTTP_PORT}): {e} '
                           '— 다른 DB 노드가 이미 떠 있는지 확인')
            return
        threading.Thread(target=server.serve_forever, name='db_http', daemon=True).start()
