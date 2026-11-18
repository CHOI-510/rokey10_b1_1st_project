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
    POST /cancel  → 관제 /job_command cancel (카운트다운 중 취소, 관제가 판단)
    POST /inventory {a,b,c,d,u[,source]} → 시작 전 재고 설정 (관제 READY일 때만, 11/19) → 관제에 알림
    GET  /inventory → 지금 재고 {a,b,c,d,u, capacity}
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

from box_comm.common import FIXED_POSES, INVENTORY_KEYS, REMOTE_JOB_COMMANDS, VALID_CLASSES, stack_capacity
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
        self._wake = threading.Event()   # 단계가 바뀌면 0.5초 주기를 기다리지 않고 바로 씀
        self._pending = {}           # 아직 안 쓴 관제 상태 열
        self._written = {}           # 마지막으로 쓴 관제 상태 열 (같은 값 다시 쓰지 않게)
        self._seen_wall = None       # 관제 상태를 마지막으로 받은 시각 (ISO)
        self._seen_mono = 0.0
        self._last_seen_write = 0.0
        self.stage = ''              # 지금 관제 단계 (/start 판단용)

        self._log_job = None         # 로그 파일 중복 거르기 (작업별 마지막 seq)
        self._log_seq = 0
        self._log_lock = threading.Lock()

        self._inv_updates = 0        # 이 노드가 받은 재고 반영 횟수 (시험용 estop_at)
        self._db_down = False        # 저장소 쓰기 실패 중 (재고·상태를 모아 두고 다시 보내는 중)
        self._setup_test_params(node)

        threading.Thread(target=self._sync_loop, name='db_sync', daemon=True).start()
        self._start_http()
        where = (f'파일 {self.store.folder}' if self.store.kind == 'file'
                 else f'Supabase {cfg.SUPABASE_URL}')
        self.log.info(f'[database] 저장소: {where} | 로그 {cfg.LOG_DIR} | '
                      f'버튼 창구 http://{cfg.HTTP_HOST}:{cfg.HTTP_PORT}'
                      + (' (start 토큰 사용)' if cfg.START_TOKEN else ''))

    # ------------------------------------------------------------------
    # 시험용 파라미터 (파일 모드 전용, 11/18 — 예전 box_control/fake_database 기능을 옮김)
    # ------------------------------------------------------------------
    def _setup_test_params(self, node):
        """start_inventory: [A, B, C, D(, U)] 이 값으로 재고를 덮어쓰고 시작 (기본 [-1] = 안 건드림)
        estop_at        : N번째 재고 반영 직후 UI 비상정지 버튼을 누른 것처럼 관제 /emergency_stop (0 = 끔)
        Supabase(실제 데이터)에서는 둘 다 무시한다."""
        start_inv = [int(n) for n in node.declare_parameter('start_inventory', [-1]).value]
        self.estop_at = int(node.declare_parameter('estop_at', 0).value)
        if self.store.kind != 'file':
            if (start_inv and start_inv[0] >= 0) or self.estop_at:
                self.log.warn('[database] start_inventory·estop_at은 파일 모드 시험 전용 → Supabase에서는 무시')
            self.estop_at = 0
            return
        if start_inv and start_inv[0] >= 0:
            patch = {c.lower(): (start_inv[i] if i < len(start_inv) else 0) for i, c in enumerate(INVENTORY_KEYS)}
            self.store.update_state(patch)
            self.log.warn(f'[database] (시험) 재고를 {patch}로 시작')
        if self.estop_at:
            self.log.warn(f'[database] (시험) {self.estop_at}번째 재고 반영 직후 비상정지 요청')

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
        with self._lock:                        # 아직 못 올린 재고가 있으면 그 값이 최신
            st.update({k: v for k, v in self._pending.items() if k in {c.lower() for c in INVENTORY_KEYS}})
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
        self._write_or_queue(patch, f'재고 {box_id}')
        self.log.info(f'[database] 재고 ← {box_id}: {inventory}')
        self._inv_updates += 1
        if self.estop_at and self._inv_updates == self.estop_at:
            self.log.warn(f'[database] (시험) estop_at={self.estop_at} → UI 비상정지 흉내')
            self.node.request_emergency_stop('test')

    def set_job_status(self, status, reason):
        patch = {'status': status, 'reason': reason or '', 'updated_at': now_iso()}
        if status == 'RUNNING':
            patch.update(started_at=now_iso(), finished_at=None)
        elif status == 'BEFORE':                 # 시작 취소 (사전 점검 실패·카운트다운 취소) → 지난 작업 시각 비움
            patch.update(started_at=None, finished_at=None)
        elif status in cfg.FINISHED_STATUS:
            patch['finished_at'] = now_iso()
        self._write_or_queue(patch, f'작업 상태 {status}')

    def _write_or_queue(self, patch, what):
        """재고·작업 상태 쓰기 (11/18). 저장소(Supabase)가 안 되면 실패로 돌려주지 않고 모아 두었다가
        동기화 스레드가 다시 쓴다 (새 값이 옛 값을 이김). 와이파이가 잠깐 끊겨도 작업이 멈추지 않게.
        재고는 적재가 매번 실제로 재므로, 늦게 올라가도 값이 틀어지지 않는다."""
        try:
            self.store.update_state(patch)
            return
        except Exception as e:
            err = f'{type(e).__name__}: {e}'
        with self._lock:
            self._pending.update(patch)
            first = not self._db_down
            self._db_down = True
        self._wake.set()
        self.log.warn(f'[database] {what} 저장 실패 → 모아 두고 다시 보냄: {err}')
        if first:
            self._log_db_event('WARN', f'DB 저장 실패 → 연결되면 다시 보냄 (작업은 계속): {err}')

    def _log_db_event(self, level, message):
        """DB 노드 자신의 사건을 로그 파일에 (seq 0, source 'db')"""
        event = {'seq': 0, 'stamp': time.strftime('%Y-%m-%d %H:%M:%S'), 'job_id': self._log_job or '',
                 'source': 'db', 'stage': self.stage, 'level': level, 'code': 'DB_RETRY',
                 'box_id': '', 'class_id': '', 'expected': -1, 'measured': -1, 'message': message}
        try:
            with self._log_lock:
                self._append_log(self._log_job or 'no_job', event)
        except Exception:
            pass

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
            ready_again = stage == 'READY' and self.stage != 'READY'
            if stage != self.stage:             # 카운트다운 시작·끝 → UI 팝업 남은 시간용
                sec = float(st.get('standby_sec') or 0)
                cols['start_at'] = now_iso(sec) if stage == 'STANDBY' and sec > 0 else None
                self.log.info(f'[database] 관제 단계 {self.stage or "-"} → {stage}')
                self._wake.set()
            self.stage = stage
            known = {**self._written, **self._pending}
            self._pending.update({k: v for k, v in cols.items() if known.get(k, object()) != v})
            self._seen_wall = now_iso()
            self._seen_mono = time.monotonic()
        if ready_again:
            self._close_orphan_job()

    def _close_orphan_job(self):
        """관제가 READY인데 DB에 RUNNING이 남아 있으면 정리 (관제가 작업 중에 죽고 다시 켜진 경우).
        정상 흐름에서는 관제가 결과(DONE/ERROR/STOPPED)를 쓴 뒤에 READY가 되므로 걸리지 않는다."""
        try:
            if self.store.get_state().get('status') == 'RUNNING':
                why = '작업 결과가 기록되지 않은 채 대기로 돌아옴 (관제가 작업 중 종료·재시작됨)'
                self.set_job_status('ERROR', why)
                self.log.warn(f'[database] RUNNING으로 남은 작업 정리 → ERROR : {why}')
        except Exception as e:
            self.log.warn(f'[database] 남은 작업 확인 실패: {type(e).__name__}: {e}')

    def _sync_loop(self):
        last_hb, warned = 0.0, 0.0
        while True:
            self._wake.wait(cfg.SYNC_PERIOD_SEC)
            self._wake.clear()
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
                if self._db_down:
                    with self._lock:
                        self._db_down = False
                    self.log.info('[database] 저장소 다시 연결됨 → 모아 둔 재고·상태 반영')
                    self._log_db_event('INFO', 'DB 다시 연결됨 → 모아 둔 재고·상태를 올림')
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
    # ------------------------------------------------------------------
    # 시작 전 재고 설정 (11/19) — UI 시작 팝업의 −/+ · [전부 0], 관리자 GUI 시작 확인창
    #   관제가 READY일 때만 (작업 중에 바뀌면 관제가 들고 있는 재고와 어긋남)
    #   A~D는 0 ~ 구역 정원, U는 0 이상. 적재가 실제로 재면 그 값으로 덮어써지고, 다르면 COUNT_MISMATCH로 남음
    # ------------------------------------------------------------------
    def set_inventory(self, body, headers):
        if cfg.START_TOKEN and headers.get('X-Box-Token', '') != cfg.START_TOKEN:
            return 403, {'ok': False, 'message': '권한 없음'}
        if self.stage != 'READY':
            return 200, {'ok': False, 'message': f'대기(READY)일 때만 재고를 바꿀 수 있음 (지금 관제 단계: {self.stage or "알 수 없음"})'}
        try:
            data = json.loads(body or b'{}')
            source = str(data.get('source') or 'ui')
            cap = stack_capacity()
            new = {}
            for c in INVENTORY_KEYS:
                v = data.get(c.lower(), data.get(c))
                if v is None:
                    raise ValueError(f'{c.lower()} 값이 없음')
                if isinstance(v, bool) or int(v) != float(v):
                    raise ValueError(f'{c.lower()}={v!r} 정수가 아님')
                v = int(v)
                if v < 0 or (c in VALID_CLASSES and v > cap):
                    raise ValueError(f'{c.lower()}={v} 범위 밖 (A~D 0~{cap}, U 0 이상)')
                new[c] = v
        except (ValueError, TypeError) as e:
            return 400, {'ok': False, 'message': f'잘못된 값: {e}'}
        st = self.store.get_state()
        with self._lock:
            st.update({k: v for k, v in self._pending.items() if k in {c.lower() for c in INVENTORY_KEYS}})
        old = {c: int(st.get(c.lower()) or 0) for c in INVENTORY_KEYS}
        changed = [f'{c} {old[c]}→{new[c]}' for c in INVENTORY_KEYS if old[c] != new[c]]
        if changed:
            patch = {c.lower(): new[c] for c in INVENTORY_KEYS}
            patch['updated_at'] = now_iso()
            self._write_or_queue(patch, f'재고 설정 ({source})')
        msg = f'재고 설정 ({source}): ' + (', '.join(changed) if changed else '바뀐 값 없음')
        self.log.warn(f'[database] {msg}')
        event = {'seq': 0, 'stamp': time.strftime('%Y-%m-%d %H:%M:%S'), 'job_id': self._log_job or '',
                 'source': source if source in ('ui', 'monitor_gui') else 'ui', 'stage': self.stage,
                 'level': 'WARN' if changed else 'INFO', 'code': 'INVENTORY_SET', 'box_id': '', 'class_id': '',
                 'expected': -1, 'measured': -1, 'message': msg}
        with self._log_lock:
            self._append_log(self._log_job or 'no_job', event)
        if changed:                              # 관제가 READY 화면·상태에 새 재고를 바로 보이게
            ok, why = self.node.request_job_command('inventory_changed', 'db')
            if not ok:
                self.log.warn(f'[database] 관제에 재고 변경 알림 실패: {why} (다음 시작 때 DB 값을 읽으므로 작업에는 영향 없음)')
        return 200, {'ok': True, 'message': msg, 'inventory': new}

    def get_inventory(self):
        st = self.store.get_state()
        with self._lock:
            st.update({k: v for k, v in self._pending.items() if k in {c.lower() for c in INVENTORY_KEYS}})
        out = {c.lower(): int(st.get(c.lower()) or 0) for c in INVENTORY_KEYS}
        out['capacity'] = stack_capacity()
        return out

    def handle_command(self, command, headers):
        if command == 'estop':
            ok = self.node.request_emergency_stop('ui')
            msg = '' if ok else '관제 연결 안 됨 — 펜던트 비상정지 버튼을 쓰세요'
        elif command in REMOTE_JOB_COMMANDS:            # start_now(바로 시작)는 현장 전용 → 여기로 못 옴
            if cfg.START_TOKEN and headers.get('X-Box-Token', '') != cfg.START_TOKEN:
                return 403, {'ok': False, 'message': '권한 없음'}
            if command == 'start' and self.stage != 'READY':
                ok, msg = False, f'준비 안 됨 (지금 관제 단계: {self.stage or "알 수 없음"})'
            else:                               # cancel은 관제가 단계를 보고 판단
                ok, msg = self.node.request_job_command(command, 'ui')
        else:
            return 404, {'ok': False, 'message': f'알 수 없는 명령: {command}'}
        self.log.warn(f'[database] UI {command} → {"OK" if ok else "거절"} {msg}')
        self._log_ui_command(command, ok, msg)
        return 200, {'ok': bool(ok), 'message': msg}

    def handle_get(self, path):
        if path == '/health':
            return 200, {'ok': True, 'stage': self.stage, 'store': self.store.kind,
                         'folder': getattr(self.store, 'folder', ''), 'log_dir': cfg.LOG_DIR}
        if path == '/inventory':
            try:
                return 200, self.get_inventory()
            except Exception as e:
                return 500, {'ok': False, 'message': f'{type(e).__name__}: {e}'}
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
                body = self.rfile.read(n) if n else b''
                if self.path.strip('/') == 'inventory':      # 본문을 쓰는 건 재고 설정뿐
                    self._send(*work.set_inventory(body, self.headers))
                    return
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
