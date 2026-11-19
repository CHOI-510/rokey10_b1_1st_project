#!/usr/bin/env python3
"""
database_work.py — DB 노드가 하는 일 (관제·DB: 정쓰)
================================================================
ROS 통신은 box_comm/database_interface.py가 하고, 이 클래스의 함수를 부른다.
저장소(Supabase / 파일)는 database_store.py, 설정은 database_config.py.

무엇을 어디에 두나 (11/19)
  DB(Supabase)   product · loc · inventory(재고 1행)  ← 기록할 것만. 재고가 바뀔 때만 씀
  메모리         작업 상태 · 로봇 · 연결              ← GET /state 로 보여 주기만 (휘발)
  로그 파일      ~/box_logs/{job_id}.jsonl            ← 이벤트 전부 (작업 이력 = JOB_* 줄)

  관제 → DB 노드 (ROS)                      DB 노드가 하는 일
  -----------------------------------------------------------------------------
  get_job_data()        작업 시작 때          product(분류 기준) + inventory(재고) 읽기
  get_fixed_poses()     작업 시작 때          loc(측정존·적재 구역) 읽기, HOME은 코드 값
  update_inventory()    박스 하나 끝날 때     inventory 쓰기 (실패하면 모아 두고 다시 보냄)
  set_job_status()      RUNNING·DONE 등      메모리에만 (/state의 status·reason)
  on_state()            /control_status      메모리에만 (/state의 status·로봇·연결)
  on_event()            /event_log           로그 파일에 한 줄

  웹 → DB 노드 (같은 노트북, HTTP 127.0.0.1:8765)
    GET  /state      → 화면 한 장에 필요한 것 전부 (웹이 1초마다)
    POST /estop      → 관제 /emergency_stop     (언제든, 인터넷 없어도 동작)
    POST /start      → 관제 /job_command start  (관제가 READY일 때만)
    POST /cancel     → 관제 /job_command cancel (카운트다운 중 취소, 관제가 판단)
    POST /inventory  {a,b,c,d,u[,source]} → 시작 전 재고 설정 (관제 READY일 때만)
    GET  /inventory  → {a,b,c,d,u, capacity}
    GET  /health     → {ok, stage, store}  (시험·디버그용)
================================================================
"""
import json
import os
import threading
import time
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from box_comm.common import (FIXED_POSES, INVENTORY_KEYS, JOB_COMMANDS, REMOTE_JOB_COMMANDS, VALID_CLASSES,
                             stack_capacity)
from box_database import database_config as cfg
from box_database.database_store import make_store


def now_iso(offset_sec=0.0):
    return (datetime.now(timezone.utc) + timedelta(seconds=offset_sec)).isoformat()


def _iso_from_epoch(t):
    return datetime.fromtimestamp(float(t), timezone.utc).isoformat()


class DatabaseWork:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        mode = str(node.declare_parameter('store', 'auto').value).lower()
        fake_dir = str(node.declare_parameter('fake_dir', cfg.DEFAULT_FAKE_DIR).value)
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        self.store = make_store(mode, fake_dir, self.log)

        self._lock = threading.Lock()
        # 재고: DB가 원본, 메모리는 마지막으로 알고 있는 값. 저장 실패 중이면 메모리 값이 최신
        self._inv = {c: 0 for c in INVENTORY_KEYS}
        self._inv_unsaved = False
        self._inv_wake = threading.Event()
        # 실시간 상태 (휘발) — /state
        self.stage = ''                  # 관제 단계 (/start·/inventory 판단용)
        self._result = {'status': 'BEFORE', 'reason': '', 'started_at': None, 'finished_at': None}
        self._robot = {'robot_ok': False, 'robot_hint': ''}
        self._start_at = None
        self._seen = 0.0                 # 관제 상태를 마지막으로 받은 시각 (monotonic)

        self._log_job = None             # 로그 파일 중복 거르기 (작업별 마지막 seq)
        self._log_seq = 0
        self._log_lock = threading.Lock()
        self._inv_updates = 0            # 받은 재고 반영 횟수 (시험용 estop_at)

        self._load_inventory()
        self._setup_test_params(node)
        threading.Thread(target=self._retry_loop, name='db_retry', daemon=True).start()
        self._start_http()
        where = f'파일 {self.store.folder}' if self.store.kind == 'file' else f'Supabase {cfg.SUPABASE_URL}'
        self.log.info(f'[database] 저장소: {where} (product·loc·inventory) | 로그 {cfg.LOG_DIR} | '
                      f'창구 http://{cfg.HTTP_HOST}:{cfg.HTTP_PORT}' + (' (토큰 사용)' if cfg.START_TOKEN else ''))

    # ------------------------------------------------------------------
    # 시험용 파라미터 (파일 모드 전용)
    # ------------------------------------------------------------------
    def _setup_test_params(self, node):
        """start_inventory: [A, B, C, D(, U)] 이 값으로 재고를 덮어쓰고 시작 (기본 [-1] = 안 건드림)
        estop_at        : N번째 재고 반영 직후 UI 비상정지를 누른 것처럼 관제 /emergency_stop (0 = 끔)
        Supabase(실제 데이터)에서는 둘 다 무시한다."""
        start_inv = [int(n) for n in node.declare_parameter('start_inventory', [-1]).value]
        self.estop_at = int(node.declare_parameter('estop_at', 0).value)
        if self.store.kind != 'file':
            if (start_inv and start_inv[0] >= 0) or self.estop_at:
                self.log.warn('[database] start_inventory·estop_at은 파일 모드 시험 전용 → Supabase에서는 무시')
            self.estop_at = 0
            return
        if start_inv and start_inv[0] >= 0:
            inv = {c: (start_inv[i] if i < len(start_inv) else 0) for i, c in enumerate(INVENTORY_KEYS)}
            self._save_inventory(inv, '시험 시작값')
            self.log.warn(f'[database] (시험) 재고를 {inv}로 시작')
        if self.estop_at:
            self.log.warn(f'[database] (시험) {self.estop_at}번째 재고 반영 직후 비상정지 요청')

    # ------------------------------------------------------------------
    # 재고 (DB에 기록하는 유일한 가변 값)
    # ------------------------------------------------------------------
    def _load_inventory(self):
        try:
            row = self.store.get_inventory()
            with self._lock:
                self._inv = {c: int(row.get(c.lower()) or 0) for c in INVENTORY_KEYS}
        except Exception as e:
            self.log.error(f'[database] 재고 읽기 실패: {type(e).__name__}: {e}')
            raise

    def _current_inventory(self):
        """작업 시작·화면용 재고. 못 올린 값이 있으면 메모리, 아니면 DB를 다시 읽음 (사람이 DB를 고친 경우 반영)"""
        with self._lock:
            if self._inv_unsaved:
                return dict(self._inv)
        try:
            self._load_inventory()
        except Exception:
            pass                                  # DB가 안 되면 마지막으로 알던 값
        with self._lock:
            return dict(self._inv)

    def _save_inventory(self, inv, what):
        """재고 쓰기. 실패해도 관제에 실패로 돌려주지 않고 모아 두었다가 다시 보낸다 (작업이 멈추지 않게).
        재고는 적재가 매번 실제로 재므로, 늦게 올라가도 값이 틀어지지 않는다."""
        with self._lock:
            self._inv = {c: int(inv.get(c, 0)) for c in INVENTORY_KEYS}
            values = {c.lower(): n for c, n in self._inv.items()}
        try:
            self.store.set_inventory(values)
            with self._lock:
                recovered, self._inv_unsaved = self._inv_unsaved, False
            if recovered:
                self._log_db_event('INFO', 'DB 다시 연결됨 → 모아 둔 재고를 올림')
            return
        except Exception as e:
            err = f'{type(e).__name__}: {e}'
        with self._lock:
            first, self._inv_unsaved = not self._inv_unsaved, True
        self._inv_wake.set()
        self.log.warn(f'[database] {what} 저장 실패 → 다시 보냄: {err}')
        if first:
            self._log_db_event('WARN', f'DB 저장 실패 → 연결되면 다시 보냄 (작업은 계속): {err}')

    def _retry_loop(self):
        while True:
            self._inv_wake.wait(cfg.RETRY_SEC)
            self._inv_wake.clear()
            with self._lock:
                if not self._inv_unsaved:
                    continue
                inv = dict(self._inv)
            time.sleep(cfg.RETRY_SEC)
            self._save_inventory(inv, '재고 다시 보내기')

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
        return specs, self._current_inventory()

    def get_fixed_poses(self):
        locs = {r['name']: r for r in self.store.locs()}
        mz = locs.get(cfg.LOC_MEASURE_ZONE)
        if mz is None or mz.get('z') is None:
            self.log.error(f'[database] {cfg.TABLE_LOC}에 {cfg.LOC_MEASURE_ZONE}(x, y, z) 없음 → 관제 사전 점검 FAIL')
            return None
        zones = {}
        for c in VALID_CLASSES:
            r = locs.get(c)
            zones[c] = [float(r['x']), float(r['y'])] if r else None      # 빠지면 관제가 사전 점검 FAIL
        return {'home_j': list(FIXED_POSES['home_j']),                     # HOME은 코드가 원본
                'measure_zone': [float(mz['x']), float(mz['y']), float(mz['z'])],
                'place_zones': zones}

    def update_inventory(self, box_id, inventory):
        self._save_inventory(inventory, f'재고 {box_id}')
        self.log.info(f'[database] 재고 ← {box_id}: {inventory}')
        self._inv_updates += 1
        if self.estop_at and self._inv_updates == self.estop_at:
            self.log.warn(f'[database] (시험) estop_at={self.estop_at} → UI 비상정지 흉내')
            self.node.request_emergency_stop('test')

    def set_job_status(self, status, reason):
        """작업 결과 — 메모리에만 (기록은 로그 파일의 JOB_* 줄)"""
        with self._lock:
            r = self._result
            r['status'], r['reason'] = status, reason or ''
            if status == 'RUNNING':
                r['started_at'], r['finished_at'] = now_iso(), None
            elif status == 'BEFORE':               # 시작 안 함 (사전 점검 실패·취소)
                r['started_at'], r['finished_at'] = None, None
            elif status in cfg.FINISHED_STATUS:
                r['finished_at'] = now_iso()
        self.log.info(f'[database] 작업 {status}' + (f' : {reason}' if reason else ''))

    # ------------------------------------------------------------------
    # 관제 상태 (/control_status, 2초마다 + 바뀌면 즉시) — 메모리에만
    # ------------------------------------------------------------------
    def on_state(self, st):
        stage = str(st.get('state') or '')
        with self._lock:
            prev, self.stage = self.stage, stage
            self._seen = time.monotonic()
            if 'robot_ok' in st:
                self._robot['robot_ok'] = bool(st['robot_ok'])
            if 'robot_hint' in st:
                self._robot['robot_hint'] = str(st.get('robot_hint') or '')
            if stage == 'STANDBY':
                end = st.get('start_at')               # 관제가 보낸 카운트다운 끝 (epoch) — 관리자 GUI와 같은 값
                if end:
                    self._start_at = _iso_from_epoch(end)
                elif prev != 'STANDBY':
                    self._start_at = now_iso(float(st.get('standby_sec') or 0))
            else:
                self._start_at = None
            orphan = stage == 'READY' and self._result['status'] == 'RUNNING'
        if stage != prev:
            self.log.info(f'[database] 관제 단계 {prev or "-"} → {stage}')
        if orphan:                                  # 관제가 작업 중에 죽었다가 다시 켜진 경우
            self.set_job_status('ERROR', '작업 결과 없이 대기로 돌아옴 (관제가 작업 중 종료·재시작됨)')

    def _status_now(self):
        """화면 status: 점검·카운트다운 STARTING, 작업 중 RUNNING, 그 밖은 마지막 결과"""
        if self.stage in cfg.STARTING_STAGES:
            return 'STARTING'
        if self.stage in cfg.RUNNING_STAGES:
            return 'RUNNING'
        return self._result['status']

    def live_state(self):
        with self._lock:
            control_ok = self._seen > 0 and time.monotonic() - self._seen < cfg.CONTROL_ALIVE_SEC
            out = {'status': self._status_now(),
                   'reason': self._result['reason'],
                   'start_at': self._start_at,
                   'started_at': self._result['started_at'],
                   'finished_at': self._result['finished_at'],
                   'robot_ok': self._robot['robot_ok'] and control_ok,
                   'robot_hint': self._robot['robot_hint'] if control_ok else '',
                   'control_ok': control_ok,
                   'db_ok': not self._inv_unsaved,
                   # 재고는 메모리 값 (웹이 1초마다 물어도 DB를 읽지 않음. 바뀔 때마다 메모리와 DB를 같이 씀)
                   'inventory': {c.lower(): n for c, n in self._inv.items()}}
        out['capacity'] = stack_capacity()
        out['at'] = now_iso()
        return out

    # ------------------------------------------------------------------
    # 이벤트 로그 (/event_log) → 파일 (웹은 Next.js가 최근 N줄을 읽음)
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

    def _write_own_log(self, source, level, code, message):
        """DB 노드 자신이 남기는 줄 (seq 0 = 관제가 보낸 줄이 아님)"""
        event = {'seq': 0, 'stamp': time.strftime('%Y-%m-%d %H:%M:%S'), 'job_id': self._log_job or '',
                 'source': source, 'stage': self.stage, 'level': level, 'code': code,
                 'box_id': '', 'class_id': '', 'expected': -1, 'measured': -1, 'message': message}
        try:
            with self._log_lock:
                self._append_log(self._log_job or 'no_job', event)
        except Exception:
            pass

    def _log_db_event(self, level, message):
        self._write_own_log('db', level, 'DB_RETRY', message)

    # ------------------------------------------------------------------
    # 시작 전 재고 설정 — UI 시작 팝업의 −/+ · [전부 0]
    #   관제가 READY일 때만. A~D는 0 ~ 구역 정원, U는 0 이상
    # ------------------------------------------------------------------
    def set_inventory(self, body, headers):
        if cfg.START_TOKEN and headers.get('X-Box-Token', '') != cfg.START_TOKEN:
            return 403, {'ok': False, 'message': '권한 없음'}
        if self.stage != 'READY':
            return 200, {'ok': False,
                         'message': f'대기(READY)일 때만 재고를 바꿀 수 있음 (지금 관제 단계: {self.stage or "알 수 없음"})'}
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
        old = self._current_inventory()
        changed = [f'{c} {old[c]}→{new[c]}' for c in INVENTORY_KEYS if old[c] != new[c]]
        if changed:
            self._save_inventory(new, f'재고 설정 ({source})')
        msg = f'재고 설정 ({source}): ' + (', '.join(changed) if changed else '바뀐 값 없음')
        self.log.warn(f'[database] {msg}')
        self._write_own_log(source if source in ('ui', 'monitor_gui') else 'ui',
                            'WARN' if changed else 'INFO', 'INVENTORY_SET', msg)
        if changed and 'inventory_changed' in JOB_COMMANDS:     # 관제가 이 알림을 받는 경우에만
            ok, why = self.node.request_job_command('inventory_changed', 'db')
            if not ok:
                self.log.warn(f'[database] 관제에 재고 변경 알림 실패: {why} (다음 시작 때 DB 값을 읽으므로 작업에는 영향 없음)')
        return 200, {'ok': True, 'message': msg, 'inventory': new}

    def get_inventory(self):
        out = {c.lower(): n for c, n in self._current_inventory().items()}
        out['capacity'] = stack_capacity()
        return out

    # ------------------------------------------------------------------
    # 웹 창구 (HTTP, 127.0.0.1 전용)
    # ------------------------------------------------------------------
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
        self._write_own_log('ui', 'WARN' if command == 'estop' else 'INFO', 'UI_COMMAND',
                            f'UI {command} → {"OK" if ok else "거절"} {msg}'.strip())
        return 200, {'ok': bool(ok), 'message': msg}

    def handle_get(self, path):
        try:
            if path == '/state':
                return 200, self.live_state()
            if path == '/inventory':
                return 200, self.get_inventory()
            if path == '/health':
                return 200, {'ok': True, 'stage': self.stage, 'store': self.store.kind,
                             'folder': getattr(self.store, 'folder', ''), 'log_dir': cfg.LOG_DIR}
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
                self._send(*work.handle_get(self.path.split('?')[0]))

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
            self.log.error(f'[database] 창구를 못 엶 ({cfg.HTTP_HOST}:{cfg.HTTP_PORT}): {e} '
                           '— 다른 DB 노드가 이미 떠 있는지 확인')
            return
        threading.Thread(target=server.serve_forever, name='db_http', daemon=True).start()
