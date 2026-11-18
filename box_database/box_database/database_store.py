#!/usr/bin/env python3
"""
database_store.py — 저장소 두 가지 (같은 함수 4개)
================================================================
  SupabaseStore : 실제 Supabase (product, loc, state 테이블)
  FileStore     : Supabase 대신 JSON 파일 (DB 연동 전 시험·확인용)
                  폴더(기본 ~/box_logs/fake_db/)에 테이블마다 파일 하나:
                    product.json        A~D 4행    ← 처음엔 database_config 값으로 만들어짐, 직접 고쳐도 됨
                    loc.json            위치 5행   ← 같음 (작업 시작 때마다 다시 읽음)
                    state.json          1행        ← DB 노드가 씀 (Supabase state 행과 같은 모양)
                    state_updates.jsonl state에 쓴 내용 기록 (한 줄 = update 한 번: 시각, 누가, 바뀐 열)
                                        by: service = 관제 서비스(재고·작업 결과), control_status = 관제 단계·로봇
                                        → Supabase에 무엇이 언제 올라가는지 그대로 보임
                                        (2초마다 도는 heartbeat·control_seen_at만 바뀐 줄은 빼고 기록)
                  Supabase로 옮길 때: product.json·loc.json 값을 테이블에 넣고 store를 supabase로 바꾸면 끝.

  products()          -> [{'class_id', 'name', 'width_mm', 'depth_mm', 'height_mm', 'weight_g'}, ...]  A→D 순
  locs()              -> [{'name', 'x', 'y', 'z'}, ...]
  get_state()         -> state 행 dict
  update_state(patch) -> None   바꿀 열만 (행 전체를 덮어쓰지 않음)
================================================================
"""
import json
import os
import threading
from datetime import datetime, timezone

from box_database import database_config as cfg


class SupabaseStore:
    kind = 'supabase'

    def __init__(self, url, key):
        from supabase import create_client          # pip install supabase --break-system-packages
        try:
            try:
                from supabase import ClientOptions
            except ImportError:
                from supabase.lib.client_options import ClientOptions
            self.sb = create_client(url, key, options=ClientOptions(postgrest_client_timeout=cfg.DB_TIMEOUT_SEC))
        except TypeError:           # 버전에 따라 옵션 이름이 다르면 기본값으로
            self.sb = create_client(url, key)

    def products(self):
        return (self.sb.table(cfg.TABLE_PRODUCT)
                .select('class_id,name,width_mm,depth_mm,height_mm,weight_g')
                .order('class_id').execute().data)

    def locs(self):
        return self.sb.table(cfg.TABLE_LOC).select('name,x,y,z').execute().data

    def get_state(self):
        rows = (self.sb.table(cfg.TABLE_STATE).select('*')
                .eq('id', cfg.STATE_ID).limit(1).execute().data)
        if not rows:
            raise RuntimeError(f'{cfg.TABLE_STATE} 테이블에 id={cfg.STATE_ID} 행이 없음 '
                               f'(insert into {cfg.TABLE_STATE} (id) values (1))')
        return rows[0]

    def update_state(self, patch):
        rows = (self.sb.table(cfg.TABLE_STATE).update(patch)
                .eq('id', cfg.STATE_ID).execute().data)
        if not rows:
            raise RuntimeError(f'{cfg.TABLE_STATE} id={cfg.STATE_ID} 행이 없거나 쓰기 권한 없음 '
                               '(service_role 키인지 확인)')


_WRITER = {'MainThread': 'service', 'db_sync': 'control_status'}   # 누가 썼나: 관제 서비스 응답 / 관제 상태 동기화


class FileStore:
    """테이블마다 JSON 파일. 여러 스레드가 같은 객체를 써도 되게 잠금."""
    kind = 'file'

    def __init__(self, folder):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self._lock = threading.Lock()
        self.p_product = os.path.join(folder, f'{cfg.TABLE_PRODUCT}.json')
        self.p_loc = os.path.join(folder, f'{cfg.TABLE_LOC}.json')
        self.p_state = os.path.join(folder, f'{cfg.TABLE_STATE}.json')
        self.p_updates = os.path.join(folder, f'{cfg.TABLE_STATE}_updates.jsonl')
        if not os.path.exists(self.p_product):
            self._write(self.p_product, cfg.FILE_PRODUCT)
        if not os.path.exists(self.p_loc):
            self._write(self.p_loc, cfg.FILE_LOC)
        state = dict(cfg.FILE_STATE)
        try:
            state.update(self._read(self.p_state))
        except FileNotFoundError:
            pass
        self._state = state
        self._write(self.p_state, state)

    @staticmethod
    def _read(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)

    @staticmethod
    def _write(path, data):
        tmp = path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write('\n')
        os.replace(tmp, path)          # 쓰는 중에 열어 봐도 반쯤 쓴 파일이 안 보이게

    def products(self):
        rows = self._read(self.p_product)          # 매번 읽음 → 파일을 고치면 다음 작업부터 반영
        return sorted(rows, key=lambda r: r['class_id'])

    def locs(self):
        return self._read(self.p_loc)

    def get_state(self):
        with self._lock:
            return dict(self._state)

    def update_state(self, patch):
        with self._lock:
            changed = {k: v for k, v in patch.items() if self._state.get(k) != v}
            self._state.update(patch)
            self._write(self.p_state, self._state)
            if set(changed) - set(cfg.SYNC_ONLY_COLUMNS):          # heartbeat만 바뀐 줄은 기록 안 함
                line = {'at': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        'by': _WRITER.get(threading.current_thread().name, threading.current_thread().name),
                        'set': changed}
                with open(self.p_updates, 'a', encoding='utf-8') as f:
                    f.write(json.dumps(line, ensure_ascii=False) + '\n')


def make_stores(mode, fake_dir, log):
    """(서비스용 저장소, 동기화 스레드용 저장소). Supabase는 연결을 따로, 파일은 같은 객체."""
    mode = 'file' if mode == 'memory' else mode          # 예전 이름
    want = mode
    if mode == 'auto':
        want = 'supabase' if (cfg.SUPABASE_URL and cfg.SUPABASE_KEY) else 'file'
    if want == 'supabase':
        if not (cfg.SUPABASE_URL and cfg.SUPABASE_KEY):
            raise RuntimeError('store=supabase인데 SUPABASE_URL / SUPABASE_SERVICE_KEY가 없음')
        try:
            return (SupabaseStore(cfg.SUPABASE_URL, cfg.SUPABASE_KEY),
                    SupabaseStore(cfg.SUPABASE_URL, cfg.SUPABASE_KEY))
        except ImportError:
            if mode == 'supabase':
                raise RuntimeError('supabase 패키지 없음 (pip install supabase --break-system-packages)')
            log.error('[database] supabase 패키지 없음 → 파일 모드')
    elif want != 'file':
        raise RuntimeError(f"store='{mode}' 알 수 없음 (auto / supabase / file)")
    folder = os.path.expanduser(fake_dir or cfg.DEFAULT_FAKE_DIR)
    if mode == 'auto':
        log.warn(f'[database] Supabase 키 없음 → 파일 모드 ({folder}) — 웹 화면에는 안 보임')
    store = FileStore(folder)
    return store, store
