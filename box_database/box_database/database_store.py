#!/usr/bin/env python3
"""
database_store.py — 저장소 두 가지 (같은 함수 4개)
================================================================
  SupabaseStore : 실제 Supabase (product, loc, inventory 테이블)
  FileStore     : Supabase 대신 JSON 파일 (DB 연동 전 시험·확인용, 기본 ~/box_logs/fake_db/)
                    product.json    A~D 4행   ← 처음엔 database_config 값으로 만들어짐, 직접 고쳐도 됨
                    loc.json        위치 5행  ← 같음 (작업 시작 때마다 다시 읽음)
                    inventory.json  재고 1행  ← DB 노드가 씀
                    inventory_updates.jsonl   재고를 쓴 기록 (한 줄 = 한 번: 시각, 바뀐 값) — 확인용
                  Supabase로 옮길 때: python3 scripts/db_to_sql.py 출력을 SQL Editor에 넣고 store를 바꾸면 끝.

  products()             -> [{'class_id', 'name', 'width_mm', 'depth_mm', 'height_mm', 'weight_g'}, ...]  A→D 순
  locs()                 -> [{'name', 'x', 'y', 'z'}, ...]
  get_inventory()        -> {'a', 'b', 'c', 'd', 'u', 'updated_at'}
  set_inventory(values)  -> None   {'a': .., 'b': .., 'c': .., 'd': .., 'u': ..} 최종값
================================================================
"""
import json
import os
import threading
from datetime import datetime, timezone

from box_database import database_config as cfg

INV_COLS = ('a', 'b', 'c', 'd', 'u')


def _now_iso():
    return datetime.now(timezone.utc).isoformat()


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

    def get_inventory(self):
        rows = (self.sb.table(cfg.TABLE_INVENTORY).select('a,b,c,d,u,updated_at')
                .eq('id', cfg.INVENTORY_ID).limit(1).execute().data)
        if not rows:
            raise RuntimeError(f'{cfg.TABLE_INVENTORY} 테이블에 id={cfg.INVENTORY_ID} 행이 없음 '
                               f'(insert into {cfg.TABLE_INVENTORY} (id) values (1))')
        return rows[0]

    def set_inventory(self, values):
        patch = {k: int(values[k]) for k in INV_COLS}
        patch['updated_at'] = _now_iso()
        rows = (self.sb.table(cfg.TABLE_INVENTORY).update(patch)
                .eq('id', cfg.INVENTORY_ID).execute().data)
        if not rows:
            raise RuntimeError(f'{cfg.TABLE_INVENTORY} id={cfg.INVENTORY_ID} 행이 없거나 쓰기 권한 없음 '
                               '(service_role 키인지 확인)')


class FileStore:
    """테이블마다 JSON 파일. 여러 스레드가 같은 객체를 써도 되게 잠금."""
    kind = 'file'

    def __init__(self, folder):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self._lock = threading.Lock()
        self.p_product = os.path.join(folder, f'{cfg.TABLE_PRODUCT}.json')
        self.p_loc = os.path.join(folder, f'{cfg.TABLE_LOC}.json')
        self.p_inv = os.path.join(folder, f'{cfg.TABLE_INVENTORY}.json')
        self.p_updates = os.path.join(folder, f'{cfg.TABLE_INVENTORY}_updates.jsonl')
        for path, default in ((self.p_product, cfg.FILE_PRODUCT), (self.p_loc, cfg.FILE_LOC),
                              (self.p_inv, cfg.FILE_INVENTORY)):
            if not os.path.exists(path):
                self._write(path, default)

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

    def _read_table(self, path, default):
        """매번 읽음 → 파일을 고치면 다음 작업부터 반영. 지워져 있으면 처음 값으로 다시 만듦"""
        with self._lock:
            if not os.path.exists(path):
                self._write(path, default)
            return self._read(path)

    def products(self):
        return sorted(self._read_table(self.p_product, cfg.FILE_PRODUCT), key=lambda r: r['class_id'])

    def locs(self):
        return self._read_table(self.p_loc, cfg.FILE_LOC)

    def get_inventory(self):
        row = dict(cfg.FILE_INVENTORY)
        row.update(self._read_table(self.p_inv, cfg.FILE_INVENTORY))
        return row

    def set_inventory(self, values):
        new = {k: int(values[k]) for k in INV_COLS}
        with self._lock:
            try:
                old = self._read(self.p_inv)
            except FileNotFoundError:
                old = dict(cfg.FILE_INVENTORY)
            self._write(self.p_inv, dict(old, **new, updated_at=_now_iso()))
            changed = {k: v for k, v in new.items() if old.get(k) != v}
            if changed:
                line = {'at': datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], 'set': changed}
                with open(self.p_updates, 'a', encoding='utf-8') as f:
                    f.write(json.dumps(line, ensure_ascii=False) + '\n')


def make_store(mode, fake_dir, log):
    mode = 'file' if mode == 'memory' else mode          # 예전 이름
    want = mode
    if mode == 'auto':
        want = 'supabase' if (cfg.SUPABASE_URL and cfg.SUPABASE_KEY) else 'file'
    if want == 'supabase':
        if not (cfg.SUPABASE_URL and cfg.SUPABASE_KEY):
            raise RuntimeError('store=supabase인데 SUPABASE_URL / SUPABASE_SERVICE_KEY가 없음')
        try:
            return SupabaseStore(cfg.SUPABASE_URL, cfg.SUPABASE_KEY)
        except ImportError:
            if mode == 'supabase':
                raise RuntimeError('supabase 패키지 없음 (pip install supabase --break-system-packages)')
            log.error('[database] supabase 패키지 없음 → 파일 모드')
    elif want != 'file':
        raise RuntimeError(f"store='{mode}' 알 수 없음 (auto / supabase / file)")
    folder = os.path.expanduser(fake_dir or cfg.DEFAULT_FAKE_DIR)
    if mode == 'auto':
        log.warn(f'[database] Supabase 키 없음 → 파일 모드 ({folder})')
    return FileStore(folder)
