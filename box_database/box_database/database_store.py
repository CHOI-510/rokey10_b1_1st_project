#!/usr/bin/env python3
"""
database_store.py — 저장소 두 가지 (같은 함수 4개)
================================================================
  SupabaseStore : 실제 Supabase (product, inventory, loc; 로그는 LogSync)
  FileStore     : Supabase 대신 JSON 파일 (DB 연동 전 시험·확인용)
                  폴더(기본 ~/box_logs/fake_db/)에 테이블마다 파일 하나:
                    product.json        A~D 4행    ← 처음엔 database_config 값으로 만들어짐, 직접 고쳐도 됨
                    loc.json            위치 5행   ← 같음 (작업 시작 때마다 다시 읽음)
                    state.json          1행        ← DB 노드가 씀 (Supabase state 행과 같은 모양)
                    state_updates.jsonl state에 쓴 내용 기록 (한 줄 = update 한 번: 시각, 누가, 바뀐 열)
                                        by: service = 관제 서비스(재고·작업 결과), control_status = 관제 단계·로봇
                                        → Supabase에 무엇이 언제 올라가는지 그대로 보임
                                        (2초마다 도는 heartbeat·control_seen_at만 바뀐 줄은 빼고 기록)
                  Supabase 모드는 product(name,len,weight), inventory, loc의 실제 스키마를 사용한다.

  products()          -> [{'class_id', 'name', 'width_mm', 'depth_mm', 'height_mm', 'weight_g'}, ...]  A→D 순
  locs()              -> [{'name', 'x', 'y', 'z'}, ...]
  get_state()         -> state 행 dict
  update_state(patch) -> None   바꿀 열만 (행 전체를 덮어쓰지 않음)
================================================================
"""
import json
import math
import os
import threading
from urllib.parse import urlencode
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from datetime import datetime, timezone

from box_database import database_config as cfg


def _location_from_row(row):
    """Map public.loc(id, coords) to the robot's internal location format."""
    name = str(row.get('id') or '').strip()
    if not name:
        raise ValueError('loc.id가 비어 있습니다.')
    if name.upper() in ('A', 'B', 'C', 'D', 'U'):
        name = name.upper()
    elif name.lower() == 'measure_zone' or name == '판별존':
        name = 'measure_zone'
    if name in ('A', 'B', 'C', 'D'):
        calibration = row.get('inventory_calibration')
        first = calibration.get('1') if isinstance(calibration, dict) else None
        signature = first.get('signature') if isinstance(first, dict) else None
        coords = signature.get('xy') if isinstance(signature, dict) else None
        if (not isinstance(coords, (list, tuple)) or len(coords) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in coords)):
            raise ValueError(f'loc[{name}].inventory_calibration의 1번 자리 signature.xy가 필요합니다.')
    else:
        coords = row.get('coords')
    if isinstance(coords, str):
        try:
            coords = json.loads(coords)
        except (ValueError, TypeError):
            # Also accept comma-separated coordinates, optionally in parentheses.
            coords = coords.strip().removeprefix('(').removesuffix(')').split(',')
    if isinstance(coords, dict):
        if 'x' not in coords or 'y' not in coords:
            raise ValueError(f'loc[{name}].coords에 x, y가 필요합니다.')
        coords = [coords['x'], coords['y'], coords.get('z')]
    if not isinstance(coords, (list, tuple)) or len(coords) not in (2, 3, 6):
        raise ValueError(f'loc[{name}].coords는 [x,y], [x,y,z] 또는 6개 pose 값이어야 합니다.')
    try:
        if any(isinstance(value, bool) for value in coords):
            raise ValueError('boolean coordinate')
        values = [float(value) if value is not None else None for value in coords]
        if any(value is not None and not math.isfinite(value) for value in values):
            raise ValueError('non-finite coordinate')
        if values[0] is None or values[1] is None or (len(values) == 6 and any(v is None for v in values)):
            raise ValueError('missing coordinate')
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(f'loc[{name}].coords에 유효한 숫자 좌표가 필요합니다.') from error
    return {'name': name, 'x': values[0], 'y': values[1],
            'z': values[2] if len(values) >= 3 else None}


def _number(value, label, positive=False):
    if isinstance(value, bool) or value is None or value == '':
        raise ValueError(f'{label}: 숫자가 필요합니다.')
    number = float(value)
    if not math.isfinite(number) or number < 0 or (positive and number == 0):
        raise ValueError(f'{label}: 유효한 숫자 범위가 아닙니다.')
    return number


def _inventory_rows(rows):
    mapped = {}
    for row in rows:
        category = str(row.get('loc_id') or '').strip().upper()
        if category not in 'ABCDU' or len(category) != 1:
            continue
        if category in mapped:
            raise ValueError(f'inventory.loc_id={category}: 중복 행')
        count = _number(row.get('cnt'), f'inventory[{category}].cnt')
        if not count.is_integer() or (category != 'U' and count > 6):
            raise ValueError(f'inventory[{category}].cnt: 정수 수량 범위 오류')
        mapped[category] = dict(row, cnt=int(count))
    missing = set('ABCDU') - set(mapped)
    if missing:
        raise ValueError(f'inventory에 {sorted(missing)} 행이 필요합니다.')
    return mapped


class SupabaseStore:
    """Existing product/inventory/loc schema; runtime metadata is not a DB table.

    Each HTTP request uses stdlib urllib, so no supabase SDK installation is needed.
    Inventory is always read from DB; there is no fallback to fake_db stock.
    """
    kind = 'supabase'

    def __init__(self, url, key):
        self.url = url.rstrip('/')
        self.key = key
        self._lock = threading.RLock()
        self._metadata = {k: v for k, v in cfg.FILE_STATE.items() if k not in ('a','b','c','d','u')}

    def _request(self, table, query, method='GET', patch=None):
        headers = {'apikey': self.key, 'Accept': 'application/json'}
        if not self.key.startswith('sb_secret_'):
            headers['Authorization'] = 'Bearer ' + self.key
        if patch is not None:
            headers.update({'Content-Type': 'application/json', 'Prefer': 'return=representation'})
        request = Request(self.url + '/rest/v1/' + table + '?' + urlencode(query),
                          method=method, headers=headers,
                          data=json.dumps(patch).encode() if patch is not None else None)
        try:
            with urlopen(request, timeout=cfg.DB_TIMEOUT_SEC) as response:
                rows = json.load(response)
        except HTTPError as error:
            try:
                detail = json.loads(error.read())
                message = str(detail.get('message', ''))
            except (ValueError, OSError):
                message = 'DB 요청 실패'
            raise RuntimeError(f'Supabase {table} HTTP {error.code}: {message}') from error
        if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
            raise ValueError(f'Supabase {table}: 행 목록 응답이 필요합니다.')
        return rows

    def _inventory(self):
        return _inventory_rows(self._request('inventory', {'select': 'loc_id,product_name,cnt'}))

    def products(self):
        rows = self._request(cfg.TABLE_PRODUCT, {'select': 'name,len,weight'})
        products = {}
        for row in rows:
            name = row.get('name')
            if name in products:
                raise ValueError(f'product.name={name}: 중복 행')
            products[name] = row
        inventory = self._inventory()
        result = []
        for category in 'ABCD':
            name = inventory[category].get('product_name')
            row = products.get(name)
            if row is None:
                raise ValueError(f'inventory[{category}].product_name={name}: product 행 없음')
            dimensions = row.get('len')
            if isinstance(dimensions, str):
                try:
                    dimensions = json.loads(dimensions)
                except ValueError:
                    dimensions = dimensions.strip().strip('()[]').split(',')
            if not isinstance(dimensions, (list, tuple)) or len(dimensions) != 3:
                raise ValueError(f'product[{name}].len: 가로,세로,높이 3개가 필요합니다.')
            width, depth, height = [_number(v, f'product[{name}].len', True) for v in dimensions]
            weight = 0.0 if row.get('weight') in (None, '') else _number(row['weight'], f'product[{name}].weight')
            result.append(dict(class_id=category, name=name, width_mm=width,
                               depth_mm=depth, height_mm=height, weight_g=weight))
        return result

    def locs(self):
        rows = self._request(cfg.TABLE_LOC, {'select': 'id,coords,inventory_calibration'})
        return [_location_from_row(row) for row in rows
                if str(row.get('id', '')).strip().upper() != 'U']

    def get_state(self):
        inventory = self._inventory()
        with self._lock:
            state = dict(self._metadata)
        state.update({category.lower(): row['cnt'] for category, row in inventory.items()})
        return state

    def update_state(self, patch):
        inventory_patch = {category.upper(): value for category, value in patch.items()
                           if category in ('a', 'b', 'c', 'd', 'u')}
        validated = {}
        for category, value in inventory_patch.items():
            count = _number(value, f'inventory[{category}].cnt')
            if not count.is_integer() or (category != 'U' and count > 6):
                raise ValueError(f'inventory[{category}].cnt: 정수 수량 범위 오류')
            validated[category] = int(count)
        with self._lock:
            if validated:
                existing = self._inventory()  # Validate all target rows before the first write.
                for category, count in validated.items():
                    if existing[category]['cnt'] == count:
                        continue
                    rows = self._request('inventory', {'loc_id': 'eq.' + category, 'select': 'loc_id,cnt'},
                                         method='PATCH', patch={'cnt': str(count)})
                    if len(rows) != 1 or rows[0].get('loc_id') != category or int(rows[0]['cnt']) != count:
                        raise RuntimeError(f'inventory[{category}] 갱신 결과를 확인할 수 없습니다.')
            self._metadata.update({k: v for k, v in patch.items() if k not in ('a','b','c','d','u')})


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
    """서비스와 동기화 스레드가 런타임 메타데이터를 공유한다."""
    mode = 'file' if mode == 'memory' else mode          # 예전 이름
    want = mode
    if mode == 'auto':
        want = 'supabase' if (cfg.SUPABASE_URL and cfg.SUPABASE_KEY) else 'file'
    if want == 'supabase':
        if not (cfg.SUPABASE_URL and cfg.SUPABASE_KEY):
            raise RuntimeError('store=supabase인데 SUPABASE_URL / SUPABASE_SERVICE_KEY가 없음')
        store = SupabaseStore(cfg.SUPABASE_URL, cfg.SUPABASE_KEY)
        return store, store  # Shared runtime metadata; HTTP requests themselves are independent.
    elif want != 'file':
        raise RuntimeError(f"store='{mode}' 알 수 없음 (auto / supabase / file)")
    folder = os.path.expanduser(fake_dir or cfg.DEFAULT_FAKE_DIR)
    if mode == 'auto':
        log.warn(f'[database] Supabase 키 없음 → 파일 모드 ({folder}) — 웹 화면에는 안 보임')
    store = FileStore(folder)
    return store, store
