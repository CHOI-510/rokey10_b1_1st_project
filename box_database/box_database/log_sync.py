"""Ship the durable JSONL journal to Supabase without blocking robot callbacks."""
import json
import fcntl
from urllib.error import HTTPError
from urllib.parse import urlencode
from datetime import datetime, timedelta, timezone
from pathlib import Path
import threading
from urllib.request import Request, urlopen

# Edit this list to choose which messages are persisted in the summary table.
KEYWORDS = ('성공', '완료', '적재', '판별', '탐지', '스캔', '시작', '취소', '정지', '실패', '오류', '재시도')
STATE_LABELS = {
    'READY': '대기 (시작 가능)', 'BEFORE': '작업전', 'PREFLIGHT': '사전점검',
    'STANDBY': '시작대기', 'RUNNING': '작업중', 'SCANNING': '스캔중',
    'SCAN_DONE': '스캔완료', 'JUDGING': '판별중', 'JUDGE_DONE': '판별완료',
    'PLACING': '적재중', 'PLACE_DONE': '적재완료', 'CLEARING': '측정존 비우기',
    'DONE': '작업완료', 'ERROR': '오류중단', 'STOPPED': '정지', 'CANCELED': '시작취소',
}


def event_row(event):
    message = str(event.get('message') or '')
    if not any(word in message for word in KEYWORDS):
        return None
    timestamp = event.get('received_at')
    if not timestamp:
        # Older journals use a Korean wall-clock stamp without a timezone.
        stamp = datetime.strptime(event['stamp'], '%Y-%m-%d %H:%M:%S').replace(tzinfo=timezone(timedelta(hours=9)))
        timestamp = (stamp + timedelta(microseconds=int(event.get('seq') or 0))).isoformat()
    stage = event.get('control_stage') or event.get('stage', '')
    state = STATE_LABELS.get(stage)
    if not state:
        # Historical files do not contain the concurrent controller stage.
        state = {'scan': '스캔중', 'judge': '판별중', 'place': '적재중',
                 'inventory_check': '재고점검'}.get(event.get('source'))
    if not state:
        state = {'JOB_START': '작업중', 'JOB_DONE': '작업완료', 'JOB_ERROR': '오류중단',
                 'JOB_STOPPED': '정지', 'JOB_CANCELED': '시작취소', 'PREFLIGHT': '사전점검'}.get(event.get('code'), '작업중')
    if stage == 'STOPPED' or (stage == 'READY' and event.get('code') in ('JOB_STOPPED', 'UI_COMMAND')):
        if 'EMERGENCY' in message or '비상' in message:
            state = '비상정지'
    return dict(timestamp=timestamp, job_id=str(event.get('job_id') or ''), message=message, state=state)


class LogSync:
    def __init__(self, folder, url, key, logger):
        self.folder = Path(folder)
        self.url, self.key, self.logger = url.rstrip('/'), key, logger
        self.offsets = {}
        self.wake = threading.Event()
        self.use_upsert = True
        self._lock_file = None

    def start(self):
        if not self.url or not self.key:
            self.logger.warn('[log_sync] Supabase 설정 없음 — JSONL만 저장; 연결 설정 후 재시작하면 재전송')
            return
        threading.Thread(target=self.run, name='job_log_sync', daemon=True).start()

    def upload(self, rows):
        headers = {'apikey': self.key, 'Content-Type': 'application/json',
                   'Prefer': 'resolution=ignore-duplicates,return=minimal'}
        if not self.key.startswith('sb_secret_'):
            headers['Authorization'] = 'Bearer ' + self.key
        req = Request(self.url + '/rest/v1/log?on_conflict=timestamp,job_id',
                      data=json.dumps(rows, ensure_ascii=False).encode(), method='POST',
                      headers=headers)
        if not self.use_upsert:
            return self.insert_missing(rows, headers)
        try:
            with urlopen(req, timeout=5) as response:
                if response.status not in (200, 201, 204):
                    raise RuntimeError('Log upload failed')
        except HTTPError as error:
            body = error.read().decode()
            if error.code == 400 and '42P10' in body:
                self.use_upsert = False
                self.insert_missing(rows, headers)
            else:
                raise

    def insert_missing(self, rows, headers):
        # Support the user's four-column table even without a UNIQUE constraint.
        # A process lock in run() ensures a single local writer.
        def identity(row):
            stamp = datetime.fromisoformat(row['timestamp'].replace('Z', '+00:00'))
            return stamp.astimezone(timezone.utc), row['job_id']
        missing = []
        for job in dict.fromkeys(row['job_id'] for row in rows):
            group = [row for row in rows if row['job_id'] == job]
            stamps = ','.join('"' + row['timestamp'] + '"' for row in group)
            query = urlencode({'select': 'timestamp,job_id', 'job_id': 'eq.' + job,
                               'timestamp': 'in.(' + stamps + ')'})
            req = Request(self.url + '/rest/v1/log?' + query, headers=headers)
            with urlopen(req, timeout=5) as response:
                existing = {identity(row) for row in json.load(response)}
            for row in group:
                key = identity(row)
                if key not in existing:
                    missing.append(row)
                    existing.add(key)
        if missing:
            insert_headers = dict(headers, Prefer='return=minimal')
            req = Request(self.url + '/rest/v1/log', method='POST', headers=insert_headers,
                          data=json.dumps(missing, ensure_ascii=False).encode())
            with urlopen(req, timeout=5) as response:
                if response.status not in (200, 201, 204):
                    raise RuntimeError('Log insert failed')

    def sync_file(self, path):
        offset = self.offsets.get(str(path), 0)
        if path.stat().st_size < offset:
            offset = 0
        rows = []
        scanned = 0
        with path.open('rb') as stream:
            stream.seek(offset)
            while scanned < 100:
                line = stream.readline()
                if not line or not line.endswith(b'\n'):
                    break  # Writer may still be appending; retry the complete line next time.
                event = json.loads(line)
                row = event_row(event)
                if row is not None:
                    rows.append(row)
                scanned += 1
                offset = stream.tell()
        if rows:
            self.upload(rows)
        self.offsets[str(path)] = offset  # Filtered lines advance too; failed uploads never do.
        return scanned

    def run(self):
        self._lock_file = (self.folder / '.supabase-log-sync.lock').open('a')
        try:
            fcntl.flock(self._lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.logger.warning('[log_sync] 이미 실행 중인 전송기가 있어 중복 실행을 종료합니다.')
            return
        failures = 0
        while True:
            self.wake.clear()
            try:
                backlog = False
                for path in sorted(self.folder.glob('*.jsonl'), key=lambda p: p.stat().st_mtime, reverse=True):
                    # Each file gets a turn; a corrupt/failed file never blocks newer jobs.
                    try:
                        backlog = self.sync_file(path) == 100 or backlog
                    except Exception as error:
                        failures += 1
                        if failures == 1 or failures % 30 == 0:
                            self.logger.warn(f'[log_sync] DB 저장 실패 ({type(error).__name__}, HTTP {getattr(error, "code", "-")}); log 테이블·서비스 키 확인. JSONL 유지 후 재시도')
                self.wake.wait(0.1 if backlog else 1.0)
            except Exception as error:
                self.logger.warn(f'[log_sync] {type(error).__name__}; 5초 후 재시도')
                self.wake.wait(5.0)


def main():
    """Independent file watcher for a controller that is already writing JSONL."""
    import argparse
    import logging
    import os
    parser = argparse.ArgumentParser(description='JSONL log folder → Supabase log')
    parser.add_argument('--folder', default=os.environ.get('BOX_LOG_DIR', '~/box_logs'))
    parser.add_argument('--once', action='store_true', help='Upload existing complete lines, then exit')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger('log_sync')
    folder = Path(args.folder).expanduser()
    if not folder.is_dir():
        parser.error('Log folder does not exist')
    url, key = os.environ.get('SUPABASE_URL', ''), os.environ.get('SUPABASE_SERVICE_KEY', '')
    if not url or not key:
        parser.error('Set SUPABASE_URL and SUPABASE_SERVICE_KEY in the server environment')
    sync = LogSync(folder, url, key, logger)
    if args.once:
        for path in sorted(folder.glob('*.jsonl')):
            while sync.sync_file(path) == 100:
                pass
        logger.info('Existing JSONL upload completed')
    else:
        logger.info('Watching JSONL folder: %s', folder)
        sync.run()


if __name__ == '__main__':
    main()
