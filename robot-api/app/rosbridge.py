"""rosbridge v2: a reconnecting topic subscription and isolated service calls."""
from datetime import datetime, timezone, timedelta
import json
import logging
from threading import Event, Lock, Thread
import time
from uuid import uuid4

from websocket import create_connection, WebSocketTimeoutException

logger = logging.getLogger(__name__)


class RosbridgeError(RuntimeError):
    pass


class RosbridgeClient:
    def __init__(self, url: str, timeout: float = 8.0, stale_after: float = 10.0):
        self.url = url
        self.timeout = timeout
        self.stale_after = stale_after
        self._lock = Lock()
        self._stop = Event()
        self._thread = None
        self._socket = None
        self._online = False
        self._seen = 0.0
        self._data = {}
        self._stage = ''
        self._outcome = 'BEFORE'
        self._start_at = None
        self._started_at = None
        self._ended_at = None
        self._job_id = None
        self._reason = ''

    def start(self):
        self._stop.clear()
        self._thread = Thread(target=self._listen, name='rosbridge-status', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _accept_status(self, data: dict):
        stage = str(data.get('state') or '')
        now = datetime.now(timezone.utc)
        with self._lock:
            job_id = data.get('job_id')
            if job_id and job_id != self._job_id:
                self._job_id = job_id
                self._started_at = self._ended_at = self._start_at = None
                self._outcome, self._reason = 'BEFORE', ''
            if stage != self._stage:
                if stage in {'BEFORE', 'PREFLIGHT'}:
                    self._outcome, self._reason, self._started_at = 'BEFORE', '', None
                    self._ended_at = None
                if stage == 'STANDBY':
                    self._start_at = (now + timedelta(seconds=float(data.get('standby_sec') or 0))).isoformat()
                else:
                    self._start_at = None
                if stage == 'RUNNING':
                    self._outcome, self._started_at = 'RUNNING', now.isoformat()
                if stage in {'DONE', 'ERROR', 'STOPPED', 'CANCELED'}:
                    if self._started_at and self._ended_at is None:
                        self._ended_at = now.isoformat()
                    self._outcome = stage
            stop = data.get('stop')
            if stop:
                if self._started_at and self._ended_at is None:
                    self._ended_at = now.isoformat()
                self._outcome = 'ERROR' if stop[0] == 'CONTROL_ERROR' else 'STOPPED'
                self._reason = ' — '.join(str(item) for item in stop if item)
            elif data.get('reason'):
                self._reason = str(data['reason'])
            self._stage, self._data = stage, dict(data)
            self._seen = time.monotonic()
            self._online = True

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'stage': self._stage,
                'status': self._outcome,
                'reason': self._reason,
                'control_connected': self._online and bool(self._seen) and time.monotonic() - self._seen < self.stale_after,
                'start_at': self._start_at,
                'started_at': self._started_at,
                'ended_at': self._ended_at,
                'stop': self._data.get('stop'),
                'box_seq': self._data.get('box_seq', 0),
                'job_id': self._data.get('job_id', ''),
                'robot_monitor': self._data.get('robot_monitor', ''),
                'robot_hardware_mode': self._data.get('robot_hardware_mode', ''),
                'robot_connected': self._data.get('robot_connected') is True,
                'robot_state': self._data.get('robot_state'),
            }

    def _listen(self):
        while not self._stop.is_set():
            ws = None
            try:
                ws = create_connection(self.url, timeout=self.timeout, http_no_proxy=['*'])
                self._socket = ws
                ws.send(json.dumps({'op': 'subscribe', 'id': 'robot-api-status',
                                    'topic': '/control_status', 'type': 'std_msgs/msg/String'}))
                ws.settimeout(1)
                while not self._stop.is_set():
                    try:
                        message = json.loads(ws.recv())
                    except WebSocketTimeoutException:
                        continue
                    if message.get('op') == 'publish' and message.get('topic') == '/control_status':
                        data = json.loads(message['msg']['data'])
                        if not isinstance(data, dict) or not data.get('state'):
                            raise RosbridgeError('Invalid /control_status payload')
                        self._accept_status(data)
                    elif message.get('op') == 'status' and message.get('level') == 'error':
                        raise RosbridgeError(message.get('msg', 'Subscription failed'))
            except Exception as exc:
                if not self._stop.is_set():
                    logger.warning('rosbridge subscription disconnected: %s', exc)
            finally:
                with self._lock:
                    self._online = False
                self._socket = None
                if ws is not None:
                    ws.close()
            self._stop.wait(1)

    def call_service(self, service: str, args: dict) -> dict:
        """One fresh socket per command; stop never waits behind start/status."""
        request_id = uuid4().hex
        ws = None
        try:
            ws = create_connection(self.url, timeout=self.timeout, http_no_proxy=['*'])
            ws.send(json.dumps({'op': 'call_service', 'id': request_id,
                                'service': service, 'args': args, 'timeout': self.timeout}))
            deadline = time.monotonic() + self.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RosbridgeError('Service response timeout')
                ws.settimeout(remaining)
                message = json.loads(ws.recv())
                if message.get('id') != request_id:
                    continue
                if message.get('op') == 'service_response':
                    if message.get('result') is not True or not isinstance(message.get('values'), dict):
                        raise RosbridgeError(str(message.get('values') or 'Service call failed'))
                    return message['values']
                if message.get('op') == 'status' and message.get('level') == 'error':
                    raise RosbridgeError(message.get('msg', 'Service call failed'))
        except Exception as exc:
            # Never replay a command after a lost connection or timeout.
            raise RosbridgeError(f'rosbridge 서비스 응답을 확인할 수 없습니다: {service}. '
                                 '명령 처리 여부를 확인하세요.') from exc
        finally:
            if ws is not None:
                ws.close()
