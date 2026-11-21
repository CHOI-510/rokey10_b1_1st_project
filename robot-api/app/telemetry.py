"""Read-only BASE-frame robot telemetry; never sends movement or gripper commands."""
from datetime import datetime, timezone
import json
import logging
import math
from threading import Event, Lock, Thread
import time
from uuid import uuid4

from websocket import create_connection

logger = logging.getLogger(__name__)


def finite_six(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError('Expected six joint/pose values')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('Invalid joint/pose values')
    return list(map(float, values))


class TelemetryReader:
    def __init__(self, url, real_robot_available, service_ns='/dsr01/dsr_controller2',
                 period=0.1, timeout=1.0, stale_after=2.0):
        self.url = url
        self.real_robot_available = real_robot_available
        self.service_ns = service_ns.rstrip('/')
        self.period = max(0.05, period)
        self.timeout = timeout
        self.stale_after = stale_after
        self._stop = Event()
        self._lock = Lock()
        self._thread = None
        self._socket = None
        self._sample = None
        self._seen = 0.0
        self._online = False
        self._message = '실제 로봇 연결 대기'

    def start(self):
        self._stop.clear()
        self._thread = Thread(target=self._listen, name='rosbridge-telemetry', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout + 1)

    def snapshot(self):
        real = self.real_robot_available()
        with self._lock:
            age = (time.monotonic() - self._seen) if self._sample else None
            connected = bool(real and self._online and age is not None and age < self.stale_after)
            return {**(self._sample or {}), 'connected': connected,
                    'source': 'dsr_read_services', 'frame': 'BASE',
                    'age_ms': round(age * 1000) if age is not None else None,
                    'message': '실측 좌표 수신 중' if connected else
                    self._message if real else '실제 로봇 연결 대기 (real 모드 필요)'}

    def _read(self, ws, suffix, args, service_type):
        request_id = uuid4().hex
        ws.send(json.dumps({'op': 'call_service', 'id': request_id,
                            'service': f'{self.service_ns}/aux_control/{suffix}',
                            'type': f'dsr_msgs2/srv/{service_type}',
                            'args': args, 'timeout': self.timeout}))
        deadline = time.monotonic() + self.timeout
        while not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Robot telemetry response timeout')
            ws.settimeout(remaining)
            response = json.loads(ws.recv())
            if response.get('id') != request_id:
                continue
            if response.get('op') == 'status' and response.get('level') == 'error':
                raise ValueError(response.get('msg', 'Telemetry service error'))
            if response.get('op') == 'service_response':
                values = response.get('values')
                if response.get('result') is not True or not isinstance(values, dict) or values.get('success') is not True:
                    raise ValueError('Robot telemetry service failed')
                return values
        raise RuntimeError('Telemetry stopped')

    def _sample_once(self, ws):
        joints = finite_six(self._read(ws, 'get_current_posj', {}, 'GetCurrentPosj').get('pos'))
        task = self._read(ws, 'get_current_posx', {'ref': 0}, 'GetCurrentPosx').get('task_pos_info')
        if not isinstance(task, list) or not task or not isinstance(task[0], dict):
            raise ValueError('Invalid BASE TCP response')
        pose = finite_six(task[0].get('data', [])[:6])
        # Both reads must succeed in the same cycle. Never merge a new joint sample with an old TCP.
        if not self.real_robot_available():
            raise ValueError('Real robot disconnected during telemetry read')
        with self._lock:
            self._sample = {'joints_deg': joints, 'tcp_mm': pose[:3], 'orientation_deg': pose[3:],
                            'received_at': datetime.now(timezone.utc).isoformat()}
            self._seen = time.monotonic()
            self._online = True

    def _listen(self):
        while not self._stop.is_set():
            ws = None
            try:
                if not self.real_robot_available():
                    self._stop.wait(0.5)
                    continue
                ws = create_connection(self.url, timeout=self.timeout, http_no_proxy=['*'])
                self._socket = ws
                while not self._stop.is_set() and self.real_robot_available():
                    started = time.monotonic()
                    self._sample_once(ws)
                    self._stop.wait(max(0, self.period - (time.monotonic() - started)))
            except Exception as exc:
                if not self._stop.is_set():
                    logger.warning('Robot telemetry unavailable: %s', exc)
                with self._lock:
                    self._message = '좌표 수신 끊김 · 두산 조회 서비스/rosbridge 확인 필요'
            finally:
                with self._lock:
                    self._online = False
                self._socket = None
                if ws is not None:
                    ws.close()
            self._stop.wait(0.5)
