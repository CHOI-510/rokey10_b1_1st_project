"""Read-only BASE-frame robot telemetry; never sends movement or gripper commands."""
from datetime import datetime, timezone
import json
import logging
import math
from threading import Event, Lock, Thread
import time
from uuid import uuid4

from websocket import create_connection, WebSocketTimeoutException

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
        self._joint_thread = None
        self._joint_socket = None
        self._joint_sample = None
        self._joint_seen = 0.0
        self._joint_online = False
        self._joint_stamp = None
        self._sample = None
        self._seen = 0.0
        self._online = False
        self._message = '실제 로봇 연결 대기'

    def start(self):
        self._stop.clear()
        self._thread = Thread(target=self._listen, name='rosbridge-telemetry', daemon=True)
        self._thread.start()
        self._joint_thread = Thread(target=self._listen_joints, name='rosbridge-joints', daemon=True)
        self._joint_thread.start()

    def close(self):
        self._stop.set()
        if self._joint_socket is not None:
            self._joint_socket.close()
        if self._joint_thread is not None:
            self._joint_thread.join(timeout=2)
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout + 1)

    def snapshot(self):
        real = self.real_robot_available()
        with self._lock:
            now = time.monotonic()
            tcp_age = now - self._seen if self._sample else None
            joint_age = now - self._joint_seen if self._joint_sample else None
            use_topic = self._joint_sample is not None and (
                (self._joint_online and joint_age < self.stale_after)
                or not self._sample or self._joint_seen >= self._seen)
            age = joint_age if use_topic else tcp_age
            online = self._joint_online if use_topic else self._online
            connected = bool(real and online and age is not None and age < self.stale_after)
            tcp_connected = bool(real and self._online and tcp_age is not None and tcp_age < self.stale_after)
            sample = dict(self._sample or {})
            if use_topic:
                sample.update(self._joint_sample)
            return {**sample, 'connected': connected,
                    'source': 'dsr_joint_states' if use_topic else 'dsr_read_services', 'frame': 'BASE',
                    'age_ms': round(age * 1000) if age is not None else None,
                    'tcp_connected': tcp_connected,
                    'tcp_age_ms': round(tcp_age * 1000) if tcp_age is not None else None,
                    'tcp_received_at': self._sample.get('received_at') if self._sample else None,
                    'message': ('실측 관절각·TCP 수신 중' if tcp_connected else '실측 관절각 수신 중 · TCP 조회 지연') if connected else
                    self._message if real else '실제 로봇 연결 대기 (real 모드 필요)'}

    def _accept_joints(self, message):
        names, positions = message.get('name'), message.get('position')
        if not isinstance(names, list) or not isinstance(positions, list) or len(names) != len(positions):
            raise ValueError('Invalid joint state arrays')
        indices = []
        for name in ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'):
            if names.count(name) != 1:
                raise ValueError('Missing/duplicate robot joint')
            indices.append(names.index(name))
        joints = finite_six([positions[index] for index in indices])
        stamp = message.get('header', {}).get('stamp', {})
        seconds = stamp.get('sec', 0) + stamp.get('nanosec', 0) / 1e9
        age = time.time() - seconds
        if seconds <= 0 or age >= self.stale_after or age < -self.stale_after or not self.real_robot_available():
            return
        with self._lock:
            if self._joint_stamp is not None and seconds <= self._joint_stamp:
                return
            self._joint_stamp = seconds
            self._joint_sample = {'joints_deg': list(map(math.degrees, joints)),
                                  'received_at': datetime.fromtimestamp(seconds, timezone.utc).isoformat()}
            self._joint_seen = time.monotonic() - max(0, age)
            self._joint_online = True

    def _listen_joints(self):
        # The driver's joint_state_broadcaster continues publishing during long
        # movement service calls. Never wait for TCP to update measured joints.
        namespace = self.service_ns.rsplit('/', 1)[0]
        topic = namespace + '/joint_states'
        while not self._stop.is_set():
            ws = None
            try:
                if not self.real_robot_available():
                    self._stop.wait(0.5)
                    continue
                ws = create_connection(self.url, timeout=2, http_no_proxy=['*'])
                self._joint_socket = ws
                ws.send(json.dumps({'op': 'subscribe', 'id': 'robot-api-joints', 'topic': topic,
                                    'type': 'sensor_msgs/msg/JointState', 'throttle_rate': 100, 'queue_length': 1}))
                ws.settimeout(0.5)
                while not self._stop.is_set() and self.real_robot_available():
                    try:
                        response = json.loads(ws.recv())
                    except WebSocketTimeoutException:
                        continue
                    if response.get('op') == 'publish' and response.get('topic') == topic:
                        self._accept_joints(response.get('msg', {}))
                    elif response.get('op') == 'status' and response.get('level') == 'error':
                        raise ValueError(response.get('msg', 'Joint subscription failed'))
            except Exception as error:
                if not self._stop.is_set():
                    logger.warning('Joint state subscription unavailable: %s', error)
            finally:
                with self._lock:
                    self._joint_online = False
                    self._joint_stamp = None
                self._joint_socket = None
                if ws is not None:
                    ws.close()
            self._stop.wait(0.5)

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
