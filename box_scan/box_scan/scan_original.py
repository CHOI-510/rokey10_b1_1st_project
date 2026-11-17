#!/usr/bin/env python3
# X270~570/30mm 간격, Y240↔-90, Z35 지그재그 탐색(유효 보정 영역 300×300).
# 경로별 Fy 보정/CSV 유지. 접촉 정지 → 부채꼴 최대4점 Force 높이 탐지 → 0/45/90° 집기.
# 집기/배치 후 기준 자세 복원45deg/s. 설정값은 기능별 구역에서 변경.
# 높이/집기 접근: 목표 하향 힘25N, 접촉은 정지 기준 대비 Fz 증가8N.
# 집기 접근은 Fz만 접촉 판정, XYZ 합성 변화50N은 강제 중단. 조기 접촉/DI 실패는 각도 재시도.
# PLACE_XY에 놓기 → 수직 이탈/축정렬 → 감지 선 X/충돌 BASE Y-50으로 직접 복귀.
# 일반 대기0.2초, 힘 안정화/실제 닫힘 대기 유지. 시작/전체 종료에만 홈 이동.
# 재출발 첫15mm는 접촉 판정 제외. 기준 축정렬은 높이 탐지 사이클 종료 시 한번.
# 전체11선 완료 후 홈/종료. 오류 시 정지/힘 해제만 시도, 자동 안전정지 해제 없음.
from bisect import bisect_right
import argparse
import math
import time
import importlib
import sys
import types
import json
import csv
from datetime import datetime
from pathlib import Path

# 로봇 연결 · TCP
VERSION = 'scan-final-v17-grouped-settings'
ROBOT_MODEL = 'm0609'
SERVICE_PREFIX = '/dsr01/dsr_controller2'
TCP_OFFSET = [0.0, 0.0, 208.4, 0.0, 0.0, 0.0]
TCP_POSITION_TOL = 0.5
TCP_ROTATION_TOL = 0.005

# 일반 이동 속도 · 가속도 (Force/탐색 속도와 별도)
HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
HOME_VEL, HOME_ACC = 60.0, 80.0
APPROACH_VEL, APPROACH_ACC = 60.0, 80.0  # 관절 deg/s, deg/s²
LINEAR_VEL, LINEAR_ACC = 80.0, 100.0
TRANSFER_JOINT_VEL, TRANSFER_JOINT_ACC = 60.0, 80.0
GRIP_ROTATE_VEL, GRIP_ROTATE_ACC = 45.0, 90.0  # deg/s, deg/s²; 탐색 기울기 속도와 독립

# 탐색 경로 · 외력 보정 조건
X_LANES = tuple(270.0 + 30.0*i for i in range(11))
Y_START, Y_END = 240.0, -90.0
SCAN_Z = 35.0
SCAN_VEL, SCAN_ACC = 30.0, 40.0
FORCE_LIMIT = 1.5  # 보정 외력 절댓값 N. 1개 샘플 초과 즉시 정지 요청.
SAMPLE_PERIOD = 0.05  # 기준 데이터의 측정 목표 주기와 동일; 통신 지연은 별도
BACKWARD_TILT_DEG = 10.0

# 탐색 재개 · 일반 대기
SCAN_RESUME_Y_OFFSET_MM = -50.0  # 진행 방향과 무관한 BASE Y 보정
RESUME_IGNORE_MM = 15.0  # 재탐색 첫 구간만 제외; 다음 왕복부터 기존 판정
BETWEEN_LINES_WAIT = 0.2
SETTLE_SECONDS = 0.2
INITIAL_CLOSE_WAIT = 2.0  # 초기 실제 닫힘 대기

# 그리퍼 신호 · 집기 높이 · 재시도
GRIP_DI = 1
OPEN_DI = 2
Z0 = 0.0
MAX_OBJECT_HEIGHT_MM = 80.0
GRIP_HEIGHT_RATIO = 0.5
MIN_GRIP_ABOVE_Z0_MM = 2.0
GRIP_RETRY_ANGLES = (0.0, 45.0, 90.0)
GRIP_RETRY_LIFT_MM = 20.0
PICK_LIFT_MM = 20.0
PICK_HOLD_SECONDS = 0.2  # 집기 확인 후 유지 시간
PROBE_CLOSE_WAIT = 3.0

# 높이 탐지 지점 · 접촉 조건
PROBE_START_Z = 100.0
PROBE_POINTS = ((25.0, 0.0), (60.0, 20.0), (60.0, 0.0), (60.0, -20.0))
PROBE_CLEARANCE_MM = 20.0
PROBE_CONTACT_N = 8.0  # 기준 대비 양의 Fz 증가량(N)
PROBE_FORCE_TARGET_N = 25.0  # 목표 힘이며 하강 속도에 비례하지 않음
PROBE_SAFETY_N = 50.0

# 집기 접근 Force · 접촉 조건
GRIP_APPROACH_FORCE_TARGET_N = 25.0  # 집기 접근 전용 하향 목표 힘(N)
GRIP_APPROACH_CONTACT_N = 8.0  # 높이 탐지와 같은 Fz 증가 판정(N)
GRIP_APPROACH_SAFETY_N = 50.0
GRIP_APPROACH_Z_TOL_MM = 2.0

# 높이/집기 Force 공통 안정화 · 감시
PROBE_STIFFNESS = [3000.0, 3000.0, 500.0, 200.0, 200.0, 200.0]
PROBE_FORCE_RAMP_SECONDS = 0.3
PROBE_MODE_WAIT_SECONDS = 0.5
PROBE_BASELINE_SAMPLES = 10
PROBE_NOISE_MAX_N = 1.0
PROBE_XY_DRIFT_LIMIT_MM = 5.0
PROBE_TIMEOUT_REFERENCE_MM_S = 5.0
PROBE_SPEED_LOG_SECONDS = 0.5

# 물체 배치 좌표
PLACE_XY = [250.0, -180.0]
PLACE_Z_OFFSET_MM = 35.0  # 같은 작업대면 0; 놓는 곳 높이 차이(mm)

# 통신 주기 · 제한 시간 · 기록
POLL = 0.005
GRIP_POLL = 0.02
DISCOVERY_TIMEOUT = 10.0
RESPONSE_TIMEOUT = 3.0
DEFAULT_OUTPUT = "~/ROkEY_협동1/04_실험데이터"
RECORD_FILE = "~/rokey_scan_grip_last.json"

# 경로별 외력 보정 데이터 (탐색 조건과 함께 유지)
Y_MM = [-75.0, -65.0, -55.0, -45.0, -35.0, -25.0, -15.0, -5.0, 5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115.0, 125.0, 135.0, 145.0, 155.0, 165.0, 175.0, 185.0, 195.0, 205.0, 215.0, 225.0]
# 경로별 기준표: lane → X 위치·이동 방향·Y-75~225의 31개 기준값.
BASELINE_N = {1: {'x_mm': 270.0, 'direction': '-Y', 'values': [0.245229, 0.130973, -0.028782, -0.214828, -0.344738, -0.360215, -0.550376, -0.59634, -0.630192, -0.7819, -0.867324, -0.963784, -0.88293, -0.890583, -0.741088, -0.770544, -0.722376, -0.62101, -0.428877, -0.372254, -0.377577, -0.335227, -0.173405, 0.017183, -0.030802, 0.033672, 0.030339, 0.008024, 0.075072, -0.001348, 0.077456]}, 2: {'x_mm': 300.0, 'direction': '+Y', 'values': [-10.197535, -10.094623, -10.074717, -10.083933, -10.123622, -10.107575, -10.10033, -10.015296, -9.889341, -9.722886, -9.642506, -9.574125, -9.449228, -9.291629, -9.190596, -9.078696, -8.96171, -8.825521, -8.692802, -8.682957, -8.569397, -8.473716, -8.426546, -8.39214, -8.321907, -8.216789, -8.252018, -8.232602, -8.11525, -8.009222, -7.931779]}, 3: {'x_mm': 330.0, 'direction': '-Y', 'values': [0.254104, 0.057003, -0.101584, -0.244132, -0.369404, -0.532635, -0.613045, -0.695967, -0.788838, -0.831093, -0.900265, -0.873276, -0.916138, -0.926093, -0.827491, -0.781367, -0.794255, -0.715615, -0.724094, -0.590693, -0.490221, -0.537439, -0.437522, -0.398139, -0.265972, -0.105204, -0.091018, -0.035146, 0.009619, -0.003854, 0.132913]}, 4: {'x_mm': 360.0, 'direction': '+Y', 'values': [-8.847894, -8.775429, -8.771812, -8.780514, -8.856731, -8.914856, -8.924276, -8.915336, -8.884794, -8.854299, -8.774325, -8.711027, -8.65808, -8.536934, -8.409669, -8.303349, -8.220748, -8.162798, -8.107327, -8.023993, -7.952399, -7.832559, -7.759425, -7.793434, -7.663322, -7.649412, -7.564346, -7.56971, -7.432776, -7.424457, -7.458964]}, 5: {'x_mm': 390.0, 'direction': '-Y', 'values': [0.301119, 0.1401, 0.05206, -0.110163, -0.2773, -0.31611, -0.390599, -0.506453, -0.542558, -0.641085, -0.683901, -0.68168, -0.753572, -0.763488, -0.707247, -0.671726, -0.70483, -0.685077, -0.676607, -0.681109, -0.592026, -0.56271, -0.597977, -0.506726, -0.470589, -0.433527, -0.268801, -0.224429, -0.206825, -0.108377, 0.054889]}, 6: {'x_mm': 420.0, 'direction': '+Y', 'values': [-7.721559, -7.661393, -7.672774, -7.675754, -7.770079, -7.827328, -7.837034, -7.857528, -7.866993, -7.880909, -7.832489, -7.794841, -7.750539, -7.741952, -7.755232, -7.698988, -7.589477, -7.511084, -7.510827, -7.504656, -7.471784, -7.437414, -7.323136, -7.303363, -7.27083, -7.295119, -7.201806, -7.265506, -7.252905, -7.289883, -7.147368]}, 7: {'x_mm': 450.0, 'direction': '-Y', 'values': [0.394521, 0.261482, 0.155001, 0.039679, -0.108312, -0.161692, -0.256357, -0.370838, -0.45746, -0.510987, -0.53498, -0.546268, -0.667111, -0.610587, -0.588973, -0.596606, -0.636625, -0.657274, -0.655784, -0.722063, -0.68372, -0.652158, -0.649896, -0.657809, -0.61339, -0.55217, -0.536834, -0.404545, -0.357711, -0.246488, -0.199206]}, 8: {'x_mm': 480.0, 'direction': '+Y', 'values': [-6.966615, -6.871559, -6.859427, -6.909208, -6.993714, -7.093098, -7.128362, -7.153956, -7.220825, -7.233216, -7.292736, -7.250309, -7.274552, -7.305089, -7.33379, -7.324204, -7.248377, -7.262269, -7.200441, -7.182573, -7.155041, -7.186045, -7.190149, -7.213767, -7.155335, -7.214373, -7.160636, -7.117931, -7.227251, -7.259092, -7.304871]}, 9: {'x_mm': 510.0, 'direction': '-Y', 'values': [0.531526, 0.457469, 0.356875, 0.099523, 0.039713, -0.039063, -0.036554, -0.185594, -0.241396, -0.237574, -0.278915, -0.451596, -0.47834, -0.524926, -0.493822, -0.615744, -0.565349, -0.580723, -0.595775, -0.720687, -0.69127, -0.67942, -0.621204, -0.765188, -0.677723, -0.673271, -0.615588, -0.656207, -0.588391, -0.459702, -0.23275]}, 10: {'x_mm': 540.0, 'direction': '+Y', 'values': [-6.188388, -6.157399, -6.201363, -6.257414, -6.358558, -6.444615, -6.491468, -6.554995, -6.605913, -6.67191, -6.754816, -6.827662, -6.855099, -6.78136, -6.804725, -6.780684, -6.840294, -6.839786, -6.867203, -6.98156, -6.945819, -6.953968, -6.924202, -7.010123, -7.066073, -7.042885, -7.066318, -7.174191, -7.268263, -7.237265, -7.336822]}, 11: {'x_mm': 570.0, 'direction': '-Y', 'values': [0.779139, 0.629582, 0.596794, 0.439062, 0.317208, 0.155054, 0.055427, 0.042208, -0.048524, -0.149397, -0.257825, -0.315558, -0.454075, -0.437356, -0.525823, -0.601567, -0.575093, -0.61943, -0.695359, -0.733475, -0.742573, -0.783199, -0.767537, -0.798939, -0.763556, -0.667476, -0.680675, -0.711594, -0.741026, -0.687716, -0.53889]}}


def validate_baseline():
    if set(BASELINE_N) != set(range(1, len(X_LANES)+1)):
        raise ValueError('Baseline must cover every configured lane')
    for lane, x in enumerate(X_LANES, 1):
        entry = BASELINE_N[lane]
        direction = '-Y' if lane % 2 else '+Y'
        if entry['x_mm'] != x or entry['direction'] != direction:
            raise ValueError(f'Lane {lane} baseline X/direction mismatch')
        values = entry['values']
        if len(values) != len(Y_MM) or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError(f'Lane {lane} requires 31 finite baseline values')


def baseline_fy(y_mm, direction, lane):
    entry = BASELINE_N.get(lane)
    if entry is None or entry['direction'] != direction:
        raise ValueError(f'No matching baseline: lane={lane}, direction={direction}')
    y = float(y_mm)
    if not math.isfinite(y):
        raise ValueError('Y must be finite')
    if not Y_MM[0] <= y <= Y_MM[-1]:
        return None
    values = entry['values']
    i = min(bisect_right(Y_MM, y)-1, len(Y_MM)-2)
    fraction = (y-Y_MM[i])/(Y_MM[i+1]-Y_MM[i])
    return values[i] + fraction*(values[i+1]-values[i])


class ContactDetected(Exception):
    def __init__(self, detection):
        self.detection = detection
        super().__init__(f"CONTACT {detection['direction']}: stopped={detection['stopped_pose'][:3]}, residual={detection['residual_n']:.3f}N")



class ProbeMiss(Exception):
    """정지·힘 해제가 확인된 정상 탐지에서 유효 높이를 찾지 못함."""
    def __init__(self, reason, pose=None):
        self.reason, self.pose = reason, pose
        super().__init__(reason)

def zyz_rotation(angles):
    a, b, c = map(math.radians, angles)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb],
            [sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb],
            [-sb*cc, sb*sc, cb]]


def offset_matches(flange, tcp):
    rf, rt, expected_rotation = zyz_rotation(flange[3:6]), zyz_rotation(tcp[3:6]), zyz_rotation(TCP_OFFSET[3:6])
    expected_position = [flange[i] + sum(rf[i][j]*TCP_OFFSET[j] for j in range(3)) for i in range(3)]
    target_rotation = [[sum(rf[i][k]*expected_rotation[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return (math.dist(tcp[:3], expected_position) <= TCP_POSITION_TOL
            and max(abs(rt[i][j]-target_rotation[i][j]) for i in range(3) for j in range(3)) <= TCP_ROTATION_TOL)



class BoundedServiceClient:
    def __init__(self, client):
        self.client = client

    def wait_for_service(self, timeout_sec=None):
        if not self.client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            raise TimeoutError('Service unavailable: ' + self.client.srv_name)
        return True

    def __getattr__(self, name):
        return getattr(self.client, name)


class Robot:
    def __init__(self, prefix):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from dsr_msgs2 import srv
        self.ros, self.srv, self.prefix = rclpy, srv, prefix.rstrip('/')
        self.node = self.executor = None
        self.motion_requested = False
        self.force_requested = False
        self.compliance_requested = False
        rclpy.init()
        try:
            if not self.prefix.startswith('/') or not self.prefix.strip('/'):
                raise ValueError('service-prefix must be an absolute ROS namespace')
            if 'DSR_ROBOT2' in sys.modules:
                raise RuntimeError('Run standalone: DSR_ROBOT2 already imported in this process')
            routes = {
                '_ros2_get_digital_input': 'io/get_ctrl_box_digital_input',
                '_ros2_set_ref_coord': 'motion/set_ref_coord',
                '_ros2_task_compliance_ctrl': 'force/task_compliance_ctrl',
                '_ros2_set_desired_force': 'force/set_desired_force',
                '_ros2_release_force': 'force/release_force',
                '_ros2_release_compliance_ctrl': 'force/release_compliance_ctrl',
                '_ros2_movej': 'motion/move_joint',
                '_ros2_movejx': 'motion/move_jointx',
                '_ros2_set_digital_output': 'io/set_ctrl_box_digital_output',
                '_ros2_movel': 'motion/move_line',
                '_ros2_check_motion': 'motion/check_motion',
                '_ros2_get_current_posj': 'aux_control/get_current_posj',
                '_ros2_get_current_posx': 'aux_control/get_current_posx',
                '_ros2_get_current_tool_flange_posx': 'aux_control/get_current_tool_flange_posx',
                '_ros2_get_tool_force': 'aux_control/get_tool_force',
                '_ros2_config_create_tcp': 'tcp/config_create_tcp',
                '_ros2_set_current_tcp': 'tcp/set_current_tcp',
                '_ros2_get_current_tcp': 'tcp/get_current_tcp',
                '_ros2_set_robot_mode': 'system/set_robot_mode',
                '_ros2_get_robot_mode': 'system/get_robot_mode',
            }
            import DR_init
            setattr(DR_init, '__dsr__id', self.prefix.strip('/').split('/')[0])
            setattr(DR_init, '__dsr__model', ROBOT_MODEL)
            # Some DSR_ROBOT2 versions use bare paths; others prepend
            # "dsr_controller2/". Resolve both to the requested service prefix.
            controller = self.prefix.rstrip('/').split('/')[-1]
            remappings = []
            for path in routes.values():
                remappings.append((path, self.prefix + '/' + path))
                remappings.append(('dsr_controller2/' + path, self.prefix + '/' + path))
                if controller != 'dsr_controller2':
                    remappings.append((controller + '/' + path, self.prefix + '/' + path))
            ros_args = ['--ros-args']
            for source, destination in remappings:
                ros_args.extend(['-r', source + ':=' + destination])
            self.node = rclpy.create_node(
                'm0609_scan_final', namespace=self.prefix,
                cli_args=ros_args, use_global_arguments=False)
            setattr(DR_init, '__dsr__node', self.node)
            self.executor = SingleThreadedExecutor(context=self.node.context)
            self.executor.add_node(self.node)
            self.dsr = importlib.import_module('DSR_ROBOT2')
            required = ('get_digital_input', 'set_ref_coord', 'task_compliance_ctrl',
                        'set_desired_force', 'release_force', 'release_compliance_ctrl', 'amovej', 'amovel', 'amovejx', 'set_digital_output', 'get_current_posj', 'get_current_posx',
                        'get_current_tool_flange_posx', 'get_tool_force', 'check_motion',
                        'add_tcp', 'set_tcp', 'get_tcp',
                        'set_robot_mode', 'get_robot_mode', 'wait')
            missing = [name for name in required if not callable(getattr(self.dsr, name, None))]
            if missing:
                raise RuntimeError('Installed DSR_ROBOT2 missing: ' + ', '.join(missing))
            self._original_api_clients = {}
            for attr, path in routes.items():
                client = getattr(self.dsr, attr, None)
                if client is None:
                    matches = [(key, value) for key, value in vars(self.dsr).items()
                               if key.startswith('_ros2_')
                               and isinstance(getattr(value, 'srv_name', None), str)
                               and value.srv_name.rstrip('/').endswith('/' + path)]
                    if len(matches) != 1:
                        raise RuntimeError(f'DSR client not found or ambiguous: {path}')
                    attr, client = matches[0]
                # Jazzy Client.srv_name retains the supplied (possibly relative)
                # spelling. Compare the actual expanded/remapped service name.
                raw = getattr(client, 'srv_name', None)
                resolved = (self.node.resolve_service_name(raw)
                            if isinstance(raw, str) else None)
                expected = self.prefix + '/' + path
                if client is None or resolved != expected:
                    raise RuntimeError(
                        f'DSR service mismatch: variable={attr}, raw={raw!r}, '
                        f'resolved={resolved!r}, expected={expected!r}')
                self._original_api_clients[attr] = client
                setattr(self.dsr, attr, BoundedServiceClient(client))
            self._original_dsr_rclpy = self.dsr.rclpy
            self.dsr.rclpy = types.SimpleNamespace(**vars(rclpy))
            self.dsr.rclpy.spin_until_future_complete = self._spin_api_future
        except BaseException:
            self.shutdown()
            raise

    def _spin_api_future(self, node, future, *args, **kwargs):
        if node is not self.node:
            raise RuntimeError('Unexpected DSR_ROBOT2 node')
        self.executor.spin_until_future_complete(future, timeout_sec=RESPONSE_TIMEOUT)
        if not future.done():
            future.cancel()  # does not cancel physical motion; main requests STOP
            raise TimeoutError('DSR_ROBOT2 service response timeout')
        result = future.result()
        if result is None or (hasattr(result, 'success') and not result.success):
            raise RuntimeError('DSR_ROBOT2 service failed/rejected')

    def command(self, name, *args, **kwargs):
        result = getattr(self.dsr, name)(*args, **kwargs)
        if type(result) is not int or result != 0:
            raise RuntimeError(f'{name} failed: {result!r}')

    def tcp_name(self):
        name = self.dsr.get_tcp()
        if not isinstance(name, str):
            raise RuntimeError('Invalid current TCP name')
        return name

    def robot_mode(self):
        mode = self.dsr.get_robot_mode()
        if mode not in (0, 1):
            raise RuntimeError('Expected manual/automatic robot mode')
        return mode

    def log(self, message):
        self.node.get_logger().info(message)

    def check_setup(self):
        # Reuse the vendor API clients, including move_jointx entry motion.
        for client in self._original_api_clients.values():
            if not client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
                raise TimeoutError('Service unavailable: ' + client.srv_name)
        # Keep the proven direct STOP service; create it only once before motion.
        self.stop_client = self.node.create_client(
            self.srv.MoveStop, self.prefix + '/motion/move_stop')
        if not self.stop_client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT):
            raise TimeoutError('STOP service unavailable')
        if self.state() != 0:
            raise RuntimeError('Robot already moving; no sequence started')
        self.log(f'Current TCP={self.tcp_name()!r}; TCP transform will be verified after start.')

    def flange_pose(self):
        p = list(self.dsr.get_current_tool_flange_posx(ref=self.dsr.DR_BASE))
        if len(p) != 6 or not all(math.isfinite(x) for x in p):
            raise RuntimeError('Invalid flange pose')
        return p

    def tcp_offset_correct(self):
        if self.state() != 0:
            raise RuntimeError('TCP configuration requires stationary robot')
        flange, tcp = self.flange_pose(), self.pose()
        if self.state() != 0:
            raise RuntimeError('Robot moved during TCP check')
        matched = offset_matches(flange, tcp)
        self.log(f'TCP transform check: flange={flange}, TCP={tcp}, matches={matched}')
        return matched

    def set_mode(self, mode):
        self.command('set_robot_mode', mode)
        deadline = time.monotonic() + 5.0
        while self.robot_mode() != mode:
            if time.monotonic() >= deadline:
                raise TimeoutError('Robot mode change not confirmed')
            time.sleep(POLL)

    def select_tcp_confirmed(self, name):
        self.command('set_tcp', name)
        deadline = time.monotonic() + RESPONSE_TIMEOUT
        while self.tcp_name() != name:
            if time.monotonic() >= deadline:
                raise TimeoutError(f'TCP selection not confirmed: {name}')
            time.sleep(0.05)

    def configure_tcp_from_zero(self):
        """0-TCP와 대상 TCP를 같은 get_current_posx로 비교해 전체 변환 검증."""
        import uuid
        token = uuid.uuid4().hex[:8]
        zero_name, target_name = 'RG2_ZERO_' + token, 'RG2_SCAN_' + token
        self.set_mode(0)
        joints = list(self.dsr.get_current_posj())
        self.command('add_tcp', zero_name, [0.0] * 6)
        self.select_tcp_confirmed(zero_name)
        # 다른 API의 자세 표현에 의존하지 않고 실제 0-TCP 자세를 기준으로 사용.
        deadline = time.monotonic() + RESPONSE_TIMEOUT
        while True:
            zero_pose, flange = self.pose(), self.flange_pose()
            if math.dist(zero_pose[:3], flange[:3]) <= TCP_POSITION_TOL:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Zero-TCP position could not be verified; no motion started')
            time.sleep(0.05)
        self.command('add_tcp', target_name, TCP_OFFSET[:])
        self.select_tcp_confirmed(target_name)
        deadline = time.monotonic() + RESPONSE_TIMEOUT
        while True:
            actual_joints = list(self.dsr.get_current_posj())
            if len(joints) != 6 or len(actual_joints) != 6 or not all(math.isfinite(v) for v in (*joints, *actual_joints)):
                raise RuntimeError('Invalid joints during stationary TCP verification')
            if self.state() != 0 or max(abs(a-b) for a,b in zip(joints,actual_joints)) > 0.1:
                raise RuntimeError('Robot moved during zero-TCP verification')
            tcp = self.pose()
            if offset_matches(zero_pose, tcp):
                self.log(f'TCP verified using zero-TCP reference: zero={zero_pose}, TCP={tcp}')
                self.log('TCP selected: ' + target_name)
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'TCP transform mismatch: zero={zero_pose}, TCP={tcp}, offset={TCP_OFFSET}; no motion started')
            time.sleep(0.05)

    def ensure_tcp(self):
        original_mode = self.robot_mode()
        try:
            if not self.tcp_offset_correct():
                self.log('Verifying TCP with zero-TCP reference: ' + str(TCP_OFFSET))
                self.configure_tcp_from_zero()
            else:
                self.log('TCP offset already correct; keeping existing TCP registration')
            if self.robot_mode() != 1:
                self.set_mode(1)
        except BaseException:
            try:
                self.set_mode(original_mode)
            except Exception as exc:
                self.log('Could not restore previous mode: ' + str(exc))
            raise

    def state(self):
        state = self.dsr.check_motion()
        if state not in (0, 1, 2):
            raise RuntimeError('Unexpected motion state: ' + str(state))
        return state

    def pose(self):
        result = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        if not isinstance(result, (tuple, list)) or len(result) != 2:
            raise RuntimeError('Invalid TCP pose response')
        position, solution = result  # DSR_ROBOT2 returns (posx, solution_space)
        if not isinstance(position, (tuple, list)) or len(position) != 6:
            raise RuntimeError('Invalid TCP pose')
        pose = list(position)
        if not all(math.isfinite(x) for x in pose):
            raise RuntimeError('Non-finite TCP pose')
        return pose

    def force_y(self):
        values = self.dsr.get_tool_force(ref=self.dsr.DR_BASE)
        if not isinstance(values, (list, tuple)) or len(values) != 6 or not all(math.isfinite(x) for x in values):
            raise RuntimeError('Invalid external tool force')
        return float(values[1])

    def wait_motion(self, timeout=180.0):
        deadline = time.monotonic() + timeout
        while self.state() != 0:
            if time.monotonic() >= deadline:
                raise TimeoutError('Motion completion timeout')
            time.sleep(POLL)

    def stop(self):
        request = self.srv.MoveStop.Request()
        request.stop_mode = 1  # DR_QSTOP
        future = self.stop_client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=RESPONSE_TIMEOUT)
        if not future.done():
            future.cancel()
            raise TimeoutError('STOP service response timeout')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('STOP request rejected')
        self.wait_motion(timeout=5.0)
        self.motion_requested = False

    def home(self):
        self.log('HOME: ' + str(HOME))
        self.motion_requested = True
        self.command('amovej', HOME[:], vel=HOME_VEL, acc=HOME_ACC,
                     mod=self.dsr.DR_MV_MOD_ABS)
        started = time.monotonic()
        while True:
            if self.state() == 0:
                actual = list(self.dsr.get_current_posj())
                if len(actual) == 6 and all(math.isfinite(v) for v in actual) and max(abs(a-b) for a,b in zip(actual,HOME)) <= 1.0:
                    break
                if time.monotonic()-started > 2.0:
                    raise RuntimeError('Stopped before reaching HOME')
            if time.monotonic()-started >= 180.0:
                raise TimeoutError('HOME completion timeout')
            time.sleep(POLL)
        self.motion_requested = False
        self.log('HOME reached')

    def close_gripper(self):
        self.command('set_digital_output', 2, 0)
        self.command('set_digital_output', 1, 1)
        self.log('Gripper CLOSE commanded: DO2=0, DO1=1')

    def joint_to_pose(self, target, speed, acceleration):
        _, solution = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        self.log(f'JOINT entry XYZ={target[:3]}, speed={speed:g}deg/s, acc={acceleration:g}deg/s2')
        self.motion_requested = True
        self.command('amovejx', list(map(float, target)), vel=speed, acc=acceleration,
                     sol=solution, ref=self.dsr.DR_BASE, mod=self.dsr.DR_MV_MOD_ABS)
        # 초기 idle 상태를 완료로 오판하지 않도록 정지와 목표 도달을 함께 확인한다.
        deadline = time.monotonic() + 180.0
        while True:
            if self.state() == 0 and math.dist(self.pose()[:3], target[:3]) <= 1.0:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('Joint entry completion timeout')
            time.sleep(POLL)
        self.motion_requested = False

    def scan_line(self, target, direction, recorder, lane, phase='scan', ignore_start_mm=0.0):
        start_y = self.pose()[1]
        sign = 1.0 if direction == '+Y' else -1.0
        self.log(f'SCAN lane={lane} {direction}: target={target[:3]}, ignore={ignore_start_mm:g}mm')
        self.motion_requested = True
        self.command('amovel', list(map(float, target)),
                     vel=[SCAN_VEL, 10.0], acc=[SCAN_ACC, 20.0],
                     ref=self.dsr.DR_BASE, mod=self.dsr.DR_MV_MOD_ABS)
        started = next_sample = time.monotonic()
        while True:
            status = self.state()
            if time.monotonic() >= next_sample:
                stamp = time.monotonic()
                fy = self.force_y()
                pose = self.pose()
                offset = baseline_fy(pose[1], direction, lane) if phase == 'scan' else None
                residual = None if offset is None else fy-offset
                enabled = status != 0 and residual is not None and sign*(pose[1]-start_y) >= ignore_start_mm
                contact = enabled and abs(residual) >= FORCE_LIMIT
                recorder.add(stamp, lane, phase, direction, pose, fy, offset,
                             residual, status, enabled, contact)
                if contact:
                    self.stop()
                    stopped = self.pose()
                    raise ContactDetected({
                        'lane': lane, 'direction': direction, 'sample_pose': pose,
                        'stopped_pose': stopped, 'fy_n': fy, 'baseline_n': offset,
                        'residual_n': residual, 'detected_unix_s': time.time(),
                    })
                next_sample = stamp + SAMPLE_PERIOD
            if self.state() == 0:
                actual = self.pose()
                if math.dist(actual[:3], target[:3]) <= 1.0:
                    self.motion_requested = False
                    return
                if time.monotonic()-started > 2.0:
                    raise RuntimeError(f'Linear segment stopped: actual={actual[:3]}, target={target[:3]}')
            if time.monotonic()-started >= 90.0:
                raise TimeoutError('Linear segment completion timeout')
            time.sleep(POLL)

    def release_probe_control(self):
            errors = []
            for flag, command in (('force_requested', 'release_force'),
                                  ('compliance_requested', 'release_compliance_ctrl')):
                if getattr(self, flag, False):
                    try:
                        if command == 'release_force':
                            self.command(command, time=0.0)
                        else:
                            self.command(command)
                        setattr(self, flag, False)
                    except Exception as exc:
                        errors.append(f'{command}: {exc}')
            if errors:
                raise RuntimeError('Force/compliance release not confirmed: ' + '; '.join(errors))

    def stop_and_release(self):
            """정지 실패 여부와 무관하게 힘·순응 해제를 시도."""
            errors = []
            if self.motion_requested:
                try:
                    self.stop()
                except BaseException as exc:
                    errors.append(f'Quick Stop: {exc}')
            try:
                self.release_probe_control()
            except BaseException as exc:
                errors.append(str(exc))
            if errors:
                raise RuntimeError('Cleanup not confirmed: ' + '; '.join(errors))

    def shutdown(self):
        for attr, client in getattr(self, '_original_api_clients', {}).items():
            setattr(self.dsr, attr, client)
        if hasattr(self, '_original_dsr_rclpy'):
            self.dsr.rclpy = self._original_dsr_rclpy
        try:
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=1.0)
                self.executor = None
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
                    self.node = None
            finally:
                self.ros.try_shutdown()


class Recorder:
    FIELDS = ('elapsed_s', 'segment_elapsed_s', 'actual_interval_s', 'lane',
              'phase', 'direction', 'x_mm', 'y_mm', 'z_mm', 'fy_raw_n',
              'fy_baseline_n', 'fy_corrected_n', 'motion_state',
              'detection_enabled', 'contact', 'speed_mm_s', 'acc_mm_s2',
              'baseline_loaded')

    def __init__(self, root):
        self.directory = Path(root).expanduser() / (
            f'raster_v{SCAN_VEL:g}_a{SCAN_ACC:g}_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.rows = []
        self.origin = self.segment_origin = self.previous = None
        self.index = 0

    def add(self, stamp, lane, phase, direction, pose, fy, baseline,
            residual, status, enabled, contact):
        if self.origin is None:
            self.origin = stamp
        if self.segment_origin is None:
            self.segment_origin = stamp
        interval = '' if self.previous is None else stamp-self.previous
        self.rows.append((stamp-self.origin, stamp-self.segment_origin, interval,
                          lane, phase, direction, *pose[:3], fy,
                          '' if baseline is None else baseline,
                          '' if residual is None else residual, status,
                          int(enabled), int(contact), SCAN_VEL, SCAN_ACC, int(bool(BASELINE_N))))
        self.previous = stamp

    def flush(self):
        if not self.rows:
            return
        name = f'{self.index+1:02d}_lane_{self.rows[0][3]:02d}_{self.rows[0][4]}.csv'
        target = self.directory / name
        # Atomic publish; retain memory on failure, no overwrite of completed CSV.
        temporary = target.with_suffix('.csv.partial')
        with temporary.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(self.FIELDS)
            writer.writerows(self.rows)
        if target.exists():
            raise FileExistsError(target)
        temporary.rename(target)
        self.index += 1
        self.rows.clear()
        self.segment_origin = self.previous = None


def backward_orientation(reference, direction):
    if direction not in ('-Y', '+Y'):
        raise ValueError('Direction must be -Y or +Y')
    if not math.isfinite(BACKWARD_TILT_DEG):
        raise ValueError('Tilt must be finite')
    if BACKWARD_TILT_DEG == 0:
        return list(reference)
    r = zyz_rotation(reference)
    if r[2][2] > -0.98:
        raise RuntimeError('Backward tilt requires an approximately downward tool Z reference')
    angle = math.radians(BACKWARD_TILT_DEG if direction == '-Y' else -BACKWARD_TILT_DEG)
    c, t = math.cos(angle), math.sin(angle)
    rx = [[1,0,0],[0,c,-t],[0,t,c]]
    q = [[sum(rx[i][k]*r[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    b = math.acos(max(-1.0, min(1.0, q[2][2])))
    if abs(math.sin(b)) < 1e-8:
        # Equivalent ZYZ representation at the pole; preserve the rotation matrix.
        a = math.atan2(-q[0][1], -q[0][0]) if q[2][2] < 0 else math.atan2(q[1][0], q[0][0])
        return [math.degrees(a), math.degrees(b), 0.0]
    return [math.degrees(math.atan2(q[1][2], q[0][2])), math.degrees(b),
            math.degrees(math.atan2(q[2][1], -q[2][0]))]


def set_scan_orientation(robot, angles, angular_vel=5.0, angular_acc=10.0):
    target = robot.pose()
    expected = zyz_rotation(angles)
    current = zyz_rotation(target[3:])
    if max(abs(current[i][j]-expected[i][j]) for i in range(3) for j in range(3)) <= 1e-6:
        return
    target[3:6] = angles
    robot.motion_requested = True
    robot.command('amovel', target, vel=[5.0, angular_vel], acc=[10.0, angular_acc],
                  ref=robot.dsr.DR_BASE, mod=robot.dsr.DR_MV_MOD_ABS)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if robot.state() == 0:
            actual = robot.pose()
            r = zyz_rotation(actual[3:])
            error = max(abs(r[i][j]-expected[i][j]) for i in range(3) for j in range(3))
            if math.dist(actual[:3],target[:3]) <= 1.0 and error <= 0.005:
                robot.motion_requested = False
                return
        time.sleep(POLL)
    raise TimeoutError('Tilt completion not confirmed')




def feedback(robot, grip_di, open_di):
    values = tuple(robot.dsr.get_digital_input(i) for i in (grip_di, open_di))
    if any(type(v) is not int or v not in (0, 1) for v in values):
        raise RuntimeError(f'Invalid DI feedback: {values!r}')
    return values


def wait_feedback(robot, args, expected, timeout):
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        if feedback(robot, args.grip_di, args.open_di) == expected:
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= 0.15:
                return True
        else:
            stable_since = None
        time.sleep(GRIP_POLL)
    return False


def open_gripper(robot, args):
    robot.command('set_digital_output', 1, 0)
    robot.command('set_digital_output', 2, 1)
    if not wait_feedback(robot, args, (0, 1), args.timeout):
        raise TimeoutError('OPEN not confirmed or old GRIP signal not cleared; test stopped')
    robot.log('OPEN confirmed (grip=0, open=1)')


def command_close(robot):
    robot.close_gripper()


def read_fz(robot):
    return float(force_vector(robot)[2])


def calculate_grip_z(z0, detected_z, ratio):
    height = detected_z - z0
    z = z0 + height * ratio
    if z < z0 + MIN_GRIP_ABOVE_Z0_MM:
        raise RuntimeError('Calculated grip height too low; do not clamp and continue')
    return z


class ProbeSpeedLog:
    """기존 TCP 표본으로 속도 계산. 추가 ROS 조회 없음."""

    def __init__(self):
        self.first = self.window = self.last = None

    def update(self, z, stamp, fz, delta):
        if self.first is None:
            self.first = self.window = self.last = (z, stamp)
            return
        if stamp <= self.last[1]:
            return
        self.last = (z, stamp)
        elapsed = stamp - self.window[1]
        if elapsed >= PROBE_SPEED_LOG_SECONDS:
            speed = (self.window[0] - z) / elapsed
            print(
                f'[PROBE_SPEED] target={PROBE_FORCE_TARGET_N:g} N | '
                f'V_down={speed:+.2f} mm/s | Z={z:.2f} mm | '
                f'Fz={fz:+.2f} N | deltaFz={delta:+.2f} N | dt={elapsed:.3f} s',
                flush=True,
            )
            self.window = (z, stamp)

    def summary(self):
        if self.first is None or self.last[1] <= self.first[1]:
            print('[PROBE_SPEED_SUMMARY] insufficient pre-contact position samples', flush=True)
            return
        elapsed = self.last[1] - self.first[1]
        descent = self.first[0] - self.last[0]
        print(
            f'[PROBE_SPEED_SUMMARY] target={PROBE_FORCE_TARGET_N:g} N | '
            f'V_down_avg={descent / elapsed:+.2f} mm/s | '
            f'sampled_descent={descent:.2f} mm | sampled_time={elapsed:.3f} s',
            flush=True,
        )


def probe_contact(robot, z0, min_height_mm=None):
    time.sleep(0.3)
    samples = []
    for _ in range(PROBE_BASELINE_SAMPLES):
        samples.append(read_fz(robot))
        time.sleep(0.03)
    if max(samples) - min(samples) > PROBE_NOISE_MAX_N:
        raise RuntimeError('Stationary Fz unstable; probing cancelled')
    baseline = sum(samples) / len(samples)
    initial = robot.pose()
    if min_height_mm is None:
        min_height_mm = MIN_GRIP_ABOVE_Z0_MM / GRIP_HEIGHT_RATIO
    # 집기 높이를 만들 수 없는 낮은 구간은 측정하지 않음(기본 Z0+4mm).
    floor_z = z0 + max(40.0, min_height_mm)
    if initial[2] <= floor_z:
        raise RuntimeError('Probe start below floor bound')
    # 힘제어 중 check_motion()==0이어도 힘에 의한 이동은 가능하므로 완료판정에 사용 안 함.
    deadline = time.monotonic() + (initial[2] - floor_z) / PROBE_TIMEOUT_REFERENCE_MM_S + 10.0
    speed_log = ProbeSpeedLog()
    try:
        robot.command('set_ref_coord', robot.dsr.DR_BASE)
        robot.compliance_requested = True  # 요청 결과가 불명확해도 해제 시도
        robot.command('task_compliance_ctrl', stx=PROBE_STIFFNESS[:], time=PROBE_FORCE_RAMP_SECONDS)
        time.sleep(PROBE_MODE_WAIT_SECONDS)
        robot.force_requested = True
        robot.motion_requested = True
        robot.command('set_desired_force',
                      fd=[0.0, 0.0, -PROBE_FORCE_TARGET_N, 0.0, 0.0, 0.0],
                      dir=[0, 0, 1, 0, 0, 0], time=PROBE_FORCE_RAMP_SECONDS,
                      mod=robot.dsr.DR_FC_MOD_REL)
        while time.monotonic() < deadline:
            fz = read_fz(robot)
            delta = fz - baseline
            safety_limit = abs(delta) >= PROBE_SAFETY_N
            if safety_limit or delta >= PROBE_CONTACT_N:
                robot.stop_and_release()
                # 위치 조회보다 정지/힘 해제를 우선. 대략 높이이므로 정지 후 TCP Z 사용.
                pose = robot.pose()
                print(
                    f"[{'FORCE_LIMIT' if safety_limit else 'CONTACT'}] "
                    f"Z_stop={pose[2]:.2f} mm | Fz={fz:.2f} N | "
                    f"기준 Fz={baseline:.2f} N | 증가량={delta:+.2f} N",
                    flush=True,
                )
                speed_log.summary()
                if safety_limit:
                    raise RuntimeError(f'Probe force limit: delta Fz={delta:.2f}N')
                if pose[2] < floor_z:
                    raise ProbeMiss('Contact too low for a valid grip height', pose)
                return pose
            pose = robot.pose()
            if pose[2] <= floor_z:
                raise ProbeMiss('Lower probe bound reached without object height', pose)
            if math.dist(pose[:2], initial[:2]) > PROBE_XY_DRIFT_LIMIT_MM:
                raise RuntimeError('Excessive XY drift during compliance probing')
            speed_log.update(pose[2], time.monotonic(), fz, delta)
            time.sleep(GRIP_POLL)
        raise TimeoutError('Force probing timed out')
    finally:
        robot.stop_and_release()


def save_record(record, args):
    path = Path(args.record_file).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


class GripApproachContact(RuntimeError):
    pass


def force_vector(robot):
    values = robot.dsr.get_tool_force(ref=robot.dsr.DR_BASE)
    if not isinstance(values, (list, tuple)) or len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise RuntimeError('Invalid approach force reading')
    return list(values[:3])


def grip_force_approach(robot, target_z):
    """높이 탐지와 같은 Force 하강. 목표 높이=완료, 조기 접촉=재시도."""
    target_z = float(target_z)
    time.sleep(0.3)
    samples = []
    for _ in range(PROBE_BASELINE_SAMPLES):
        samples.append(force_vector(robot))
        time.sleep(0.03)
    if any(max(v[i] for v in samples)-min(v[i] for v in samples) > PROBE_NOISE_MAX_N
           for i in range(3)):
        raise RuntimeError('Stationary approach force unstable; cancelled')
    baseline = [sum(v[i] for v in samples)/len(samples) for i in range(3)]
    initial = robot.pose()
    if not math.isfinite(target_z) or initial[2] <= target_z:
        raise RuntimeError('Force approach must start above grip Z')
    deadline = time.monotonic() + (initial[2]-target_z)/PROBE_TIMEOUT_REFERENCE_MM_S + 10.0
    last_z, last_stamp = initial[2], time.monotonic()
    try:
        robot.command('set_ref_coord', robot.dsr.DR_BASE)
        robot.compliance_requested = True
        robot.command('task_compliance_ctrl', stx=PROBE_STIFFNESS[:], time=PROBE_FORCE_RAMP_SECONDS)
        time.sleep(PROBE_MODE_WAIT_SECONDS)
        robot.force_requested = True
        robot.motion_requested = True
        robot.command('set_desired_force',
                      fd=[0.0, 0.0, -GRIP_APPROACH_FORCE_TARGET_N, 0.0, 0.0, 0.0],
                      dir=[0, 0, 1, 0, 0, 0], time=PROBE_FORCE_RAMP_SECONDS,
                      mod=robot.dsr.DR_FC_MOD_REL)
        while time.monotonic() < deadline:
            force = force_vector(robot)
            delta_fz = force[2]-baseline[2]
            change = math.dist(force, baseline)
            # 접촉 판정을 목표 Z 도달 판정보다 우선한다.
            if change >= GRIP_APPROACH_SAFETY_N:
                raise RuntimeError(f'Grip approach safety force: {change:.2f}N')
            if delta_fz >= GRIP_APPROACH_CONTACT_N:
                robot.log(f'[GRIP APPROACH CONTACT] delta_force={change:.2f}N; '
                          f'deltaFx={force[0]-baseline[0]:+.2f}N; deltaFy={force[1]-baseline[1]:+.2f}N; '
                          f'deltaFz={delta_fz:+.2f}N; target_Z={target_z:.2f}')
                raise GripApproachContact(f'Force approach contact: deltaFz={delta_fz:+.2f}N, limit={GRIP_APPROACH_CONTACT_N:g}N')
            pose = robot.pose()
            if math.dist(pose[:2], initial[:2]) > PROBE_XY_DRIFT_LIMIT_MM:
                raise RuntimeError('Excessive XY drift during grip force approach')
            if pose[2] <= target_z:
                break
            now = time.monotonic()
            if now-last_stamp >= PROBE_SPEED_LOG_SECONDS:
                robot.log(f'[GRIP APPROACH SPEED] target={GRIP_APPROACH_FORCE_TARGET_N:g}N; '
                          f'V_down={(last_z-pose[2])/(now-last_stamp):+.2f}mm/s; '
                          f'Z={pose[2]:.2f}; goal_Z={target_z:.2f}; deltaFz={delta_fz:+.2f}N')
                last_z, last_stamp = pose[2], now
            time.sleep(GRIP_POLL)
        else:
            raise TimeoutError('Grip force approach timed out')
    finally:
        # 실패/취소/목표 도달 모두 정지 및 Force/Compliance 해제 후에만 다음 단계 허용.
        robot.stop_and_release()
    actual = robot.pose()
    if abs(actual[2]-target_z) > GRIP_APPROACH_Z_TOL_MM:
        raise GripApproachContact(f'Grip Z overshoot after release: actual={actual[2]:.2f}, target={target_z:.2f}')
    robot.log(f'[GRIP APPROACH REACHED] Z={actual[2]:.2f}; target={target_z:.2f}; force released')
    return actual


def move_cartesian(robot, target, args, require_grip=False, joint=False):
    target = list(map(float, target))
    expected = zyz_rotation(target[3:])
    if require_grip and feedback(robot, args.grip_di, args.open_di) != (1, 0):
        raise RuntimeError('Grip not confirmed before transfer')
    if joint:
        _, solution = robot.dsr.get_current_posx(ref=robot.dsr.DR_BASE)
        robot.motion_requested = True
        robot.command('amovejx', target, vel=TRANSFER_JOINT_VEL, acc=TRANSFER_JOINT_ACC,
                      sol=solution, ref=robot.dsr.DR_BASE, mod=robot.dsr.DR_MV_MOD_ABS)
    else:
        robot.motion_requested = True
        robot.command('amovel', target, vel=[LINEAR_VEL, 20.0], acc=[LINEAR_ACC, 40.0],
                      ref=robot.dsr.DR_BASE, mod=robot.dsr.DR_MV_MOD_ABS)
    deadline = time.monotonic() + 180.0
    next_grip_check = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if require_grip and now >= next_grip_check:
            if feedback(robot, args.grip_di, args.open_di) != (1, 0):
                raise RuntimeError('Grip lost during transfer; no automatic release or HOME')
            next_grip_check = now + SAMPLE_PERIOD
        if robot.state() == 0:
            actual = robot.pose()
            matrix = zyz_rotation(actual[3:])
            angle_error = max(abs(matrix[i][j]-expected[i][j]) for i in range(3) for j in range(3))
            if math.dist(actual[:3], target[:3]) <= 1.0 and angle_error <= TCP_ROTATION_TOL:
                robot.motion_requested = False
                return
        time.sleep(GRIP_POLL)
    raise TimeoutError('Transfer motion completion timeout')


def move_joint(robot, target, args, require_grip=False):
    """좌표 목표의 관절 보간. 직선/최저 높이가 보장되는 경로가 아니므로 이탈 후 사용."""
    move_cartesian(robot, target, args, require_grip, joint=True)


def move_z(robot, z, args, require_grip=False):
    target = robot.pose()
    if abs(target[2]-float(z)) <= 0.1:
        if require_grip and feedback(robot,args.grip_di,args.open_di) != (1,0):
            raise RuntimeError('Grip not confirmed during hold')
        return
    target[2] = float(z)
    move_cartesian(robot, target, args, require_grip)


def sector_probe_points(detection):
    side = detection['stopped_pose']
    direction = detection['direction']
    if direction not in ('-Y', '+Y'):
        raise ValueError('Invalid detection direction')
    sign = 1.0 if direction == '+Y' else -1.0
    return [[side[0] + radius * math.sin(math.radians(angle)),
             side[1] + sign * radius * math.cos(math.radians(angle)), PROBE_START_Z]
            for radius, angle in PROBE_POINTS]


def grip_plan(planned_probe, top_pose, z0, ratio):
    if math.dist(top_pose[:2], planned_probe[:2]) > PROBE_XY_DRIFT_LIMIT_MM:
        raise RuntimeError('Top contact drifted outside the planned probe point')
    return [*top_pose[:2], calculate_grip_z(z0, top_pose[2], ratio)]


def try_pick_and_place(robot, args, planned_probe, top, reference, home_z, record, attempt):
    grip_xyz = grip_plan(planned_probe, top, args.z0, args.height_ratio)
    grip_z = grip_xyz[2]
    place_z = grip_z + PLACE_Z_OFFSET_MM
    transfer_z = max(home_z, PROBE_START_Z, top[2] + PROBE_CLEARANCE_MM,
                     grip_z + PICK_LIFT_MM, place_z + PICK_LIFT_MM)
    attempt.update(status='grip_planned', grip_xyz=grip_xyz, place_xyz=[*PLACE_XY, place_z])
    record.update(status='grip_planned', grip_xyz=grip_xyz, place_xyz=[*PLACE_XY, place_z], transfer_z=transfer_z)
    save_record(record, args)
    robot.log(f'[GRIP] PICK={grip_xyz}; PLACE={[*PLACE_XY, place_z]}')
    # 회전 시 손끝이 물체에 닿지 않도록 측정 상단보다 높은 곳까지 이탈.
    retry_z = max(PROBE_START_Z, top[2] + GRIP_RETRY_LIFT_MM,
                  grip_z + GRIP_RETRY_LIFT_MM)
    move_z(robot, retry_z, args)
    initial_orientation = robot.pose()[3:6]
    gripped = False
    attempt['grip_trials'] = []
    for angle in GRIP_RETRY_ANGLES:
        orientation = initial_orientation[:]
        orientation[2] += angle  # ZYZ의 마지막 회전: 초기 툴 로컬 Z축 기준
        trial = {'angle_deg': angle}
        attempt['grip_trials'].append(trial)
        open_gripper(robot, args)
        robot.log(f'[GRIP TRY {len(attempt["grip_trials"])}/{len(GRIP_RETRY_ANGLES)}] angle={angle:g}deg; retry_Z={retry_z:.2f}; goal_Z={grip_z:.2f}')
        set_scan_orientation(robot, orientation, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)
        try:
            grip_force_approach(robot, grip_z)
        except GripApproachContact as contact:
            trial.update(status='approach_contact', reason=str(contact))
            robot.log(f'[GRIP RETRY] angle={angle:g}deg; reason={contact}; retract before next angle')
        else:
            command_close(robot)
            if wait_feedback(robot, args, (1, 0), args.timeout):
                trial['status'] = 'gripped'
                gripped = True
                break
            trial.update(status='grip_failed', last_di=list(feedback(robot, args.grip_di, args.open_di)))
            robot.log(f'[GRIP RETRY] angle={angle:g}deg; DI={trial["last_di"]}; grip not confirmed')
            open_gripper(robot, args)  # 미확인 물체는 원위치에서 해제
        save_record(record, args)
        move_z(robot, retry_z, args)
    if not gripped:
        set_scan_orientation(robot, initial_orientation, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)  # 90→0, 누적 회전 방지
        attempt['status'] = 'grip_failed'
        save_record(record, args)
        return False
    attempt['status'] = 'gripped'
    move_z(robot, grip_z + PICK_LIFT_MM, args, require_grip=True)
    time.sleep(PICK_HOLD_SECONDS)
    # 20mm 수직 이탈 후 상승/XY/자세를 하나의 관절 이동으로 수행.
    move_joint(robot, [*PLACE_XY, transfer_z, *orientation], args, require_grip=True)
    move_z(robot, place_z, args, require_grip=True)
    open_gripper(robot, args)
    attempt['status'] = 'placed'
    record['status'] = 'placed'
    save_record(record, args)
    retract_and_align_probe(robot, args, reference)  # 성공으로 조기 종료된 사이클 정렬
    record['status'] = 'object_placed'
    save_record(record, args)
    robot.log('PICK/PLACE COMPLETE; resuming raster directly')
    return True


def retract_and_align_probe(robot, args, reference):
    """접촉에서 수직 이탈한 뒤 기준 ABC 복원(툴Z 정렬, yaw 포함)."""
    if (robot.motion_requested or getattr(robot, 'force_requested', False)
            or getattr(robot, 'compliance_requested', False)):
        raise RuntimeError('Cannot align before probe stop and force/compliance release are confirmed')
    current = robot.pose()
    safe_z = max(PROBE_START_Z, current[2])
    move_z(robot, safe_z, args)  # 현 자세로 먼저 상승해 물체에서 이탈
    set_scan_orientation(robot, reference, GRIP_ROTATE_VEL, GRIP_ROTATE_ACC)  # 높은 위치에서 빠르게 0° 복원
    robot.log(f'[PROBE ALIGN] Z={safe_z:.2f}mm; reference ABC={reference}')
    return robot.pose()


def pick_and_place(robot, args, detection, reference, home_z, record):
    """True=이송 완료, False=집기 모두 실패, None=높이 전부 없음. 모두 이후 경로 재개."""
    side = detection['stopped_pose']
    if PROBE_START_Z <= side[2]:
        raise RuntimeError('Probe Z must exceed scan contact Z')
    if PROBE_START_Z < args.z0 + args.max_height + PROBE_CLEARANCE_MM:
        raise RuntimeError('Probe Z must clear configured maximum object height')
    event = {'horizontal_contact': detection, 'attempts': [], 'status': 'probing'}
    record.setdefault('events', []).append(event)
    record.update(status='horizontal_contact', horizontal_contact=detection,
                  reference_orientation=reference, verified_tcp_offset=TCP_OFFSET[:],
                  z0=args.z0, height_ratio=args.height_ratio)
    save_record(record, args)
    move_z(robot, PROBE_START_Z, args)
    set_scan_orientation(robot, reference)
    found_height = False
    closed_ready = True  # 탐색과 높이 탐지는 닫힘 명령을 유지
    for index, planned_probe in enumerate(sector_probe_points(detection), 1):
        attempt = {'index': index, 'radius_mm': PROBE_POINTS[index-1][0],
                   'angle_deg': PROBE_POINTS[index-1][1], 'planned_probe_xyz': planned_probe,
                   'status': 'probing'}
        event['attempts'].append(attempt)
        record.update(status='probing', active_probe_index=index, planned_probe_xyz=planned_probe)
        save_record(record, args)
        # 점 사이 이동은 항상 높은 위치에서, 그리퍼를 닫은 상태로 수행.
        if not closed_ready:
            command_close(robot)
            time.sleep(PROBE_CLOSE_WAIT)  # 실제 재닫힘에 필요한 시간은 유지
            closed_ready = True
        move_joint(robot, [*planned_probe, *robot.pose()[3:6]], args)
        robot.log(f'[HEIGHT {index}/{len(PROBE_POINTS)}] target={planned_probe}')
        try:
            top = probe_contact(robot, args.z0,
                                min_height_mm=MIN_GRIP_ABOVE_Z0_MM / args.height_ratio)
        except ProbeMiss as miss:
            # probe_contact의 finally에서 정지/해제가 실패하면 이 분기로 진입하지 않음.
            attempt.update(status='height_not_found', reason=miss.reason, last_pose=miss.pose)
            save_record(record, args)
            robot.log(f'[HEIGHT MISS {index}] {miss.reason}')
            move_z(robot, PROBE_START_Z, args)
            save_record(record, args)
            continue
        found_height = True
        attempt.update(status='top_contact', top_contact_pose=top,
                       estimated_top_height_mm=top[2]-args.z0)
        record.update(status='top_contact', top_contact_pose=top,
                      estimated_top_height_mm=top[2]-args.z0)
        save_record(record, args)
        move_z(robot, PROBE_START_Z, args)
        save_record(record, args)
        if try_pick_and_place(robot, args, planned_probe, top, reference, home_z, record, attempt):
            event['status'] = 'placed'
            save_record(record, args)
            return True
        closed_ready = False  # 집기 실패 분기에서 열었으므로 다음 탐지 전에 닫기
    event['aligned_pose'] = retract_and_align_probe(robot, args, reference)
    if not found_height:
        event['status'] = 'all_heights_missing'
        record['status'] = 'all_heights_missing'
        save_record(record, args)
        return None
    # 높이를 찾았지만 집기 실패: 높은 위치로 이탈 후 같은 경로 재개.
    event['status'] = 'all_grips_failed'
    record['status'] = 'all_grips_failed'
    save_record(record, args)
    return False


def resume_plan(detection):
    requested_y = detection['stopped_pose'][1] + SCAN_RESUME_Y_OFFSET_MM
    y = max(min(Y_START, Y_END), min(max(Y_START, Y_END), requested_y))
    # 방향을 뒤집으면 해당 선의 보정표와 불일치하므로 유지.
    return requested_y, y, detection['direction']


def resume_scan(robot, args, detection, orientations, record, close_required=True):
    requested_y, y, direction = resume_plan(detection)
    lane = detection['lane']
    x = X_LANES[lane-1]
    move_z(robot, max(PROBE_START_Z, robot.pose()[2]), args)
    if close_required:
        command_close(robot)
        time.sleep(PROBE_CLOSE_WAIT)
    move_joint(robot, [x, y, PROBE_START_Z, *orientations[direction]], args)
    move_z(robot, SCAN_Z, args)
    robot.dsr.wait(SETTLE_SECONDS)
    record.update(status='scan_resumed', active_lane=lane, resume_xyz=[x,y,SCAN_Z],
                  resume_requested_y=requested_y, resume_direction=direction)
    save_record(record, args)
    robot.log(f'[RESUME] lane={lane}, XYZ={[x,y,SCAN_Z]}, direction={direction}; ignore first {RESUME_IGNORE_MM:g}mm')
    return direction


def sequence(robot, args, record, recorder):
    robot.home()
    robot.close_gripper()
    robot.dsr.wait(INITIAL_CLOSE_WAIT)
    home = robot.pose()
    reference, home_z = home[3:6], home[2]
    orientations = {d: backward_orientation(reference,d) for d in ('-Y','+Y')}
    robot.joint_to_pose([X_LANES[0],Y_START,SCAN_Z,*orientations['-Y']],
                        APPROACH_VEL, APPROACH_ACC)
    robot.dsr.wait(SETTLE_SECONDS)
    record['placed_count'] = 0
    for index, x in enumerate(X_LANES):
        lane = index+1
        direction = '-Y' if index%2==0 else '+Y'
        end_y = Y_END if direction=='-Y' else Y_START
        orientation = orientations[direction]
        set_scan_orientation(robot, orientation)
        robot.dsr.wait(BETWEEN_LINES_WAIT)
        ignore = 0.0
        while True:
            record.update(status='scanning', active_lane=lane)
            detection = None
            try:
                robot.scan_line([x,end_y,SCAN_Z,*orientation], direction, recorder,lane,
                                ignore_start_mm=ignore)
            except ContactDetected as contact:
                detection = contact.detection
                robot.log(str(contact))
            recorder.flush()
            if detection is None:
                break
            # 높이 탐지/집기 시간은 탐색 CSV 구간과 분리.
            result = pick_and_place(robot,args,detection,reference,home_z,record)
            if result:
                record['placed_count'] += 1
            resume_scan(robot,args,detection,orientations,record, close_required=result is not None)
            ignore = RESUME_IGNORE_MM
            # 복귀점이 진행 방향 끝점이면 다음 선으로 진행.
            if abs(robot.pose()[1]-end_y)<=1.0:
                break
        record.update(status='lane_complete', completed_lane=lane)
        save_record(record,args)
        robot.dsr.wait(BETWEEN_LINES_WAIT)
        if index+1<len(X_LANES):
            robot.scan_line([X_LANES[index+1],end_y,SCAN_Z,*orientation],
                            '+X',recorder,lane,phase='transfer')
            recorder.flush()
            robot.dsr.wait(BETWEEN_LINES_WAIT)
    robot.home()
    record['status']='scan_complete'
    save_record(record,args)
    robot.log(f"SCAN COMPLETE; placed={record['placed_count']}; HOME reached")
    return True


def main():
    parser = argparse.ArgumentParser(description='Y scan -> force height probe -> grip -> place -> HOME')
    parser.add_argument('--service-prefix', default=SERVICE_PREFIX)
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT)
    parser.add_argument('--grip-di', type=int, default=GRIP_DI, choices=range(1,17))
    parser.add_argument('--open-di', type=int, default=OPEN_DI, choices=range(1,17))
    parser.add_argument('--z0', type=float, default=Z0)
    parser.add_argument('--max-height', type=float, default=MAX_OBJECT_HEIGHT_MM)
    parser.add_argument('--height-ratio', type=float, default=GRIP_HEIGHT_RATIO)
    parser.add_argument('--timeout', type=float, default=3.0)
    parser.add_argument('--record-file', default=RECORD_FILE)
    args = parser.parse_args()
    validate_baseline()
    if args.grip_di not in range(1,17) or args.open_di not in range(1,17) or args.grip_di == args.open_di:
        parser.error('DI must be distinct integers in 1..16')
    if not all(math.isfinite(v) for v in (args.z0, args.max_height, args.height_ratio, args.timeout)):
        parser.error('Inputs must be finite')
    if args.max_height <= 0 or args.timeout <= 0 or not 0 < args.height_ratio < 1:
        parser.error('max-height/timeout>0 and 0<height-ratio<1 required')
    if not 0 < GRIP_APPROACH_CONTACT_N < GRIP_APPROACH_FORCE_TARGET_N < GRIP_APPROACH_SAFETY_N:
        parser.error('grip approach requires 0 < contact < target < safety')
    if not PROBE_CONTACT_N < PROBE_FORCE_TARGET_N < PROBE_SAFETY_N:
        parser.error('contact < force target < force safety required')
    if PROBE_START_Z < args.z0 + args.max_height + PROBE_CLEARANCE_MM:
        parser.error('Probe Z must clear configured object maximum height plus clearance')
    if not PROBE_POINTS or any(not math.isfinite(r) or r <= 0 or not math.isfinite(a) or abs(a) >= 90 for r,a in PROBE_POINTS):
        parser.error('Probe points require finite radius>0 and -90<angle<90')
    print(VERSION, flush=True)
    print(f'Grip rotation={GRIP_ROTATE_VEL:g}deg/s, acc={GRIP_ROTATE_ACC:g}deg/s2')
    print(f'Grip approach: Compliance/Force target={GRIP_APPROACH_FORCE_TARGET_N:g}N; contact=deltaFz>={GRIP_APPROACH_CONTACT_N:g}N; angles={GRIP_RETRY_ANGLES}')
    print(f'Raster X={X_LANES[0]}..{X_LANES[-1]}, Y={Y_START}..{Y_END}, Z={SCAN_Z}; {len(X_LANES)} lanes; place XY={PLACE_XY}')
    print(f'All heights missing: resume at collision BASE Y {SCAN_RESUME_Y_OFFSET_MM:+g}mm (bounded by scan endpoints).')
    print(f'Force target={PROBE_FORCE_TARGET_N:g}N, contact={PROBE_CONTACT_N:g}N; DI={args.grip_di}/{args.open_di}; Z0={args.z0:g}')
    print(f'Place Z = grip Z + {PLACE_Z_OFFSET_MM:g}mm; grip -> place -> same lane Y-50 resume (HOME only at raster end).')
    robot, recorder, record, code = None, None, {}, 1
    try:
        robot = Robot(args.service_prefix)
        robot.check_setup()
        feedback(robot, args.grip_di, args.open_di)
        if input('Type start then Enter to run: ').strip().lower() != 'start':
            return 0
        robot.ensure_tcp()
        recorder = Recorder(args.output_dir)
        print('CSV directory:', recorder.directory)
        code = 0 if sequence(robot, args, record, recorder) else 2
    except (KeyboardInterrupt, EOFError):
        record.update(status='interrupted')
        print('Interrupted; no automatic transfer/HOME/release')
        code = 130
    except Exception as exc:
        record.update(status='error', error=str(exc))
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        if robot is not None:
            try:
                robot.stop_and_release()
            except BaseException as exc:
                record.update(status='cleanup_error', cleanup_error=str(exc))
                print('STOP/RELEASE NOT CONFIRMED:', exc)
                code = 1
            finally:
                robot.shutdown()
        if recorder is not None:
            try:
                recorder.flush()
            except Exception as exc:
                print('CSV save failed:', exc)
                code = 1
        if record:
            try:
                save_record(record, args)
                print('Record:', str(Path(args.record_file).expanduser()))
            except Exception as exc:
                print('Record write failed:', exc)
                code = 1
    return code


if __name__ == '__main__':
    raise SystemExit(main())
