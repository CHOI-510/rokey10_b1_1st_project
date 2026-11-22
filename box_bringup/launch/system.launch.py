"""
system.launch.py — 전체 실행 (로봇 bringup + 탐지·판별·적재 + DB·웹 브리지 + 관제 GUI)
================================================================
관리: 관제(정쓰).

  ros2 launch box_bringup system.launch.py mode:=real       # 실제 로봇 (시연)
  ros2 launch box_bringup system.launch.py mode:=virtual    # 에뮬레이터 (판별은 fake — 시뮬 모드가 없음)
  ros2 launch box_bringup system.launch.py fake:=true       # 로봇 없이 전부 fake (예전 fake_run.sh 기본)

mode에 따라 바뀌는 것
  | mode            | 로봇 bringup              | 탐지              | 판별        | 적재                 | 그리퍼(BOX_GRIPPER) |
  | real            | 실제 192.168.1.100        | scan_main         | judge_main  | place_main           | modbus              |
  | virtual         | 에뮬레이터 127.0.0.1       | scan_main 가상 박스 | fake_judge  | place_main 가상 접촉 | fake                |
  | fake:=true      | 없음 (robot_monitor=fake) | fake_scan         | fake_judge  | fake_place           | fake                |

그 밖의 인자 (기본값)
  bringup:=true        false면 로봇 bringup은 따로 띄운 것을 씀 (sodvir/sodreal 터미널을 따로 쓸 때)
  start_delay:=600.0   시작 카운트다운 [s] (시험: 10.0)
  db_store:=auto       DB 노드 저장소: auto(키 있으면 Supabase) / file / supabase
  gui:=true            관리자 모니터 GUI
  rosbridge:=true      웹 UI용 rosbridge WebSocket (127.0.0.1에만 바인딩)
  rosbridge_port:=9090 rosbridge WebSocket 포트
  inventory_check:=false 시작 재고 실측 (real은 실제 점검, virtual/fake는 가짜 점검)
  rviz:=false          RViz (bringup에 포함)
  gripper_driver:=false  OnRobot 드라이버 (켜면 파트의 Modbus 그리퍼와 충돌 — 보통 끔)
  sim_boxes:="420,100,50;510,-20,60"   virtual 탐지 가상 박스
  box_count:=4 classes:="A,B,B,D"      fake 탐지 박스 수 / fake 판별 순서
고정 파라미터는 config/params.yaml (관제 progress_timeout_sec, clear_zone ...)

관제는 상주 모드: launch가 뜨면 READY에서 시작 명령(웹 UI / 관리자 GUI)을 기다린다.
  launch로 띄우면 터미널 키(s/q)는 들어가지 않는다 → 시작·취소·바로 시작은 관리자 GUI 버튼으로.
  비상정지: 웹 UI / 관리자 GUI / 펜던트. 전체 종료: launch 터미널에서 Ctrl+C 한 번.
재고는 DB/UI에서 설정하는 것이 기본이며, inventory_check:=true로 시작 재고 실측을 켤 수 있다.
DB HTTP(기본 8765) 또는 rosbridge(기본 9090) 포트가 이미 사용 중이면 로봇 노드를 시작하지 않는다.
================================================================
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from box_bringup.runtime_guard import ensure_ports_available


def _expr(*parts):
    return PythonExpression(list(parts))


def _check_runtime(context):
    endpoints = [('DB HTTP', int(os.environ.get('BOX_DB_PORT', '8765')))]
    if LaunchConfiguration('rosbridge').perform(context).lower() == 'true':
        endpoints.append(('rosbridge WebSocket', int(LaunchConfiguration('rosbridge_port').perform(context))))
    ensure_ports_available(endpoints)
    return []


def _include_rosbridge(context):
    if LaunchConfiguration('rosbridge').perform(context).lower() != 'true':
        return []
    bridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
    return [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(bridge_launch),
        launch_arguments={'address': '127.0.0.1', 'port': LaunchConfiguration('rosbridge_port')}.items())]


def generate_launch_description():
    here = get_package_share_directory('box_bringup')
    params = os.path.join(here, 'config', 'params.yaml')

    mode = LaunchConfiguration('mode')
    fake = LaunchConfiguration('fake')

    # 조건 문자열 조각 (PythonExpression 안에서 평가)
    is_fake = ["'", fake, "'.lower() == 'true'"]
    not_fake = ["'", fake, "'.lower() != 'true'"]
    inventory_enabled = ["'", LaunchConfiguration('inventory_check'), "'.lower() == 'true'"]
    is_real = ["'", mode, "' == 'real'"]
    is_virtual = ["'", mode, "' == 'virtual'"]

    def when(*conds):                       # 조건 여러 개 AND
        parts = []
        for i, c in enumerate(conds):
            parts += ([' and '] if i else []) + ['('] + c + [')']
        return IfCondition(PythonExpression(parts))

    def when_any(*conds):                   # 조건 여러 개 OR
        parts = []
        for i, c in enumerate(conds):
            parts += ([' or '] if i else []) + ['('] + c + [')']
        return IfCondition(PythonExpression(parts))

    args = [
        DeclareLaunchArgument('mode', default_value='virtual', choices=['virtual', 'real']),
        DeclareLaunchArgument('fake', default_value='false', description='true: 로봇 없이 전부 fake'),
        DeclareLaunchArgument('bringup', default_value='true', description='false: 로봇 bringup을 따로 띄운 경우'),
        DeclareLaunchArgument('start_delay', default_value='600.0'),
        DeclareLaunchArgument('db_store', default_value='auto', choices=['auto', 'file', 'supabase']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rosbridge', default_value='true', description='웹 API용 rosbridge WebSocket'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('inventory_check', default_value='false',
                               description='true: 시작 전 재고 점검 Action 실행'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gripper_driver', default_value='false'),
        DeclareLaunchArgument('sim_boxes', default_value='420,100,50;510,-20,60'),
        DeclareLaunchArgument('box_count', default_value='4'),
        DeclareLaunchArgument('classes', default_value='A,B,B,D'),
    ]

    # 그리퍼: 실제 로봇일 때만 Modbus, 나머지(에뮬레이터·fake)는 가짜 그리퍼 — 모든 노드에 적용
    gripper_env = SetEnvironmentVariable(
        'BOX_GRIPPER', _expr("'modbus' if (", *is_real, ") and (", *not_fake, ") else 'fake'"))

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'robot.launch.py')),
        launch_arguments={'mode': mode, 'rviz': LaunchConfiguration('rviz'),
                          'gripper_driver': LaunchConfiguration('gripper_driver')}.items(),
        condition=when(not_fake, ["'", LaunchConfiguration('bringup'), "'.lower() == 'true'"]))

    def node(pkg, exe, condition=None, parameters=(), **kw):
        return Node(package=pkg, executable=exe, parameters=[params, *parameters], condition=condition,
                    output='screen', emulate_tty=True, **kw)

    parts = [
        # 탐지
        node('box_scan', 'scan_main', when(not_fake, is_real)),
        node('box_scan', 'scan_main', when(not_fake, is_virtual),
             [{'sim_boxes': LaunchConfiguration('sim_boxes')}]),
        node('box_control', 'fake_scan', when(is_fake),
             [{'box_count': LaunchConfiguration('box_count')}]),
        # 판별 (실제 코드는 힘·폭 측정이 필요해 에뮬레이터에서는 fake)
        node('box_judge', 'judge_main', when(not_fake, is_real)),
        node('box_control', 'fake_judge', when_any(is_fake, is_virtual),
             [{'class_sequence': LaunchConfiguration('classes')}]),
        # 적재
        node('box_place', 'place_main', when(not_fake, is_real)),
        node('box_place', 'place_main', when(not_fake, is_virtual), [{'sim_contact': True}]),
        node('box_control', 'fake_place', when(is_fake)),
    ]

    # 시작 재고 점검은 선택 기능. real에서는 실제 접촉 실측, virtual/fake에서는 fake Action.
    inventory_check_nodes = [
        node('box_place', 'inventory_check', when(not_fake, is_real, inventory_enabled),
             [{'robot_setup': 'on', 'inventory_calibration_source':
               _expr("'file' if '", LaunchConfiguration('db_store'), "' == 'file' else 'supabase'")}]),
        node('box_control', 'fake_inventory_check', when(is_fake, inventory_enabled),
             [{'fake_move_robot': False}]),
        node('box_control', 'fake_inventory_check', when(not_fake, is_virtual, inventory_enabled),
             [{'fake_move_robot': True}]),
    ]

    # fake runs start with a clean inventory. Real/virtual runs retain DB values.
    database = node('box_database', 'database_main', parameters=[{
        'store': LaunchConfiguration('db_store'),
        'start_inventory': _expr("[0, 0, 0, 0, 0] if (", *is_fake, ") else [-1]")}])

    control = node(
        'box_control', 'control',
        parameters=[{'start_delay_sec': LaunchConfiguration('start_delay'),
                     'inventory_check_enabled': LaunchConfiguration('inventory_check'),
                     'robot_hardware_mode': mode,
                     'robot_monitor': _expr("'fake' if (", *is_fake, ") else 'dsr'")}],
        ros_arguments=['-p', 'gui:=false'])          # GUI는 아래 노드로 따로 (관제가 또 띄우지 않게)

    gui = node('box_control', 'monitor_gui', IfCondition(LaunchConfiguration('gui')),
               [{'robot_poll': _expr("False if (", *is_fake, ") else True")}])

    return LaunchDescription(args + [
        OpaqueFunction(function=_check_runtime),
        gripper_env, robot, database, *parts, *inventory_check_nodes, gui, control,
        OpaqueFunction(function=_include_rosbridge)])
