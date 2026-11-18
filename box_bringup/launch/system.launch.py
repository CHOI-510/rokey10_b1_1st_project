"""
system.launch.py — 전체 실행 (로봇 bringup + 탐지·판별·적재 + DB 노드 + 관제 + 관리자 GUI)
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
  rviz:=false          RViz (bringup에 포함)
  gripper_driver:=false  OnRobot 드라이버 (켜면 파트의 Modbus 그리퍼와 충돌 — 보통 끔)
  sim_boxes:="420,100,50;510,-20,60"   virtual 탐지 가상 박스
  box_count:=4 classes:="A,B,B,D"      fake 탐지 박스 수 / fake 판별 순서
고정 파라미터는 config/params.yaml (관제 progress_timeout_sec, clear_zone ...)

관제는 상주 모드: launch가 뜨면 READY에서 시작 명령(웹 UI / 관리자 GUI)을 기다린다.
  launch로 띄우면 터미널 키(s/q)는 들어가지 않는다 → 시작·취소·바로 시작은 관리자 GUI 버튼으로.
  비상정지: 웹 UI / 관리자 GUI / 펜던트. 전체 종료: launch 터미널에서 Ctrl+C 한 번.
================================================================
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _expr(*parts):
    return PythonExpression(list(parts))


def generate_launch_description():
    here = get_package_share_directory('box_bringup')
    params = os.path.join(here, 'config', 'params.yaml')

    mode = LaunchConfiguration('mode')
    fake = LaunchConfiguration('fake')

    # 조건 문자열 조각 (PythonExpression 안에서 평가)
    is_fake = ["'", fake, "'.lower() == 'true'"]
    not_fake = ["'", fake, "'.lower() != 'true'"]
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

    # fake:=true는 시험이라 매번 재고 0으로 시작 (fake_place도 0에서 셈 → 불일치 안 남). 실제·에뮬레이터는 DB 값 그대로
    #   (DB 노드는 Supabase에서는 start_inventory를 무시함 — 실제 데이터 보호)
    database = node('box_database', 'database_main', parameters=[{
        'store': LaunchConfiguration('db_store'),
        'start_inventory': _expr("[0, 0, 0, 0, 0] if (", *is_fake, ") else [-1]")}])

    control = node(
        'box_control', 'control',
        parameters=[{'start_delay_sec': LaunchConfiguration('start_delay'),
                     'robot_monitor': _expr("'fake' if (", *is_fake, ") else 'dsr'")}],
        ros_arguments=['-p', 'gui:=false'])          # GUI는 아래 노드로 따로 (관제가 또 띄우지 않게)

    gui = node('box_control', 'monitor_gui', IfCondition(LaunchConfiguration('gui')),
               [{'robot_poll': _expr("False if (", *is_fake, ") else True")}])

    return LaunchDescription(args + [gripper_env, robot, database, *parts, gui, control])
