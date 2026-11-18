"""
robot.launch.py — 두산 M0609 + RG2 로봇 bringup (box_project용)
================================================================
관리: 관제(정쓰). m0609_rg2_bringup/launch/bringup.launch.py(수업 패키지)와 같은 노드를 띄우되,
우리 프로젝트에 맞게 두 가지만 다르다. (원본 패키지는 고치지 않는다)

  1. gripper_driver (기본 false): OnRobot RG2 드라이버(OnRobotRGControllerServer)를 띄우지 않는다.
     우리 파트들은 RG2를 Modbus로 직접 쓰는데(box_comm/gripper.py), Compute Box는 동시 접속이 1개라
     드라이버가 떠 있으면 파트가 그리퍼를 못 잡는다 (예전엔 매번 pkill -f OnRobotRGControllerServer).
  2. rviz (기본 false): 노트북 GPU 부담을 줄이려고 끔. 보고 싶으면 rviz:=true

사용 (보통은 system.launch.py가 이걸 포함해서 띄운다)
  ros2 launch box_bringup robot.launch.py mode:=virtual          # = 예전 sodvir
  ros2 launch box_bringup robot.launch.py mode:=real             # = 예전 sodreal (host 192.168.1.100)
  host·port는 mode에 맞춰 기본값이 정해진다 (virtual 127.0.0.1 / real 192.168.1.100, 12345)
================================================================
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

REAL_HOST = '192.168.1.100'
VIRTUAL_HOST = '127.0.0.1'


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    args = [
        DeclareLaunchArgument('mode', default_value='virtual', choices=['virtual', 'real'],
                              description='virtual (에뮬레이터) | real (실제 로봇)'),
        DeclareLaunchArgument('host', description='로봇 IP (기본: real 192.168.1.100 / virtual 127.0.0.1)',
                              default_value=PythonExpression(
                                  ["'", REAL_HOST, "' if '", mode, "' == 'real' else '", VIRTUAL_HOST, "'"])),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('gripper_driver', default_value='false',
                              description='OnRobot 드라이버 (true면 파트의 Modbus 그리퍼 접속과 충돌)'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    is_virtual = IfCondition(PythonExpression(["'", mode, "' == 'virtual'"]))
    real_and_driver = IfCondition(PythonExpression(
        ["'", mode, "' == 'real' and '", LaunchConfiguration('gripper_driver'), "'.lower() == 'true'"]))

    share = get_package_share_directory('m0609_rg2_bringup')
    xacro_file = os.path.join(share, 'xacro', 'my_m0609_rg2.urdf.xacro')
    rviz_config = os.path.join(share, 'rviz', 'default.rviz')

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' name:=dsr01', ' host:=', host, ' port:=', port, ' mode:=', mode,
        ' model:=m0609', ' update_rate:=100',
    ]), value_type=str)

    # [virtual] 이전 에뮬레이터 컨테이너 정리 → 에뮬레이터 실행 (원본과 같음)
    emulator_cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'docker rm -f dsr01_emulator 2>/dev/null || true'],
        condition=is_virtual, output='log')
    run_emulator = Node(
        package='dsr_bringup2', executable='run_emulator', namespace='dsr01',
        parameters=[{'name': 'dsr01'}, {'host': host}, {'port': port}, {'mode': mode},
                    {'model': 'm0609'}, {'gripper': 'none'}, {'mobile': 'none'}],
        condition=is_virtual, output='screen')
    start_emulator = RegisterEventHandler(OnProcessExit(target_action=emulator_cleanup, on_exit=[run_emulator]))

    control_node = Node(
        package='controller_manager', executable='ros2_control_node', namespace='dsr01',
        parameters=[{'robot_description': robot_description}, {'update_rate': 100},
                    PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml'])],
        output='both')
    jsb_spawner = Node(
        package='controller_manager', executable='spawner', namespace='dsr01',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'])
    controller_spawner = Node(
        package='controller_manager', executable='spawner', namespace='dsr01',
        arguments=['dsr_controller2', '-c', 'controller_manager'])
    delay_controller = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[controller_spawner]))

    # [virtual] 가상 그리퍼 서비스 (원본과 같음. 우리 파트는 BOX_GRIPPER=fake를 써서 이건 안 씀)
    gripper_virtual = Node(
        package='m0609_rg2_bringup', executable='gripper_virtual_node.py', namespace='dsr01',
        name='gripper_virtual_node', condition=is_virtual, output='screen')

    # [real + gripper_driver:=true만] OnRobot 드라이버 + 폭 → 관절 변환 (기본은 끔)
    onrobot_driver = Node(
        package='onrobot_rg_control', executable='OnRobotRGControllerServer', namespace='dsr01',
        name='OnRobotRGControllerServer', output='screen',
        parameters=[{'/onrobot/control': 'modbus', '/onrobot/ip': '192.168.1.1', '/onrobot/port': 502,
                     '/onrobot/changer_addr': 65, '/onrobot/gripper': 'rg2', '/onrobot/offset': 5}],
        remappings=[('/joint_states', '/onrobot_joint_states')],
        condition=real_and_driver)
    gripper_joint_state = Node(
        package='m0609_rg2_bringup', executable='gripper_joint_state_publisher.py', namespace='dsr01',
        name='gripper_joint_state_publisher', condition=real_and_driver, output='screen')

    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher', namespace='dsr01',
        name='joint_state_publisher', parameters=[{'source_list': ['joint_states', 'gripper_joint_states']}])
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', namespace='dsr01',
        name='robot_state_publisher', output='both', parameters=[{'robot_description': robot_description}])
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', namespace='dsr01',
        name='static_transform_publisher', output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])
    rviz = Node(
        package='rviz2', executable='rviz2', namespace='dsr01', name='rviz2', output='log',
        arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription(args + [
        emulator_cleanup, start_emulator, gripper_virtual,
        control_node, jsb_spawner, delay_controller,
        onrobot_driver, gripper_joint_state,
        joint_state_publisher, robot_state_publisher, static_tf, rviz,
    ])
