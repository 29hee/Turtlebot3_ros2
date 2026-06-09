#!/usr/bin/env python3
"""
phase1_mapping.launch.py — Phase 1: SLAM 한 바퀴 + 색 위치 후보 수집

구성:
  (시뮬) gzserver + gzclient + robot_state_publisher + spawn
  slam_toolbox (online async)
  image_upright        (실로봇: 카메라 180° 보정)
  vision_node          → /detected_color, /color_signal
  phase1_explorer      → 벽타기 한 바퀴 + 색 후보 격자투표
                       → maps/color_candidates.yaml 저장 + /phase1_done
  [phase1_explorer 종료 시] map_saver_cli → maps/color_room.pgm/.yaml 저장

사용:
  export TURTLEBOT3_MODEL=burger_cam
  source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash

  # 시뮬:
  ros2 launch <pkg>/launch/phase1_mapping.launch.py

  # 실로봇:
  ros2 launch <pkg>/launch/phase1_mapping.launch.py sim:=false flip:=180

Phase 1 완료 후 Phase 2 로:
  ros2 launch <pkg>/launch/phase2_visit.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

ROBOT_MODEL = 'burger_cam'
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)

    phase1_explorer = os.path.join(pkg, 'scripts', 'phase1_explorer.py')
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')

    sim = LaunchConfiguration('sim', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=sim)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    duration = LaunchConfiguration('duration', default='600')
    flip = LaunchConfiguration('flip', default='180')
    gui = LaunchConfiguration('gui', default='false')
    map_save = LaunchConfiguration(
        'map_save', default=os.path.join(pkg, 'maps', 'color_room'))

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    world = os.path.join(pkg, 'worlds', 'color_room.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(sim),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(
            ["'", sim, "' == 'true' and '", gui, "' == 'true'"])),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(sim),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        condition=IfCondition(sim),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    upright_proc = ExecuteProcess(
        cmd=['python3', image_upright, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['flip:=', flip], '-p', 'compressed_in:=false'],
        condition=IfCondition(PythonExpression(["'", sim, "' == 'false'"])),
        output='screen',
    )
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    explorer_proc = ExecuteProcess(
        cmd=['python3', phase1_explorer,
             '--duration', duration,
             '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )

    # Phase 1 탐사 종료 시: 맵 저장 → 전체 launch 종료
    map_saver_proc = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_save,
             '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    save_map = RegisterEventHandler(
        OnProcessExit(
            target_action=explorer_proc,
            on_exit=[map_saver_proc],
        )
    )
    shutdown_after_save = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver_proc,
            on_exit=[EmitEvent(event=Shutdown(reason='Phase 1 완료'))],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('sim', default_value='true',
                              description='true=시뮬 | false=실로봇'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='가제보 GUI 창 표시'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('duration', default_value='600',
                              description='Phase1 탐사 시간 상한 [s]'),
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(180|v|h|none). 실로봇 전용'),
        DeclareLaunchArgument('map_save',
                              default_value=os.path.join(pkg, 'maps', 'color_room'),
                              description='점유격자맵 저장 경로(확장자 없이)'),
        gzserver, gzclient, rsp, spawn, slam,
        upright_proc, vision_proc, explorer_proc,
        save_map,
        shutdown_after_save,
    ])
