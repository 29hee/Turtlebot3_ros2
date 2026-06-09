#!/usr/bin/env python3
"""
phase2_visit.launch.py — Phase 2: 저장 맵 로드 + AMCL + Nav2 + 색 후보 정면 방문

Phase 1 (phase1_mapping.launch.py) 이 끝난 뒤 실행한다.
maps/color_candidates.yaml + maps/color_room.yaml 이 준비되어 있어야 한다.

구성:
  (시뮬) color_maze.launch.py (gazebo + robot_state_publisher)
  nav2_bringup bringup_launch.py  (map_server + AMCL + planner/controller)
  image_upright        (실로봇: 카메라 180° 보정)
  vision_node          → /detected_color, /color_signal
  digit_recognizer     → /detected_digit  (EasyOCR)
  color_confirm        → /target_confirmed (color_confirm.py, 옵션 참고용)
  phase2_visitor       → 후보 방문 + 색/숫자 확인 → color_landmarks.yaml 저장
                       → /phase2_done(True) 발행 후 종료

사용:
  # 시뮬:
  ros2 launch <pkg>/launch/phase2_visit.launch.py

  # 실로봇:
  ros2 launch <pkg>/launch/phase2_visit.launch.py start_gazebo:=false relocalize:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)

    default_map = os.path.join(pkg, 'maps', 'color_room.yaml')
    default_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')

    phase2_visitor = os.path.join(pkg, 'scripts', 'phase2_visitor.py')
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    digit_recognizer = os.path.join(pkg, 'scripts', 'digit_recognizer.py')
    color_confirm = os.path.join(pkg, 'scripts', 'color_confirm.py')
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')

    start_gazebo = LaunchConfiguration('start_gazebo', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    relocalize = LaunchConfiguration('relocalize', default='false')
    flip = LaunchConfiguration('flip', default='180')

    nav2_bringup = get_package_share_directory('nav2_bringup')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'color_maze.launch.py')),
        condition=IfCondition(start_gazebo),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    upright_proc = ExecuteProcess(
        cmd=['python3', image_upright, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['flip:=', flip], '-p', 'compressed_in:=false'],
        condition=IfCondition(PythonExpression(["'", start_gazebo, "' == 'false'"])),
        output='screen',
    )
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    digit_proc = ExecuteProcess(
        cmd=['python3', digit_recognizer, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    # color_confirm: /target_confirmed 토픽 발행 (phase2_visitor가 /target_color 로 색 알려줌)
    confirm_proc = ExecuteProcess(
        cmd=['python3', color_confirm, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    visitor_proc = ExecuteProcess(
        cmd=['python3', phase2_visitor, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['relocalize:=', relocalize]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_gazebo', default_value='true',
                              description='시뮬이면 true, 실로봇이면 false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('relocalize', default_value='false',
                              description='실로봇이면 true: AMCL 수렴 후 방문 시작'),
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(실로봇 전용)'),
        DeclareLaunchArgument('map',
                              default_value=default_map,
                              description='Phase 1 에서 저장한 맵 yaml'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        gazebo,
        nav2,
        upright_proc,
        vision_proc,
        digit_proc,
        confirm_proc,
        visitor_proc,
    ])
