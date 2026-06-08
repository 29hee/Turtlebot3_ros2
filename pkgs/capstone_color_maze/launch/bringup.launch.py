#!/usr/bin/env python3
"""
bringup.launch.py
색미로 '런타임 상시 구동' — 한 번 켜두면 RED/GREEN/BLUE 어떤 색이 와도 처리한다.
[수정 사양: 색을 launch 인자가 아니라 런타임 토픽(/target_color)으로 받는다]

구성(전부 색 무관, 계속 떠 있음):
  (옵션) gazebo + Burger 스폰        start_gazebo:=true  (시뮬 검증용)
  nav2_bringup bringup_launch.py     map_server + AMCL + planner/controller/bt
  color_confirm.py                   /target_color 로 대상 색 동적 전환, /target_confirmed 발행
  maze_tour.py (oneshot=false)       /target_color 받으면 그 색 순회 → /maze_done → 다시 대기

사용:
  # 1) 스택 상시 구동(시뮬)
  ros2 launch <경로>/bringup.launch.py
  #    실로봇:  start_gazebo:=false use_sim_time:=false relocalize:=true
  # 2) 색은 그때그때 토픽으로 지정(재시작 불필요):
  ros2 topic pub --once /target_color std_msgs/String "{data: RED}"
  ros2 topic pub --once /target_color std_msgs/String "{data: GREEN}"
  #    또는:  ros2 launch <경로>/mission.launch.py color:=BLUE
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)
    default_map = os.path.join(pkg, 'maps', 'color_room.yaml')
    default_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')
    color_confirm = os.path.join(pkg, 'scripts', 'color_confirm.py')
    maze_tour = os.path.join(pkg, 'scripts', 'maze_tour.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_gazebo = LaunchConfiguration('start_gazebo', default='false')
    # 실로봇: 시작 시 제자리 회전으로 자기위치부터 찾기. 시뮬은 set_initial_pose 라 false.
    relocalize = LaunchConfiguration('relocalize', default='false')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    default_landmarks = os.path.join(pkg, 'maps', 'color_landmarks.yaml')
    landmarks = LaunchConfiguration('landmarks', default=default_landmarks)

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

    # 색 무관: target_color 를 주지 않는다 → 둘 다 /target_color 를 구독해 대기.
    confirm_proc = ExecuteProcess(
        cmd=['python3', color_confirm, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    tour_proc = ExecuteProcess(
        cmd=['python3', maze_tour, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['relocalize:=', relocalize],
             '-p', ['landmarks_path:=', landmarks],
             '-p', 'oneshot:=false'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_gazebo', default_value='false',
                              description='시뮬레이션이면 true, 실로봇이면 false(기본)'),
        DeclareLaunchArgument('relocalize', default_value='false',
                              description='실로봇이면 true: 시작 시 제자리 회전으로 자기위치 추정'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('landmarks', default_value=default_landmarks,
                              description='색 시맨틱맵(color_landmarks.yaml) 경로'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        gazebo,
        nav2,
        confirm_proc,
        tour_proc,
    ])
