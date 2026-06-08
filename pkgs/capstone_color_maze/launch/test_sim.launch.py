#!/usr/bin/env python3
"""
test_sim.launch.py — Gazebo 약식 테스트 런치

world: test_panels.world (4x4m 방, RED×2 + GREEN×3 패널)
robot: burger_cam (카메라 포함)

사용법 — 두 단계를 별도 터미널에서 순서대로 실행:

  [1단계: SLAM 구축]
    ros2 launch /home/hee/workspace/co_project/pkgs/capstone_color_maze/launch/test_sim.launch.py mode:=slam
    → Gazebo + SLAM + test_real(slam 모드 = 벽타기)
    → RViz 맵이 닫히면 Ctrl+C

  [2단계: 색 감지 + 추종]
    ros2 launch /home/hee/workspace/co_project/pkgs/capstone_color_maze/launch/test_sim.launch.py mode:=color
    → Gazebo + SLAM + test_real(color 모드 = 360° 스핀 → 터미널 입력)

참고: 각 단계마다 Gazebo 가 새로 뜬다. 2단계에서 SLAM 이 재시작되지만
      작은 방이라 스핀 전에 충분히 안정화된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_MODEL = 'burger_cam'
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg  = os.path.dirname(here)
    world           = os.path.join(pkg, 'worlds', 'test_panels.world')
    test_real       = os.path.join(pkg, 'scripts', 'test_real.py')
    sim_digit_pub   = os.path.join(pkg, 'scripts', 'sim_digit_pub.py')

    gazebo_ros   = get_package_share_directory('gazebo_ros')
    tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    urdf_path = os.path.join(
        tb3_gazebo, 'models', f'turtlebot3_{ROBOT_MODEL}', 'model.sdf')
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ROBOT_MODEL,
            '-file', urdf_path,
            '-x', '-1.5', '-y', '-1.5', '-z', '0.01',
        ],
        output='screen',
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    digit_proc = ExecuteProcess(
        cmd=['python3', sim_digit_pub,
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('mode', default_value='slam',
                              description='(미사용) test_real.py 는 별도 터미널에서 실행'),
        gzserver, gzclient, rsp, spawn, slam,
        digit_proc,
    ])
