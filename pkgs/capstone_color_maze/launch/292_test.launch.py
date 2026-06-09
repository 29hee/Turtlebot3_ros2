#!/usr/bin/env python3
"""
292_test.launch.py — 실로봇 SLAM 매핑 (Gazebo 없음)

구성:
  slam_toolbox        (online_async, use_sim_time=false)
  smart_color_mapper  (벽타기 + 색 발견 시 정면 확인 후 기록)

전제: 로봇(라즈베리파이)에서 아래 두 개가 먼저 켜져 있어야 함
  ros2 launch turtlebot3_bringup robot.launch.py
  ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw

사용:
  ros2 launch capstone_color_maze 292_test.launch.py
  ros2 launch capstone_color_maze 292_test.launch.py spin_speed:=0.25
  # 맵 저장:
  ros2 run nav2_map_server map_saver_cli -f maps/color_room
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)
    smart_mapper = os.path.join(pkg, 'scripts', 'smart_color_mapper.py')

    spin_speed = LaunchConfiguration('spin_speed', default='0.25')

    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    mapper_proc = ExecuteProcess(
        cmd=['python3', smart_mapper,
             '--ros-args',
             '-p', 'use_sim_time:=false',
             '-p', ['spin_speed:=', spin_speed]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('spin_speed', default_value='0.25',
                              description='회전 속도[rad/s] — 느릴수록 SLAM 안정, 기본 0.25'),
        slam,
        mapper_proc,
    ])
