#!/usr/bin/env python3
"""
color_maze.launch.py
TurtleBot3 Burger 를 color_maze.world 에 스폰하는 런치.

사용 전:
    export TURTLEBOT3_MODEL=burger
실행:
    ros2 launch <이 파일 경로> color_maze.launch.py
    # 또는 패키지로 설치했다면: ros2 launch <pkg> color_maze.launch.py

turtlebot3_gazebo / turtlebot3_description 패키지가 설치돼 있어야 합니다.
robot 시작 위치는 미로 좌하단 셀 (-1.5, -1.5) 로 잡습니다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 이 런치 파일과 같은 트리의 worlds/color_maze.world 를 사용
    here = os.path.dirname(os.path.realpath(__file__))
    world = os.path.join(os.path.dirname(here), 'worlds', 'color_room.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
    ])
