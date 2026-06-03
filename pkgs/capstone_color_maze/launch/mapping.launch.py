#!/usr/bin/env python3
"""
mapping.launch.py
color_maze.world 에서 TurtleBot3 로 SLAM(slam_toolbox) 매핑.

구성: gzserver(+world) + gzclient + robot_state_publisher + spawn + slam_toolbox + RViz

사용:
    export TURTLEBOT3_MODEL=burger        # 또는 burger_cam
    source /opt/ros/humble/setup.bash
    source /home/user/Workspace/turtlebot3_ws/install/setup.bash
    ros2 launch <경로>/mapping.launch.py

맵 저장:
    ros2 run nav2_map_server map_saver_cli -f <경로>/maps/color_maze
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
    world = os.path.join(pkg, 'worlds', 'color_room.world')
    wall_follower = os.path.join(pkg, 'scripts', 'wall_follower.py')
    color_mapper = os.path.join(pkg, 'scripts', 'color_mapper.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    explore = LaunchConfiguration('explore', default='true')   # 자율 탐색+색매핑 동시 구동
    duration = LaunchConfiguration('duration', default='180')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 자율 탐색(오른손 벽타기) + 색 라벨 누적(격자 투표 → color_landmarks.yaml)
    wf_proc = ExecuteProcess(
        cmd=['python3', wall_follower, '--duration', duration],
        condition=IfCondition(explore), output='screen',
    )
    mapper_proc = ExecuteProcess(
        cmd=['python3', color_mapper, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(explore), output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='자율 탐색+색매핑 동시 구동(false=SLAM만)'),
        DeclareLaunchArgument('duration', default_value='180',
                              description='wall_follower 자율주행 시간[s]'),
        gzserver, gzclient, rsp, spawn, slam,
        wf_proc, mapper_proc,
    ])
