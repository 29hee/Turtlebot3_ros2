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
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# ★ 매핑 때도 카메라(작품 색 감지)가 필요하므로 카메라 포함 모델을 강제한다.
#   표준 'burger' 는 카메라 sdf 가 없어 color_mapper 가 색을 못 본다. (color_maze.launch.py 와 동일)
ROBOT_MODEL = 'burger_cam'
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)
    world = os.path.join(pkg, 'worlds', 'color_room.world')
    wall_follower = os.path.join(pkg, 'scripts', 'wall_follower.py')
    scan_explorer = os.path.join(pkg, 'scripts', 'scan_explorer.py')
    color_mapper = os.path.join(pkg, 'scripts', 'color_mapper.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    explore = LaunchConfiguration('explore', default='true')   # 자율 탐색+색매핑 동시 구동
    # 탐사기 선택: scan(벽면 카메라 매핑용, 주기적 느린 360°회전) | wall(단순 벽타기)
    explorer = LaunchConfiguration('explorer', default='scan')
    # 느린 회전(스캔당 ~24s) 탓에 300s 면 둘레 60%만 돔(북쪽 벽 누락). 600s 로 완주 보장.
    duration = LaunchConfiguration('duration', default='660')

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

    # 색 라벨 누적(격자 투표 → color_landmarks.yaml)을 돕는 탐사 주행.
    #  explorer:=scan → scan_explorer(벽면 카메라 매핑용: 주기적 느린 360°회전으로 벽 face-on 스캔)
    #  explorer:=wall → wall_follower(단순 오른손 벽타기)
    scan_cond = PythonExpression(
        ["'", explore, "' == 'true' and '", explorer, "' == 'scan'"])
    wall_cond = PythonExpression(
        ["'", explore, "' == 'true' and '", explorer, "' == 'wall'"])
    scan_proc = ExecuteProcess(
        cmd=['python3', scan_explorer, '--duration', duration],
        condition=IfCondition(scan_cond), output='screen',
    )
    wf_proc = ExecuteProcess(
        cmd=['python3', wall_follower, '--duration', duration],
        condition=IfCondition(wall_cond), output='screen',
    )
    mapper_proc = ExecuteProcess(
        cmd=['python3', color_mapper, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(explore), output='screen',
    )

    return LaunchDescription([
        # 자식 프로세스(gzserver/스폰)도 카메라 모델을 상속받도록 런치 환경에 고정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='자율 탐색+색매핑 동시 구동(false=SLAM만)'),
        DeclareLaunchArgument('explorer', default_value='scan',
                              description='scan=느린360°회전 벽면스캔 | wall=단순 벽타기'),
        DeclareLaunchArgument('duration', default_value='660',
                              description='탐사 주행 시간[s] (촘촘한 스핀 완주엔 660s 권장)'),
        gzserver, gzclient, rsp, spawn, slam,
        scan_proc, wf_proc, mapper_proc,
    ])
