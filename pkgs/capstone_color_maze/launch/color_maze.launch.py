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
robot 시작 위치는 전시실 좌하단 (-2.0, -2.0) 으로 잡습니다(nav2_maze.yaml AMCL 초기포즈와 일치).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ★ 카메라가 가제보와 함께 항상 켜지도록 카메라 포함 모델을 기본으로 강제한다.
#   표준 'burger' 에는 카메라 sdf 가 없어 /camera/image_raw 발행자가 0 → color_confirm 이
#   영원히 0% 가 된다. 사용자가 셸에서 burger_cam 을 export 안 해도 카메라가 뜨도록 박아둔다.
ROBOT_MODEL = 'burger_cam'
# 이 프로세스에서 뒤이어 평가되는 tb3 런치(robot_state_publisher / spawn)가 os.environ 을
# 읽으므로, 액션 실행 전에 즉시 반영되도록 여기서도 직접 설정한다.
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


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
        # 자식 프로세스(gzserver/스폰)도 카메라 모델을 상속받도록 런치 환경에 고정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
    ])
