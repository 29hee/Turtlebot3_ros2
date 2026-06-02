import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hee_lidar')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'room_world.world')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'turtlebot.rviz')

    robot_description = Command(['xacro ', xacro_file])
    rviz_args = ['-d', rviz_file] if os.path.exists(rviz_file) else []

    return LaunchDescription([
        # Gazebo 시뮬레이터
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 로봇 TF 브로드캐스터
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Gazebo에 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'turtlebot', '-z', '0.3'],
            output='screen'
        ),

        # RViz 시각화
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
        ),

        # 장애물 감지: /scan → /is_obstacle
        Node(
            package='hee_lidar',
            executable='detect_things',
            output='screen'
        ),
        # 회전, 이동 명령 → /robot_command → /cmd_vel
        Node(
            package='hee_lidar',
            executable='move_robot',
            output='screen'
        ),
        # 로봇 컨트롤러: /is_obstacle → /robot_command
        Node(
            package='hee_lidar',
            executable='control_robot',
            output='screen'
        ),
    ])
