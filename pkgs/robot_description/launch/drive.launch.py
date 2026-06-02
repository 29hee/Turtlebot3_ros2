from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 장애물 감지: /scan → /is_obstacle
        Node(
            package='robot_description',
            executable='detect_things.py',
            output='screen'
        ),
        # 로봇 컨트롤러: /is_obstacle → /robot_command
        Node(
            package='robot_description',
            executable='control_robot.py',
            output='screen'
        ),
        # 이동 명령 실행: /robot_command → /cmd_vel
        Node(
            package='robot_description',
            executable='move_robot.py',
            output='screen'
        ),
    ])
