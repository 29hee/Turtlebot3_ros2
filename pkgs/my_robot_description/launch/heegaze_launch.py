import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'room_world.world')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'gazebo.rviz')

    robot_description = Command(['xacro ', xacro_file])

    rviz_args = ['-d', rviz_file] if os.path.exists(rviz_file) else []

    lidar_turn_node = Node(
        package='my_robot_description',
        executable='lidar_turn',
        output='screen'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'turtlebot', '-z', '0.3'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
        ),
        lidar_turn_node
    ])
