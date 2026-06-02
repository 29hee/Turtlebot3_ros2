import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='room_world.world',
        description='Gazebo world file name'
    )

    pkg_dir = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot.xacro')
    world_file = PathJoinSubstitution([pkg_dir, 'worlds', LaunchConfiguration('world')])
    rviz_file = os.path.join(pkg_dir, 'rviz', 'slam.rviz')

    robot_description = Command(['xacro ', xacro_file])

    rviz_args = ['-d', rviz_file] if os.path.exists(rviz_file) else []

    # SLAM Toolbox 추가
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                slam_toolbox_dir,
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'slam_params_file': os.path.join(
                pkg_dir,
                'config',
                'slam_param.yaml'
            )
        }.items()
    )

    return LaunchDescription([
        declare_world_arg,
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'turtlebot', '-z', '0.3'],
            output='screen'
        ),
        slam_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
        ),
    ])