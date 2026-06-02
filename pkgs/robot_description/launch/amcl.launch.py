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
    declare_map_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/hee/workspace/ros2_ws/room_map.yaml',
        description='Map file name'
    )

    pkg_dir = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot.xacro')
    world_file = PathJoinSubstitution([pkg_dir, 'worlds', LaunchConfiguration('world')])
    rviz_file = os.path.join(pkg_dir, 'rviz', 'amcl.rviz')
    amcl_file = os.path.join(pkg_dir, 'config', 'amcl_params.yaml')

    robot_description = Command(['xacro ', xacro_file])

    rviz_args = ['-d', rviz_file] if os.path.exists(rviz_file) else []

    return LaunchDescription([
        declare_world_arg,
        declare_map_arg,
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
        Node(
            package='nav2_map_server',
            executable='map_server',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml'),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            parameters=[
                amcl_file,
                {'use_sim_time': True}
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'node_names': ['map_server', 'amcl'],
                'autostart': True,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=rviz_args,
            parameters=[{'use_sim_time': True}],
        ),
    ])


