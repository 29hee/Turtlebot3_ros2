import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Get the path to the URDF file
    pkg_dir = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'turtlebot.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'turtlebot.rviz')

    # Process the xacro file to get the robot description
    robot_description_content = Command(['xacro ', xacro_file])

    use_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Create a node to publish the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
        # parameters=[{
        #     'use_sim_time': LaunchConfiguration('use_sim_time')
        # }]
    )

    # Create a node to launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])