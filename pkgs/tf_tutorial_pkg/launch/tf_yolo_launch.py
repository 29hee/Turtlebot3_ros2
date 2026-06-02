import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    # 런치 인자 선언
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz(Rqt_image_view 실행 여부)'
    )


    return LaunchDescription([
        use_rviz_arg,
             
        Node (
            package='camera_pkg',
            executable='image_pub',
        ),

        Node (
            package = 'camera_pkg',
            executable = 'yolo_pub',
        ),

        Node(
            package= 'tf2_ros',
            executable= 'static_transform_publisher',
            arguments=['2.0', '0.0', '0.0',
                       '0', '0', '0',
                       'map', 'odom']
        ),

        Node(
            package= 'tf2_ros',
            executable= 'static_transform_publisher',
            arguments=['0.1', '0.0', '0.2',
                       '0', '0', '0',
                       'base_link', 'camera_link']
        ),

        Node (
            package = 'tf_tutorial_pkg',
            executable = 'odom_simulator',
        ),

        Node (
            package = 'tf_tutorial_pkg',
            executable = 'tf_yolo',
        ),

        Node (
            package = 'tf_tutorial_pkg',
            executable = 'tf_listener',
        ),

        Node (
            package= 'rviz2',
            executable= 'rviz2',
            arguments=['-d', '/home/hee/workspace/tf_tutorial.rviz'],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
