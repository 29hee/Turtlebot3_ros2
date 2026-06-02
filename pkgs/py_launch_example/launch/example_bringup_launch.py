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
        default_value='true',
        description='Whether to launch RViz(Rqt_image_view 실행 여부)'
    )

    # img publisher 노드
    img_publisher = Node(
        package='camera_pkg',
        executable='image_pub'
    )

    # image_yolo
    img_yolo = Node(
        package='camera_pkg',
        executable='image_yolo'
    )

    # image_edge
    img_canny = Node(
        package='camera_pkg',
        executable='image_canny'
    )

    #rqt_image_view
    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # rqt_graph
    graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    #image_pose
    img_pose = Node(
        package='camera_pkg',
        executable='image_pose'
    )

    return LaunchDescription([
        use_rviz_arg,
        img_publisher,
        img_yolo,
        img_canny,
        viewer_node,
        graph_node,
        img_pose
    ])