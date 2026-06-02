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

    config = os.path.join(
        get_package_share_directory('camera_pkg'),
        'config',
        'camera_params.yaml'
    )

    # img publisher 노드
    img_publisher = Node(
        package='camera_pkg',
        executable='image_pub',
        parameters=[config]
    )

    #rqt_image_view
    viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    #yolo publisher 노드
    yolo_pub = Node(
        package='camera_pkg',
        executable='yolo_pub',
        parameters=[config]
    )

    return LaunchDescription([
        use_rviz_arg,
        img_publisher,
        yolo_pub
        # img_yolo,
        # img_canny,
        # viewer_node,
        # graph_node,
        # img_pose
    ])