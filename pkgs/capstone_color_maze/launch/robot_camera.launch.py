#!/usr/bin/env python3
"""
robot_camera.launch.py  — 로봇(Pi)에서 카메라를 '올바른 배선으로 한 번에' 띄운다.

[왜 필요한가]
그동안 카메라를 `ros2 run v4l2_camera v4l2_camera_node` 로 손수 띄우며 매번
`-r /image_raw:=/camera/image_raw_rot` 를 사람이 챙겨야 했다. 깜빡하면:
  · /camera/image_raw 에 v4l2 가 '거꾸로' 영상을 직접 쏨 → PC 의 image_upright 와
    발행자 2개가 되어 거꾸로/똑바로가 '랜덤'으로 섞임(보정 깨짐), 또는
  · image_upright 입력(_rot)이 0Hz → 색·숫자 인식 전멸.
→ 이 런치가 _rot remap 과 해상도/프레임레이트를 '박아' 그 실수를 원천 차단한다.

[배선] v4l2_camera → /camera/image_raw_rot (거꾸로) → [PC] image_upright → /camera/image_raw (똑바로)

사용(로봇 Pi):
    ros2 launch capstone_color_maze robot_camera.launch.py
    # 해상도/프레임 조정:  ... width:=320 height:=240   (저속 2.2Hz 개선용)

전제: Pi 에 ros-humble-v4l2-camera 설치. (이 런치는 카메라만 띄운다 — 로봇 bringup 은 별도.)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    # time_per_frame = [num, den] → den/num fps 목표. 기본 [1,15]=15fps 상한.
    tpf_num = LaunchConfiguration('tpf_num', default='1')
    tpf_den = LaunchConfiguration('tpf_den', default='15')

    # ★ 정수 배열로 평가되게 한다(문자열 배열이면 v4l2 가 거부). PythonExpression 이
    #   "[640,480]" 문자열을 파이썬 리스트 [640,480](int) 로 eval 한다.
    image_size = PythonExpression(["[", width, ",", height, "]"])
    time_per_frame = PythonExpression(["[", tpf_num, ",", tpf_den, "]"])

    cam = Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='v4l2_camera',
        output='screen',
        # ★ 핵심: 거꾸로 원본을 _rot 으로 보낸다(=/camera/image_raw 는 image_upright 전용).
        remappings=[('/image_raw', '/camera/image_raw_rot')],
        parameters=[{
            'image_size': image_size,        # [W,H] — 낮추면(예 320x240) Hz↑
            'time_per_frame': time_per_frame,
            'pixel_format': 'YUYV',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640',
                              description='가로 해상도(저속이면 320 권장)'),
        DeclareLaunchArgument('height', default_value='480',
                              description='세로 해상도(저속이면 240 권장)'),
        DeclareLaunchArgument('tpf_num', default_value='1'),
        DeclareLaunchArgument('tpf_den', default_value='15',
                              description='목표 fps 상한(den/num)'),
        cam,
    ])
