#!/usr/bin/env python3
"""
mission.launch.py
이미 떠 있는 bringup 스택에 '색 하나'를 지정하는 thin 런치.
/target_color 로 색을 한 번 발행할 뿐이다(스택을 새로 띄우지 않는다).

사용(bringup.launch.py 가 떠 있는 상태에서):
  ros2 launch <경로>/mission.launch.py color:=RED
  # 동등:  ros2 topic pub --once /target_color std_msgs/String "{data: RED}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    color = LaunchConfiguration('color', default='RED')
    # std_msgs/String YAML 인자: "{data: RED}"
    msg = PythonExpression(["'{data: ' + '", color, "' + '}'"])

    pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once',
             '/target_color', 'std_msgs/msg/String', msg],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('color', default_value='RED',
                              description='RED | GREEN | BLUE'),
        pub,
    ])
