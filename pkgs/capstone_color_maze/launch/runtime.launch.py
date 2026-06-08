#!/usr/bin/env python3
"""
runtime.launch.py
색미로 '런타임' 단계: 저장된 맵 로드 → AMCL 로컬라이즈 → Nav2 → 색벽 순회/정지.
[수정 사양: 출구 없음, target 색 모든 벽 순회 후 마지막 확인 벽에서 정지 + /maze_done]

구성:
  (옵션) gazebo + Burger 스폰     start_gazebo:=true  (시뮬레이션 검증용)
  nav2_bringup bringup_launch.py  (map_server + AMCL + planner/controller/bt + lifecycle)
  color_confirm.py                (/target_confirmed: target 색 >=60% 프레임 점유)
  maze_tour.py                    (모든 target 벽 순회 → 마지막 확인 벽 정지 → /maze_done)

사용:
  export TURTLEBOT3_MODEL=burger        # 또는 burger_cam (카메라 포함 모델)
  source /opt/ros/humble/setup.bash
  source <turtlebot3_ws>/install/setup.bash
  ros2 launch <경로>/runtime.launch.py target_color:=RED
  # 실로봇: start_gazebo:=false (gazebo 띄우지 않음), use_sim_time:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)                      # capstone_color_maze/
    default_map = os.path.join(pkg, 'maps', 'color_room.yaml')   # 현재 월드(color_room) 재매핑 맵
    default_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')
    color_confirm = os.path.join(pkg, 'scripts', 'color_confirm.py')
    maze_tour = os.path.join(pkg, 'scripts', 'maze_tour.py')
    # 매핑과 동일한 비전 스택 — 런타임에서 '숫자(digit)' 기반 목표 확인에 필수.
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    digit_recognizer = os.path.join(pkg, 'scripts', 'digit_recognizer.py')
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')

    target_color = LaunchConfiguration('target_color', default='RED')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_gazebo = LaunchConfiguration('start_gazebo', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    # 실로봇: 순회 전에 제자리 회전으로 자기위치부터 찾기(AMCL 전역 재초기화+수렴).
    # 시뮬은 set_initial_pose 가 맞으므로 기본 false.
    relocalize = LaunchConfiguration('relocalize', default='false')
    # 거꾸로 장착 카메라 보정(image_upright). 실로봇(start_gazebo:=false)에서만 동작 —
    #   시뮬은 gazebo 가 /camera/image_raw 를 똑바로 발행하므로 불필요.
    flip = LaunchConfiguration('flip', default='180')

    nav2_bringup = get_package_share_directory('nav2_bringup')

    # (옵션) 시뮬레이션: world + Burger 스폰 (스폰 -2.0,-2.0 = nav2_maze.yaml AMCL 초기포즈와 일치)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'color_maze.launch.py')),
        condition=IfCondition(start_gazebo),
    )

    # 맵 로드 + AMCL + Nav2 (use_sim_time / map 은 RewrittenYaml 로 주입됨)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    # 비-패키지 스크립트는 ExecuteProcess 로 직접 실행(이 패키지 관례 유지)
    confirm_proc = ExecuteProcess(
        cmd=['python3', color_confirm, '--ros-args',
             '-p', ['target_color:=', target_color],
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    # 단발(oneshot): 지정한 한 색만 순회하고 종료. 연속 운영은 bringup.launch.py + mission.launch.py.
    tour_proc = ExecuteProcess(
        cmd=['python3', maze_tour, '--ros-args',
             '-p', ['target_color:=', target_color],
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['relocalize:=', relocalize],
             '-p', 'oneshot:=true'],
        output='screen',
    )
    # 거꾸로 장착 카메라를 똑바로 세워 /camera/image_raw 채움(실로봇 전용). 시뮬은 안 띄움.
    upright_proc = ExecuteProcess(
        cmd=['python3', image_upright, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['flip:=', flip], '-p', 'compressed_in:=false'],
        condition=IfCondition(PythonExpression(["'", start_gazebo, "' == 'false'"])),
        output='screen',
    )
    # 단일 디코더 — /color_signal(숫자 인식 근접게이트용) + /detected_color 발행.
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    # 숫자 인식기(EasyOCR) — /detected_digit 발행. '특정 숫자+색' 목표 확인에 필수.
    digit_proc = ExecuteProcess(
        cmd=['python3', digit_recognizer, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_color', default_value='RED',
                              description='RED | GREEN | BLUE (RED_2 처럼 숫자 지정 가능)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true',
                              description='시뮬레이션이면 true, 실로봇이면 false'),
        DeclareLaunchArgument('relocalize', default_value='false',
                              description='실로봇이면 true: 순회 전 제자리 회전으로 자기위치 추정'),
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(180|v|h). 실로봇 카메라 거꾸로면 180'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        gazebo,
        nav2,
        confirm_proc,
        tour_proc,
        upright_proc,
        vision_proc,
        digit_proc,
    ])
