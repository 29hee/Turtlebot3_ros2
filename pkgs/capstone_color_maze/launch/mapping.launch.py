#!/usr/bin/env python3
"""
mapping.launch.py
color_maze.world 에서 TurtleBot3 로 SLAM(slam_toolbox) 매핑.

구성: gzserver(+world) + gzclient + robot_state_publisher + spawn + slam_toolbox + RViz

사용:
    export TURTLEBOT3_MODEL=burger        # 또는 burger_cam
    source /opt/ros/humble/setup.bash
    source /home/user/Workspace/turtlebot3_ws/install/setup.bash
    ros2 launch <경로>/mapping.launch.py

맵 저장:
    ros2 run nav2_map_server map_saver_cli -f <경로>/maps/color_maze
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

# ★ 매핑 때도 카메라(작품 색 감지)가 필요하므로 카메라 포함 모델을 강제한다.
#   표준 'burger' 는 카메라 sdf 가 없어 color_mapper 가 색을 못 본다. (color_maze.launch.py 와 동일)
ROBOT_MODEL = 'burger_cam'
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)
    world = os.path.join(pkg, 'worlds', 'color_room.world')
    wall_follower = os.path.join(pkg, 'scripts', 'wall_follower.py')
    scan_explorer = os.path.join(pkg, 'scripts', 'scan_explorer.py')
    maze_explorer = os.path.join(pkg, 'scripts', 'maze_explorer.py')
    color_mapper = os.path.join(pkg, 'scripts', 'color_mapper.py')
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    quality_monitor = os.path.join(pkg, 'scripts', 'quality_monitor.py')
    digit_recognizer = os.path.join(pkg, 'scripts', 'digit_recognizer.py')

    # 시뮬 여부. sim:=false 면 gazebo/spawn/robot_state_publisher 를 안 띄운다(실로봇용).
    #   실로봇은 로봇 bringup(Pi) + image_upright(PC) 가 /scan·/camera/image_raw·TF 를 이미 제공한다.
    sim = LaunchConfiguration('sim', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=sim)   # sim 따라감(실로봇=false)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    explore = LaunchConfiguration('explore', default='true')   # 자율 탐색+색매핑 동시 구동
    # 탐사기 선택: maze(색-반응 근접캡처+안티스턱, 권장) | scan(구 느린360°스캔) | wall(단순 벽타기)
    explorer = LaunchConfiguration('explorer', default='maze')
    # 거꾸로 장착 카메라 보정(image_upright 없이도). 실로봇에서 영상이 거꾸로면 flip:=true.
    #   영상 방향 확인: ros2 run rqt_image_view rqt_image_view /camera/image_raw
    flip = LaunchConfiguration('flip', default='false')
    # 가제보 GUI 창(gzclient) 표시 여부. 기본 false=안 띄움(물리 gzserver 는 그대로 동작).
    #   로봇 움직임은 RViz(맵+라이다+색마커)로 보면 충분. 굳이 가제보 창 보려면 gui:=true.
    gui = LaunchConfiguration('gui', default='false')
    # 종료는 본래 '미방문 소진'이지만 폭주 방지 시간 상한.
    duration = LaunchConfiguration('duration', default='600')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(sim),     # 실로봇(sim:=false)이면 안 띄움
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        # 시뮬 + gui:=true 일 때만 가제보 창 표시
        condition=IfCondition(PythonExpression(
            ["'", sim, "' == 'true' and '", gui, "' == 'true'"])),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(sim),     # 실로봇은 로봇 bringup 이 TF/rsp 제공 → 안 띄움
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        condition=IfCondition(sim),     # 실로봇엔 스폰 없음
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 색 라벨 누적(격자 투표 → color_landmarks.yaml)을 돕는 탐사 주행.
    #  explorer:=scan → scan_explorer(벽면 카메라 매핑용: 주기적 느린 360°회전으로 벽 face-on 스캔)
    #  explorer:=wall → wall_follower(단순 오른손 벽타기)
    maze_cond = PythonExpression(
        ["'", explore, "' == 'true' and '", explorer, "' == 'maze'"])
    scan_cond = PythonExpression(
        ["'", explore, "' == 'true' and '", explorer, "' == 'scan'"])
    wall_cond = PythonExpression(
        ["'", explore, "' == 'true' and '", explorer, "' == 'wall'"])
    maze_proc = ExecuteProcess(
        cmd=['python3', maze_explorer, '--duration', duration,
             '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(maze_cond), output='screen',
    )
    scan_proc = ExecuteProcess(
        cmd=['python3', scan_explorer, '--duration', duration],
        condition=IfCondition(scan_cond), output='screen',
    )
    wf_proc = ExecuteProcess(
        cmd=['python3', wall_follower, '--duration', duration],
        condition=IfCondition(wall_cond), output='screen',
    )
    # 단일 디코더 — 영상을 한 번만 풀어 /detected_color, /color_signal 발행(나머지가 구독).
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time], '-p', ['rotate_180:=', flip]],
        condition=IfCondition(explore), output='screen',
    )
    # color_mapper 는 '색+숫자 둘 다' 인식된 칸만 저장(무조건) → digit_recognizer 가 필수다.
    mapper_proc = ExecuteProcess(
        cmd=['python3', color_mapper, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(explore), output='screen',
    )
    # 매핑 중 라이브 품질 체크리스트(색별 벽수/digit/누락 경고).
    quality_proc = ExecuteProcess(
        cmd=['python3', quality_monitor],
        condition=IfCondition(explore), output='screen',
    )
    # 숫자 인식기(EasyOCR) — 색+숫자 둘 다 저장이 필수이므로 매핑에 '상시' 동반.
    #   /detected_digit 발행 → color_mapper 가 격자 digit 투표. (EasyOCR 미설치면 맵이 빈다.)
    digit_proc = ExecuteProcess(
        cmd=['python3', digit_recognizer, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time], '-p', ['rotate_180:=', flip]],
        condition=IfCondition(explore), output='screen',
    )

    return LaunchDescription([
        # 자식 프로세스(gzserver/스폰)도 카메라 모델을 상속받도록 런치 환경에 고정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='자율 탐색+색매핑 동시 구동(false=SLAM만)'),
        DeclareLaunchArgument('explorer', default_value='maze',
                              description='maze=색반응 근접캡처(권장) | scan=느린360°스캔 | wall=단순벽타기'),
        # 실로봇(sim:=false)은 카메라가 거꾸로 장착(직접 확인됨) → 자동 보정. 시뮬은 정방향.
        #   재장착 등으로 영상이 똑바르면 flip:=false 로 끌 것.
        DeclareLaunchArgument('flip', default_value=PythonExpression(
                                  ["'true' if '", sim, "' == 'false' else 'false'"]),
                              description='카메라 거꾸로면 true(숫자 OCR용 180° 보정). 실로봇 자동 true'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='true=시뮬(gazebo) | false=실로봇(gazebo/spawn/rsp 안 띄움)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='가제보 GUI 창 표시(기본 false=안 띄움, RViz 로 관찰)'),
        DeclareLaunchArgument('duration', default_value='600',
                              description='탐사 시간 상한[s] (종료는 미방문 소진이 우선)'),
        gzserver, gzclient, rsp, spawn, slam,
        vision_proc, maze_proc, scan_proc, wf_proc, mapper_proc, quality_proc, digit_proc,
    ])
