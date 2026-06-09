#!/usr/bin/env python3
"""
bringup.launch.py
색미로 '런타임 상시 구동' — 한 번 켜두면 RED/GREEN/BLUE 어떤 색이 와도 처리한다.
[수정 사양: 색을 launch 인자가 아니라 런타임 토픽(/target_color)으로 받는다]

구성(전부 색 무관, 계속 떠 있음):
  (옵션) gazebo + Burger 스폰        start_gazebo:=true  (시뮬 검증용)
  nav2_bringup bringup_launch.py     map_server + AMCL + planner/controller/bt
  color_confirm.py                   /target_color 로 대상 색 동적 전환, /target_confirmed 발행
  maze_tour.py (oneshot=false)       /target_color 받으면 그 색 순회 → /maze_done → 다시 대기

사용:
  # 1) 스택 상시 구동(시뮬)
  ros2 launch <경로>/bringup.launch.py
  #    실로봇:  start_gazebo:=false use_sim_time:=false relocalize:=true
  # 2) 색은 그때그때 토픽으로 지정(재시작 불필요):
  ros2 topic pub --once /target_color std_msgs/String "{data: RED}"
  ros2 topic pub --once /target_color std_msgs/String "{data: GREEN}"
  #    또는:  ros2 launch <경로>/mission.launch.py color:=BLUE
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess,
    RegisterEventHandler, Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)
    default_map = os.path.join(pkg, 'maps', 'color_room.yaml')
    default_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')
    color_confirm = os.path.join(pkg, 'scripts', 'color_confirm.py')
    maze_tour = os.path.join(pkg, 'scripts', 'maze_tour.py')
    # 매핑과 동일한 비전 스택 — 런타임에서 '숫자(digit)' 기반 목표 확인에 필수.
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    digit_recognizer = os.path.join(pkg, 'scripts', 'digit_recognizer.py')
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')
    mode_guard = os.path.join(pkg, 'scripts', 'mode_guard.py')
    scan_restamp = os.path.join(pkg, 'scripts', 'scan_restamp.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_gazebo = LaunchConfiguration('start_gazebo', default='false')
    # 실로봇: 시작 시 제자리 회전으로 자기위치부터 찾기. 시뮬은 set_initial_pose 라 false.
    # ★ 기본값을 start_gazebo 에 연동(실로봇=true, 시뮬=false). 명시 지정 시 우선.
    _reloc_default = PythonExpression(
        ["'true' if '", start_gazebo, "' == 'false' else 'false'"])
    relocalize = LaunchConfiguration('relocalize', default=_reloc_default)
    # 거꾸로 장착 카메라 보정(image_upright). 실로봇(start_gazebo:=false)에서만 동작.
    flip = LaunchConfiguration('flip', default='180')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    default_landmarks = os.path.join(pkg, 'maps', 'color_landmarks.yaml')
    landmarks = LaunchConfiguration('landmarks', default=default_landmarks)

    nav2_bringup = get_package_share_directory('nav2_bringup')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'color_maze.launch.py')),
        condition=IfCondition(start_gazebo),
    )
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

    # 색 무관: target_color 를 주지 않는다 → 둘 다 /target_color 를 구독해 대기.
    confirm_proc = ExecuteProcess(
        cmd=['python3', color_confirm, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    tour_proc = ExecuteProcess(
        cmd=['python3', maze_tour, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['relocalize:=', relocalize],
             '-p', ['landmarks_path:=', landmarks],
             '-p', 'oneshot:=false'],
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
    # ── 스캔 리스탬프 릴레이: /scan → /scan_synced(stamp=now) — Pi↔PC 클럭 skew 무시 ──
    #   amcl/costmap 이 /scan_synced 를 구독(nav2_maze.yaml) → stamp 드롭 없이 동작(시간동기 불필요).
    scan_restamp_proc = ExecuteProcess(
        cmd=['python3', scan_restamp, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen',
    )
    # ── 모드 가드: 매핑 스택(slam_toolbox/maze_explorer/color_mapper)이 떠 있으면 차단 ──
    guard_proc = ExecuteProcess(
        cmd=['python3', mode_guard, '--expect', 'runtime'], output='screen')

    def _guard_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='mode_guard: 매핑과 동시구동 충돌 — 런타임 시작 중단')]
        return []
    guard_handler = RegisterEventHandler(
        OnProcessExit(target_action=guard_proc, on_exit=_guard_exit))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_gazebo', default_value='false',
                              description='시뮬레이션이면 true, 실로봇이면 false(기본)'),
        DeclareLaunchArgument('relocalize', default_value=_reloc_default,
                              description='기본: 실로봇(start_gazebo:=false)=true, 시뮬=false. 시작 시 자기위치 추정'),
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(180|v|h). 실로봇 카메라 거꾸로면 180'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('landmarks', default_value=default_landmarks,
                              description='색 시맨틱맵(color_landmarks.yaml) 경로'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        guard_proc, guard_handler,
        gazebo,
        nav2,
        scan_restamp_proc,
        confirm_proc,
        tour_proc,
        upright_proc,
        vision_proc,
        digit_proc,
    ])
