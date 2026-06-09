#!/usr/bin/env python3
"""
finalize.launch.py
2-pass 매핑의 'Phase 2' — 동결맵 로컬라이즈 + 정면 방문 + 숫자 확정(별도 실행).

[왜 별도 런치인가]  Phase1(mapping.launch two_pass:=true)은 SLAM 으로 맵을 만들며 색 좌표만
  찍는다. 그 맵을 '동결'해 저장한 뒤, 본 런치가 map_server 로 그 맵을 로드하고 AMCL 로
  로컬라이즈한 상태에서 Nav2 로 색좌표마다 정면 주행한다. SLAM 처럼 map→odom 이 루프클로저로
  툭툭 점프하지 않으므로(동결맵), 실로봇에서 Nav2 정면 주행이 안정적이다 → '사선/측면 접근으로
  숫자를 못 읽던' 문제를 정면 수직정렬로 제거한다.

구성:
  (옵션) gazebo + Burger 스폰        start_gazebo:=true  (시뮬 검증용)
  nav2_bringup bringup_launch.py     (map_server + AMCL + planner/controller/bt + lifecycle)
  vision_node + digit_recognizer     (정면 dwell 에서 색/숫자 인식)
  image_upright                      (실로봇: 거꾸로 카메라 보정)
  digit_finalizer                    (색좌표 정면 방문 → 수직정렬 → 숫자 확정 → landmarks 병합 저장)

핸드오프(같은 map_name 으로 Phase1 과 일치):
  입력  maps/<name>.yaml(.pgm)               동결 점유맵
        maps/<name>_landmarks.yaml           색 좌표(x,y,nx,ny — digit 없음)
  출력  maps/<name>_landmarks.yaml           digit 채워 갱신
        maps/<name>.pgm/yaml                  맵 재저장(확정)

사용:
  source /opt/ros/humble/setup.bash
  source <turtlebot3_ws>/install/setup.bash
  # 시뮬 검증
  ros2 launch <경로>/finalize.launch.py
  # 실로봇
  ros2 launch <경로>/finalize.launch.py start_gazebo:=false use_sim_time:=false
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

# 매핑과 동일하게 카메라 포함 모델(정면 dwell 에서 색/숫자를 봐야 함).
ROBOT_MODEL = 'burger_cam'
os.environ['TURTLEBOT3_MODEL'] = ROBOT_MODEL


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = os.path.dirname(here)                      # capstone_color_maze/
    maps_dir = os.path.join(pkg, 'maps')
    # Phase1(mapping.launch)과 '같은 이름'으로 점유맵 + 색좌표를 읽는다(파일 핸드오프 일치).
    map_name = LaunchConfiguration('map_name', default='color_room')
    default_map = [maps_dir + os.sep, map_name, '.yaml']
    landmarks = [maps_dir + os.sep, map_name, '_landmarks.yaml']
    map_save = [maps_dir + os.sep, map_name]                       # 확정 점유맵 재저장(확장자 없이)
    default_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')

    digit_finalizer = os.path.join(pkg, 'scripts', 'digit_finalizer.py')
    vision_node = os.path.join(pkg, 'scripts', 'vision_node.py')
    digit_recognizer = os.path.join(pkg, 'scripts', 'digit_recognizer.py')
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')
    mode_guard = os.path.join(pkg, 'scripts', 'mode_guard.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_gazebo = LaunchConfiguration('start_gazebo', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=default_params)
    # 실로봇은 시작위치를 모르므로 relocalize 기본 true(제자리 회전 → AMCL 수렴), 시뮬은 false.
    _reloc_default = PythonExpression(
        ["'true' if '", start_gazebo, "' == 'false' else 'false'"])
    relocalize = LaunchConfiguration('relocalize', default=_reloc_default)
    # 거꾸로 장착 카메라 보정(image_upright) — 실로봇(start_gazebo:=false)에서만.
    flip = LaunchConfiguration('flip', default='180')
    # 실시간 인식화면(디버그): vision_node(색 coverage/cx) + digit_recognizer(ROI 박스+'#숫자 conf')
    #   cv2 창을 띄운다. Phase2 디버깅용이라 기본 true. 끄려면 show:=false (DISPLAY 필요).
    show = LaunchConfiguration('show', default='true')

    nav2_bringup = get_package_share_directory('nav2_bringup')

    # (옵션) 시뮬: world + Burger 스폰 (runtime.launch 와 동일 검증 경로)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(here, 'color_maze.launch.py')),
        condition=IfCondition(start_gazebo),
    )

    # 동결맵 로드 + AMCL + Nav2 (map / use_sim_time 은 RewrittenYaml 로 주입)
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

    # 거꾸로 장착 카메라를 똑바로 세워 /camera/image_raw 채움(실로봇 전용). 시뮬은 안 띄움.
    upright_proc = ExecuteProcess(
        cmd=['python3', image_upright, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['flip:=', flip], '-p', 'compressed_in:=false'],
        condition=IfCondition(PythonExpression(["'", start_gazebo, "' == 'false'"])),
        output='screen',
    )
    # 단일 디코더 — /detected_color, /color_signal 발행. show:=true 면 색 인식 오버레이 창.
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['show:=', show]],
        output='screen',
    )
    # 숫자 인식기(EasyOCR) — /detected_digit 발행. 정면 dwell 에서 숫자를 읽힌다.
    #   show:=true 면 ROI 박스 + '#숫자 (conf)' 실시간 인식화면(Phase2 디버깅용).
    digit_proc = ExecuteProcess(
        cmd=['python3', digit_recognizer, '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['show:=', show]],
        output='screen',
    )
    # Phase2 코디네이터 — 색좌표마다 Nav2 정면 주행 → 라이다 수직정렬 → 숫자 확정 → landmarks 병합.
    #   wait_phase1:=false (별도 실행이라 /phase1_done 대기 없음). relocalize 는 실로봇만 기본 true.
    finalizer_proc = ExecuteProcess(
        cmd=['python3', digit_finalizer, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', 'wait_phase1:=false',
             '-p', ['relocalize:=', relocalize],
             '-p', ['map_save:='] + map_save,
             '-p', ['landmarks_path:='] + landmarks],
        output='screen',
    )

    # ── 모드 가드: 매핑 스택(slam_toolbox/maze_explorer/color_mapper)이 떠 있으면 차단 ──
    guard_proc = ExecuteProcess(
        cmd=['python3', mode_guard, '--expect', 'finalize'], output='screen')

    def _guard_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='mode_guard: 매핑과 동시구동 충돌 — Phase2(finalize) 시작 중단')]
        return []
    guard_handler = RegisterEventHandler(
        OnProcessExit(target_action=guard_proc, on_exit=_guard_exit))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true',
                              description='시뮬레이션이면 true, 실로봇이면 false'),
        DeclareLaunchArgument('relocalize', default_value=_reloc_default,
                              description='기본: 실로봇(start_gazebo:=false)=true, 시뮬=false. 시작 시 AMCL 수렴 회전'),
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(180|v|h). 실로봇 카메라 거꾸로면 180'),
        DeclareLaunchArgument('show', default_value='true',
                              description='실시간 인식화면(vision_node 색 + digit_recognizer 숫자 cv2 창). 끄려면 false'),
        DeclareLaunchArgument('map_name', default_value='color_room',
                              description='Phase1 과 같은 이름 — maps/<name>.yaml + maps/<name>_landmarks.yaml'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        guard_proc, guard_handler,
        gazebo,
        nav2,
        upright_proc,
        vision_proc,
        digit_proc,
        finalizer_proc,
    ])
