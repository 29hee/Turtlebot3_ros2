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
    SetEnvironmentVariable, RegisterEventHandler, Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    image_upright = os.path.join(pkg, 'scripts', 'image_upright.py')
    mode_guard = os.path.join(pkg, 'scripts', 'mode_guard.py')
    digit_finalizer = os.path.join(pkg, 'scripts', 'digit_finalizer.py')
    nav2_params = os.path.join(pkg, 'config', 'nav2_maze.yaml')

    # 시뮬 여부. sim:=false 면 gazebo/spawn/robot_state_publisher 를 안 띄운다(실로봇용).
    #   실로봇은 로봇 bringup(Pi) + image_upright(PC) 가 /scan·/camera/image_raw·TF 를 이미 제공한다.
    sim = LaunchConfiguration('sim', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default=sim)   # sim 따라감(실로봇=false)
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    explore = LaunchConfiguration('explore', default='true')   # 자율 탐색+색매핑 동시 구동
    # 탐사기 선택: maze(색-반응 근접캡처+안티스턱, 권장) | scan(구 느린360°스캔) | wall(단순 벽타기)
    explorer = LaunchConfiguration('explorer', default='maze')
    # 거꾸로 장착 카메라 보정을 'image_upright 한 곳'에서만 한다(실로봇=sim:=false 일 때).
    #   flip = image_upright 회전 모드(180|v|h). 우리 버거는 180. 카메라가 똑바르면 none.
    #   배선: v4l2(→ /camera/image_raw_rot) → image_upright(회전) → /camera/image_raw(똑바름)
    #         → vision_node·digit_recognizer 는 회전 없이 이걸 구독(이중회전 방지).
    flip = LaunchConfiguration('flip', default='180')
    # 가제보 GUI 창(gzclient) 표시 여부. 기본 false=안 띄움(물리 gzserver 는 그대로 동작).
    #   로봇 움직임은 RViz(맵+라이다+색마커)로 보면 충분. 굳이 가제보 창 보려면 gui:=true.
    gui = LaunchConfiguration('gui', default='false')
    # 종료는 본래 '미방문 소진'이지만 폭주 방지 시간 상한.
    duration = LaunchConfiguration('duration', default='600')
    # 저장 이름(버전). 점유맵 + 색좌표를 '한 이름'으로 묶어 저장 → 같은 SLAM 좌표 한 쌍.
    #   예: map_name:=run2 → maps/run2.pgm/yaml + maps/run2_landmarks.yaml (기존 안 덮음).
    maps_dir = os.path.join(pkg, 'maps')
    map_name = LaunchConfiguration('map_name', default='color_room')
    map_save = [maps_dir + os.sep, map_name]                       # 점유맵 경로(확장자 없이)
    landmarks = [maps_dir + os.sep, map_name, '_landmarks.yaml']   # 색좌표 경로
    # 매핑 종료 품질 게이트: 자연 종료 시 색+숫자 벽이 이 수 미만이면 재탐사(0=끔).
    min_walls = LaunchConfiguration('min_walls', default='1')
    # 매핑 방식: false=단일패스(접근 중 ALIGN 정면정렬 → 인식 잘 됨, 기본/권장)
    #   true=2패스(Phase1 탐사+색좌표 → Phase2 Nav2 정면방문). 단일패스가 색 인식이 더 좋다.
    two_pass = LaunchConfiguration('two_pass', default='false')
    # 벽타기 시 오른쪽 벽 유지거리[m] — 클수록 벽과 멀리 돈다.
    wall_dist = LaunchConfiguration('wall_dist', default='0.6')
    # color_mapper require_digit = NOT two_pass (2-pass Phase1 은 색만 저장).
    require_digit = PythonExpression(["'false' if '", two_pass, "' == 'true' else 'true'"])

    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    nav2_bringup = get_package_share_directory('nav2_bringup')

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
             '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['min_quality_walls:=', min_walls],
             '-p', ['two_pass:=', two_pass],
             '-p', ['target_right:=', wall_dist],
             '-p', ['landmarks_path:='] + landmarks],
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
    # 카메라 상하반전 보정 — '한 곳에서만'(실로봇). v4l2 의 _rot(거꾸로)을 받아 똑바로 세워
    #   /camera/image_raw 를 채운다. compressed_in=false(raw _rot 구독, 안정). 대역폭 더 줄이려면
    #   Pi 에 compressed_image_transport 깔고 compressed_in:=true.
    upright_proc = ExecuteProcess(
        cmd=['python3', image_upright, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['flip:=', flip], '-p', 'compressed_in:=false'],
        condition=IfCondition(PythonExpression(["'", sim, "' == 'false'"])), output='screen',
    )
    # 단일 디코더 — 영상을 한 번만 풀어 /detected_color, /color_signal 발행(나머지가 구독).
    #   회전은 image_upright 가 끝냈으니 여기선 rotate_180 안 함(기본 false).
    vision_proc = ExecuteProcess(
        cmd=['python3', vision_node, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(explore), output='screen',
    )
    # color_mapper — 2-pass(require_digit=false): Phase1 은 색 좌표만 저장(Phase2 가 숫자 채움).
    #   단일패스(require_digit=true): 색+숫자 둘 다인 칸만 저장.
    mapper_proc = ExecuteProcess(
        cmd=['python3', color_mapper, '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['require_digit:=', require_digit],
             '-p', ['save_path:='] + landmarks],
        condition=IfCondition(explore), output='screen',
    )
    # 매핑 중 라이브 품질 체크리스트(색별 벽수/digit/누락 경고).
    quality_proc = ExecuteProcess(
        cmd=['python3', quality_monitor, '--ros-args',
             '-p', ['landmarks_path:='] + landmarks],
        condition=IfCondition(explore), output='screen',
    )
    # 숫자 인식기(EasyOCR) — 색+숫자 둘 다 저장이 필수이므로 매핑에 '상시' 동반.
    #   /detected_digit 발행 → color_mapper 가 격자 digit 투표. (EasyOCR 미설치면 맵이 빈다.)
    digit_proc = ExecuteProcess(
        cmd=['python3', digit_recognizer, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        condition=IfCondition(explore), output='screen',
    )
    # ── 2-pass Phase2 용 ── Nav2 navigation(planner/controller/bt) — slam 과 공존(AMCL/map_server
    #   없음: /map 과 map→odom 은 slam_toolbox 가 제공). two_pass 일 때만.
    nav2_cond = PythonExpression(["'", explore, "' == 'true' and '", two_pass, "' == 'true'"])
    nav2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params,
                          'autostart': 'true'}.items(),
        condition=IfCondition(nav2_cond),
    )
    # Phase2 코디네이터 — /phase1_done 받으면 색좌표마다 Nav2 정면 주행 + 수직정렬 + 숫자확정 → 맵저장.
    finalizer_proc = ExecuteProcess(
        cmd=['python3', digit_finalizer, '--ros-args',
             '-p', ['use_sim_time:=', use_sim_time],
             '-p', ['map_save:='] + map_save,
             '-p', ['landmarks_path:='] + landmarks],
        condition=IfCondition(nav2_cond), output='screen',
    )

    # ── 탐사 종료 → 점유격자맵 자동저장 (맵 핸드오프 자동화) ──────────────────
    #   탐사기(maze/scan/wall 중 실행된 것)가 끝나면 map_saver_cli 로 /map 을 저장한다.
    #   '미방문 소진/시간상한'으로 정상 종료될 때 저장됨. (수동 map_saver 깜빡 방지.)
    def _map_saver():
        return ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_save,
                 '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
            output='screen',
        )
    # ── 모드 가드: 런타임 스택(AMCL/Nav2/maze_tour)이 떠 있으면 매핑 시작 차단 ──
    guard_proc = ExecuteProcess(
        cmd=['python3', mode_guard, '--expect', 'mapping'], output='screen')

    def _guard_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='mode_guard: 런타임과 동시구동 충돌 — 매핑 시작 중단')]
        return []
    guard_handler = RegisterEventHandler(
        OnProcessExit(target_action=guard_proc, on_exit=_guard_exit))

    save_on_maze = RegisterEventHandler(
        OnProcessExit(target_action=maze_proc, on_exit=[_map_saver()]))
    save_on_scan = RegisterEventHandler(
        OnProcessExit(target_action=scan_proc, on_exit=[_map_saver()]))
    save_on_wall = RegisterEventHandler(
        OnProcessExit(target_action=wf_proc, on_exit=[_map_saver()]))

    return LaunchDescription([
        # 자식 프로세스(gzserver/스폰)도 카메라 모델을 상속받도록 런치 환경에 고정
        SetEnvironmentVariable('TURTLEBOT3_MODEL', ROBOT_MODEL),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('explore', default_value='true',
                              description='자율 탐색+색매핑 동시 구동(false=SLAM만)'),
        DeclareLaunchArgument('explorer', default_value='maze',
                              description='maze=색반응 근접캡처(권장) | scan=느린360°스캔 | wall=단순벽타기'),
        # image_upright 회전 모드(실로봇만 동작). 우리 버거 카메라는 거꾸로(직접 확인) → 180.
        #   카메라가 똑바르면 flip:=none.
        DeclareLaunchArgument('flip', default_value='180',
                              description='image_upright 회전(180|v|h|none). 실로봇 카메라 거꾸로면 180'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='true=시뮬(gazebo) | false=실로봇(gazebo/spawn/rsp 안 띄움)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='가제보 GUI 창 표시(기본 false=안 띄움, RViz 로 관찰)'),
        DeclareLaunchArgument('duration', default_value='600',
                              description='탐사 시간 상한[s] (종료는 미방문 소진이 우선)'),
        DeclareLaunchArgument('map_name', default_value='color_room',
                              description='저장 이름 — maps/<name>.pgm/yaml + maps/<name>_landmarks.yaml (버전 분리용)'),
        DeclareLaunchArgument('min_walls', default_value='1',
                              description='매핑 종료 품질 게이트: 색+숫자 벽 최소수(미달이면 재탐사, 0=끔)'),
        DeclareLaunchArgument('two_pass', default_value='false',
                              description='false=단일패스(ALIGN 정면정렬, 기본/권장) | true=2패스(Nav2 Phase2)'),
        DeclareLaunchArgument('wall_dist', default_value='0.6',
                              description='벽타기 오른쪽 벽 유지거리[m] (클수록 벽과 멀리)'),
        guard_proc, guard_handler,
        gzserver, gzclient, rsp, spawn, slam,
        upright_proc, vision_proc, maze_proc, scan_proc, wf_proc, mapper_proc, quality_proc, digit_proc,
        nav2_nav, finalizer_proc,
        save_on_maze, save_on_scan, save_on_wall,
    ])
