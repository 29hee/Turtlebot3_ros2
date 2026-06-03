# 명령어 치트시트
> TurtleBot3 Burger · ROS2 Humble · AI 기반 자율주행 프로젝트

---

## 환경 설정

```bash
# ROS2 소스 (매 터미널마다 or ~/.bashrc에 추가)
source /opt/ros/humble/setup.bash
source ~/workspace/turtle_project/install/setup.bash

# TurtleBot3 모델 설정 (Burger)
export TURTLEBOT3_MODEL=burger

# 도메인 ID (같은 네트워크에서 충돌 방지 — 강의실 환경 확인)
export ROS_DOMAIN_ID=30

# ~/.bashrc에 한번에 추가
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
```

---

## colcon 빌드

```bash
# 전체 빌드
cd ~/workspace/turtle_project
colcon build

# 특정 패키지만 빌드
colcon build --packages-select robot_description

# 여러 패키지 동시 빌드
colcon build --packages-select robot_description my_robot_interfaces camera_pkg

# Python 패키지 — 수정 즉시 반영 (소스 설치)
colcon build --symlink-install --packages-select hee_lidar

# 빌드 후 반드시 소스
source install/setup.bash
```

---

## ROS2 기본 CLI

```bash
# 노드
ros2 node list
ros2 node info /노드이름

# 토픽
ros2 topic list
ros2 topic echo /토픽이름
ros2 topic info /토픽이름
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic hz /scan                        # 토픽 주파수 확인

# 서비스
ros2 service list
ros2 service call /서비스이름 인터페이스타입 "{args}"
ros2 service call /global_localization std_srvs/srv/Empty {}   # AMCL 초기화

# 액션
ros2 action list
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"

# 파라미터
ros2 param list
ros2 param get /노드이름 파라미터명
ros2 param set /노드이름 파라미터명 값

# 패키지 실행
ros2 run 패키지명 실행파일명
ros2 launch 패키지명 런치파일명.py
```

---

## TurtleBot3 실물 연결

```bash
# SSH 접속 (IP는 로봇 확인)
ssh ubuntu@192.168.x.x

# 실물 bringup (로봇 터미널에서)
ros2 launch turtlebot3_bringup robot.launch.py

# 키보드 원격 조종 (PC 터미널에서)
ros2 run turtlebot3_teleop teleop_keyboard

# 연결 확인
ros2 topic list          # /scan, /odom, /cmd_vel 보이면 OK
ros2 topic echo /scan    # LiDAR 데이터 확인
```

---

## Gazebo 시뮬레이션

```bash
# 기본 Gazebo + 로봇 스폰 (robot_description)
ros2 launch robot_description gazebo.launch.py

# 월드 파일 변경
ros2 launch robot_description gazebo.launch.py world:=slam_world.world

# 키보드 조종
ros2 run turtlebot3_teleop teleop_keyboard

# Gazebo 없이 RViz만 URDF 확인
ros2 launch robot_description display.launch.py
```

---

## SLAM — 지도 생성

```bash
# 1. Gazebo + SLAM 시작
ros2 launch robot_description slam.launch.py

# 2. 키보드로 주행하며 지도 완성
ros2 run turtlebot3_teleop teleop_keyboard

# 3. 지도 저장 (별도 터미널)
ros2 run nav2_map_server map_saver_cli -f ~/workspace/turtle_project/data/my_map
# → my_map.pgm + my_map.yaml 생성

# 실물 SLAM (로봇 bringup 후 PC에서)
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
```

---

## AMCL — 위치 추정

```bash
# Gazebo + 저장된 맵 로드 + AMCL
ros2 launch robot_description amcl.launch.py \
  map_yaml:=~/workspace/turtle_project/data/my_map.yaml

# 초기 포즈 강제 재추정 (전역 로컬라이제이션)
ros2 service call /global_localization std_srvs/srv/Empty {}

# RViz2에서 수동으로 초기 포즈 지정:
#   → "2D Pose Estimate" 버튼 클릭 후 지도에서 드래그
```

---

## Nav2 — 자율 주행

```bash
# Gazebo + AMCL + Nav2 전체 스택
ros2 launch robot_description nav2.launch.py \
  map_yaml:=~/workspace/turtle_project/data/my_map.yaml

# CLI로 목표 전송
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"

# Python BasicNavigator로 목표 전송 (별도 터미널)
ros2 run robot_description nav2_cmd.py

# 주행 취소
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{}" --cancel

# RViz2에서 목표 지정:
#   → "Nav2 Goal" 버튼 클릭 후 지도에서 드래그
```

---

## rosbag2 — 데이터 녹화/재생

```bash
# 녹화 (모든 토픽)
ros2 bag record -o ~/workspace/turtle_project/data/rosbag_session /scan /odom /cmd_vel /image_raw

# 특정 토픽만 녹화
ros2 bag record -o ~/workspace/turtle_project/data/scan_only /scan

# 재생
ros2 bag play ~/workspace/turtle_project/data/rosbag_session

# 빠르게/느리게 재생
ros2 bag play ~/workspace/turtle_project/data/rosbag_session --rate 0.5

# 정보 확인
ros2 bag info ~/workspace/turtle_project/data/rosbag_session
```

---

## TF2 — 좌표 변환

```bash
# TF 트리 PDF로 출력
ros2 run tf2_tools view_frames

# 두 프레임 간 실시간 변환 확인
ros2 run tf2_ros tf2_echo map base_link

# 정적 변환 발행 (x y z yaw pitch roll)
ros2 run tf2_ros static_transform_publisher 0.1 0.0 0.2 0 0 0 base_link camera_link

# TF 트리 텍스트 출력
ros2 run tf2_ros tf2_monitor
```

---

## 카메라 / YOLO

```bash
# 카메라 이미지 퍼블리시 + YOLO + Canny + rqt_image_view
ros2 launch py_launch_example example_bringup_launch.py

# 카메라 + YOLO + TF2 통합
ros2 launch tf_tutorial_pkg tf_yolo_launch.py

# YOLO + TF 연동 (use_rviz 옵션)
ros2 launch tf_tutorial_pkg tf_yolo_launch.py use_rviz:=true

# 카메라 파라미터 YAML 적용 버전
ros2 launch py_launch_example example_param_launch.py

# 이미지 토픽 확인
ros2 topic echo /image_raw --no-arr     # 메타데이터만
ros2 run rqt_image_view rqt_image_view  # GUI 뷰어
```

---

## 디버깅 도구

```bash
# 노드 토폴로지 시각화
ros2 run rqt_graph rqt_graph

# 토픽 값 실시간 플롯
ros2 run rqt_plot rqt_plot /scan/ranges[180]

# 로그 확인
ros2 run rqt_console rqt_console

# RViz2 실행
rviz2
rviz2 -d ~/workspace/turtle_project/pkgs/robot_description/rviz/slam.rviz

# 패키지 경로 확인
ros2 pkg prefix robot_description

# 인터페이스 타입 확인
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show nav2_msgs/action/NavigateToPose
```

---

## 패키지 생성

```bash
# Python 패키지
cd ~/workspace/co_project/pkgs
ros2 pkg create --build-type ament_python --node-name 노드이름 패키지이름

# C++ 패키지
ros2 pkg create --build-type ament_cmake 패키지이름

# 커스텀 인터페이스 패키지 (msg/srv/action)
ros2 pkg create --build-type ament_cmake 패키지이름_interfaces
```

---


## 🎨 미술관 휠체어 (capstone_color_maze) 실행

> 새 launch는 패키지(package.xml) 미등록 → **파일 경로로 직접 실행**. 아래 명령은 모두 `capstone_color_maze/` 디렉터리에서 실행.

### 0) 초기 source (새 터미널마다)
```bash
export TURTLEBOT3_MODEL=burger_cam                  # 표준 burger엔 카메라 없음 → 반드시 burger_cam
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash           # ← 본인 turtlebot3 워크스페이스 경로로 수정
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
cd <클론경로>/co_project/pkgs/capstone_color_maze   # ← 본인 클론 경로로 수정
```

### 1) 매핑 모드 — 색맵 구축 (개관 전 큐레이션)
```bash
ros2 launch launch/mapping.launch.py                          # gazebo(color_room)+SLAM+탐사+색매핑
ros2 run nav2_map_server map_saver_cli -f maps/color_room     # 점유격자맵 저장 (별도 터미널, 방 충분히 돈 뒤)
# color_landmarks.yaml 은 자동 누적. 필터 결과 확인:
python3 -c "import sys;sys.path.insert(0,'scripts');import yaml;from maze_common import resolve_target_walls;d=yaml.safe_load(open('maps/color_landmarks.yaml'));[print(c,len(resolve_target_walls(d,c)),'개') for c in('RED','GREEN','BLUE')]"
```

### 2) 서비스 모드 — 색 안내 주행 (개관 중)
```bash
ros2 launch launch/runtime.launch.py \
  target_color:=RED start_gazebo:=true \
  map:=$(pwd)/maps/color_room.yaml
# 확인: ros2 lifecycle get /amcl   → active [3]
# 정합 안 맞으면(라이다가 벽에 안 붙으면) RViz "2D Pose Estimate"로 로봇 위치 지정
# 색 변경 시: runtime 을 그 색으로 통째로 재실행 (maze_tour·color_confirm 자동 일치)
```

### 3) 전부 종료 (좀비 프로세스 정리)
```bash
for p in '[g]zserver' '[g]zclient' '[g]azebo' '[s]lam_toolbox' '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator' '[b]ehavior_server' '[s]moother_server' '[v]elocity_smoother' '[w]aypoint_follower' '[l]ifecycle_manager' '[m]ap_server' '[a]mcl' '[r]obot_state_publisher' '[r]viz2' '[w]all_follower' '[c]olor_mapper' '[m]aze_tour' '[c]olor_confirm'; do pkill -9 -f "$p"; done; ros2 daemon stop; ros2 daemon start
```


---

## 자주 쓰는 토픽 목록

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 360도 거리 데이터 |
| `/odom` | `nav_msgs/Odometry` | 바퀴 엔코더 기반 위치 |
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 (linear.x, angular.z) |
| `/image_raw` | `sensor_msgs/Image` | 웹캠 원본 이미지 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 지도 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL 추정 위치 |
| `/tf` | `tf2_msgs/TFMessage` | 좌표 프레임 변환 |
| `/imu` | `sensor_msgs/Imu` | IMU 가속도/자이로 |
