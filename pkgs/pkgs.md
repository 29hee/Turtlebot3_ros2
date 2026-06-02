# 패키지 목록

## 패키지 분류

| 종류 | 패키지 |
|------|--------|
| 튜토리얼 (연습용) | `hello_cmake_pkg`, `hello_ros2_pkg`, `my_robot_action`, `my_robot_service`, `py_launch_example` |
| 인터페이스 정의 | `my_robot_interfaces` |
| 카메라 / 비전 | `camera_pkg`, `tf_tutorial_pkg` |
| 로봇 URDF / Gazebo | `hee_lidar`, `my_robot_description`, `robot_description` |

---

## 튜토리얼 패키지

### hello_cmake_pkg
C++ ROS2 기본 퍼블리셔/서브스크라이버 연습용.

| 노드 | 역할 |
|------|------|
| `talker` | `std_msgs/String` 퍼블리시 |
| `listener` | `std_msgs/String` 서브스크라이브 |

런치파일 없음.

---

### hello_ros2_pkg
Python ROS2 기본 노드 연습용.

| 노드 | 역할 |
|------|------|
| `talker` | `std_msgs/String` 퍼블리시 |
| `listener` | `std_msgs/String` 서브스크라이브 |
| `square` | turtlesim 사각형 이동 |

런치파일 없음.

---

### my_robot_action
ROS2 Action 서버/클라이언트 연습용. 인터페이스는 `my_robot_interfaces/action/MoveRobot.action` 사용.

| 노드 | 역할 |
|------|------|
| `move_server` | Action 서버 — 목표 거리만큼 이동 처리 |
| `move_client` | Action 클라이언트 — 이동 목표 전송 |

런치파일 없음.

---

### my_robot_service
ROS2 Service 서버/클라이언트 연습용. 인터페이스는 `my_robot_interfaces/srv/` 사용.

| 노드 | 역할 |
|------|------|
| `add_server` | AddTwoInts 서비스 서버 |
| `add_client` | AddTwoInts 서비스 클라이언트 |
| `led_server` | LedControl 서비스 서버 |
| `led_client` | LedControl 서비스 클라이언트 |

런치파일 없음.

---

### py_launch_example
Launch 파일 작성법 연습용. 카메라 노드 묶음 실행.

| 런치파일 | 실행 내용 |
|----------|-----------|
| `example_bringup_launch.py` | 카메라 퍼블리셔 + YOLO + Canny 엣지 + 포즈 감지 + rqt_image_view |
| `example_param_launch.py` | `camera_params.yaml` 파라미터 로드 + 카메라 퍼블리셔 + YOLO 퍼블리셔 |

---

## 인터페이스 패키지

### my_robot_interfaces
커스텀 메시지 / 서비스 / 액션 타입 정의 모음. 다른 패키지에서 의존.

| 종류 | 이름 | 내용 |
|------|------|------|
| msg | `ObjectDetection` | YOLO 객체 1개 (label, confidence, bbox) |
| msg | `ObjectDetectionArray` | ObjectDetection 배열 |
| srv | `AddTwoInts` | 정수 2개 합산 |
| srv | `LedControl` | LED on/off 제어 |
| action | `MoveRobot` | 거리 목표 전달 → 피드백 + 결과 수신 |

---

## 카메라 / 비전 패키지

### camera_pkg
웹캠 영상 캡처 및 처리 노드 모음.

| 노드 | 역할 |
|------|------|
| `image_pub` | 웹캠 캡처 → `/image_raw` 퍼블리시 |
| `image_canny` | `/image_raw` → Canny 엣지 처리 → `/image_edge` |
| `image_yolo` | `/image_raw` → YOLOv8 객체 감지 → `/image_yolo` |
| `image_pose` | `/image_raw` → MediaPipe 포즈 감지 |
| `image_processor` | 이미지 처리 공통 노드 |
| `yolo_pub` | YOLO 결과를 `ObjectDetectionArray` 메시지로 퍼블리시 |

런치파일 없음 (py_launch_example, tf_tutorial_pkg에서 호출).

---

### tf_tutorial_pkg
TF2 좌표 변환 + YOLO 객체 위치 추정 실습.

| 노드 | 역할 |
|------|------|
| `odom_simulator` | 가상 오도메트리 생성 → `odom → base_link` TF 브로드캐스트 |
| `tf_yolo` | YOLO 감지 객체의 TF 좌표 브로드캐스트 |
| `tf_listener` | 특정 TF 프레임 간 거리/위치 출력 |

| 런치파일 | 실행 내용 |
|----------|-----------|
| `tf_yolo_launch.py` | 카메라 + YOLO + odom 시뮬레이터 + TF 브로드캐스터 + TF 리스너 |

---

## 로봇 URDF / Gazebo 패키지

### hee_lidar
LiDAR 기반 장애물 회피 실습 (개인 커스텀 URDF 포함).

| 노드 | 역할 |
|------|------|
| `detect_things` | `/scan` → 전방 장애물 감지 → `/is_obstacle` |
| `move_robot` | `/robot_command` → `/cmd_vel` |
| `control_robot` | `/is_obstacle` → 회피 전략 결정 → `/robot_command` |

| 런치파일 | 실행 내용 |
|----------|-----------|
| `heegaze_launch.py` | Gazebo(room_world) + 로봇 스폰 + RViz + 장애물 회피 3노드 |

---

### my_robot_description
로봇 URDF 제작 실습. 기본 turtlebot + 커스텀 heeturtle URDF 포함.

| 노드 | 역할 |
|------|------|
| `lidar_turn` | LiDAR 데이터로 회전 주행 |

| 런치파일 | 실행 내용 |
|----------|-----------|
| `display_launch.py` | turtlebot URDF → RViz + joint_state_publisher_gui (URDF 확인용) |
| `hee_display_launch.py` | heeturtle 커스텀 URDF → RViz + joint_state_publisher_gui + rqt_graph |
| `heegaze_launch.py` | Gazebo(room_world) + 로봇 스폰 + RViz + lidar_turn 노드 |
| `slam.launch.py` | Gazebo + slam_toolbox(online_async) → 지도 생성 |

---

### robot_description
가장 완성도 높은 메인 패키지. URDF, Gazebo, SLAM, Nav2, PID 팔 제어까지 포함.

| 노드 | 역할 |
|------|------|
| `detect_things.py` | `/scan` → 장애물 감지 → `/is_obstacle` |
| `control_robot.py` | `/is_obstacle` → 회피 전략 → `/robot_command` |
| `move_robot.py` | `/robot_command` → `/cmd_vel` |
| `lidar_navigator.py` | LiDAR 기반 자율 주행 |
| `nav2_cmd.py` | Nav2 목표 지점 전송 (별도 터미널 실행) |
| `pid_arm_control.py` | 1-DOF 로봇 팔 PID 토크 제어 |

| 런치파일 | 실행 내용 |
|----------|-----------|
| `display.launch.py` | turtlebot URDF → RViz + joint_state_publisher_gui (URDF 확인용) |
| `gazebo.launch.py` | Gazebo(room_world) + 로봇 스폰 + RViz |
| `drive.launch.py` | 장애물 회피 3노드만 실행 (Gazebo 별도 필요) |
| `slam.launch.py` | Gazebo + slam_toolbox → 지도 생성 |
| `amcl.launch.py` | Gazebo + 저장된 맵 로드 + AMCL 위치 추정 |
| `nav2.launch.py` | Gazebo + AMCL + Nav2 자율 주행 (`nav2_cmd.py`는 별도 실행) |
| `pid_arm_launch.py` | 1-DOF 팔 Gazebo 스폰 + ros2_control + PID effort 제어 |
