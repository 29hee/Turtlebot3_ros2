# capstone_color_maze — 색 기반 시맨틱 내비게이션 (TurtleBot3 Burger)

미술관 안내 휠체어 컨셉의 색 기반 SLAM/내비게이션 캡스톤 패키지입니다.
방문객이 **색(RED/GREEN/BLUE)으로 전시 구역을 지정**하면, 로봇이 장애물을 피해
해당 색 벽(작품) 앞까지 순회·확인하고 **마지막 확인 벽에서 정지**합니다.

> 전체 설계·데이터 흐름·역할 분담은 [`ARCHITECTURE.md`](ARCHITECTURE.md) 참고.

## 환경 (`worlds/color_room.world`)
- **흰벽 5m × 5m 개방형 전시실** (벽 안쪽 가용영역 x, y ∈ [-2.5, 2.5])
- **RGB 단색 패널 9개** (색당 3개), 벽면에 부착 — 폭 1.0m × 두께 0.04m × 높이 0.5m
  - ground-truth 좌표 (`generate_world.py` 산출):
    - 🔴 RED:   (-1.50, -2.40), ( 2.40,  1.20), (-2.40,  0.80)
    - 🟢 GREEN: ( 0.00, -2.40), ( 2.40, -1.20), ( 1.00,  2.40)
    - 🔵 BLUE:  ( 1.50, -2.40), (-1.50,  2.40), (-2.40, -1.00)
- **무채색 원기둥 장애물 5개** (관람객·전시대 모사, 반경 0.12~0.18m)
- 로봇 스폰: **(-2.0, -2.0)** = `config/nav2_maze.yaml` 의 AMCL 초기포즈와 일치

> 벽/패널 높이 0.5m > LiDAR 스캔 평면(~0.18m)·카메라 높이 → 센서가 확실히 벽을 보고 색을 촬영.
> **실물 제작 시**: 무광·불투명 흰 벽 + 무광 R/G/B 패널, 높이 **최소 25~30cm 이상** 권장
> (LiDAR 스캔선 ~18cm를 여유 있게 가리도록). 투명·광택·순흑색 표면은 LiDAR 오검출 유발 → 회피.

## 파이프라인 (2단계)

### 1) 매핑 — 점유격자 + 색 시맨틱맵 생성
```bash
export TURTLEBOT3_MODEL=burger_cam        # 카메라 포함 모델(색 감지 필수)
source /opt/ros/humble/setup.bash
source <turtlebot3_ws>/install/setup.bash
ros2 launch launch/mapping.launch.py      # gazebo(color_room) + slam_toolbox + scan_explorer + color_mapper
# 충분히 돈 뒤 점유격자 저장:
ros2 run nav2_map_server map_saver_cli -f maps/color_room
# color_landmarks.yaml 은 color_mapper 가 자동 누적 저장.
```
> ⚠️ 매핑 주행 시 제자리 회전은 **느리게(0.3 rad/s)** — 빠르면 slam_toolbox 스캔매칭이
> 깨져 맵이 뒤틀린다. 재매핑 후 색 커버리지는 [`maps/REMAPPING.md`](maps/REMAPPING.md) 의
> ground-truth 체크리스트로 검증할 것(누락 패널 확인).

### 2) 런타임 — 색 지정 순회/정지
```bash
ros2 launch launch/runtime.launch.py target_color:=RED
# 실로봇: start_gazebo:=false use_sim_time:=false
```
- 저장된 맵(`maps/color_room.yaml`) 로드 → AMCL 로컬라이즈 → Nav2 주행
- `color_confirm.py` 가 target 색 프레임 점유율로 벽 도착 확인
- `maze_tour.py` 가 target 색 모든 벽 순회 → 마지막 확인 벽 정지 → `/maze_done` 발행

## 색 검출 (HSV)
- HSV 색 범위는 `scripts/maze_common.py` 의 `COLOR_RANGES` **단일 출처** (3개 노드가 import).
- R/G/B 는 Hue 가 ~120° 간격이라 넓게 열어도 안 겹침. 흰 벽은 채도(S)가 낮아
  `S_min` 으로 분리한다. 조명/카메라가 바뀌면 현장 프레임으로 `S_min` 부터 재보정.
- ⚠️ `CONFIRM_THRESHOLD` 는 현재 **0.30(임시)**, 사양 최종 목표는 **0.60** —
  실카메라 화각 + ROI/PID 정렬 적용 후 단계적으로 올리며 검증 예정(`maze_common.py` TODO).

## 의존 패키지
`turtlebot3_gazebo`, `turtlebot3_description`, `gazebo_ros`, `slam_toolbox`, `nav2_bringup`,
`cv_bridge`, `OpenCV`, `tf2_ros`.

## 순수 로직 테스트 (ROS2 불필요)
```bash
python3 -m pytest tests/         # 또는: python3 tests/test_maze_logic.py
```
