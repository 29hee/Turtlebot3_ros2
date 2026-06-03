# color_maze — 색미로 테스트 월드 (TurtleBot3 Burger)

색 기반 SLAM/내비게이션 캡스톤 테스트용 Gazebo Classic 월드입니다.

## 구성
- **정사각형 4m × 4m** (내부 가용영역 x, y ∈ [-2, 2])
- **1m 격자 4×4** (셀 중심 = {-1.5, -0.5, 0.5, 1.5})
- **내부 벽 전부 동일 크기**: 1.0(L) × 0.15(T) × **0.5(H)** m
- **색상 3개만**: 🔴Red / 🟢Green / 🔵Blue
  - `y=-1` 가로 장벽 → **RED**  (오른쪽 col3 가 열림)
  - `y= 0` 가로 장벽 → **GREEN** (왼쪽 col0 가 열림)
  - `y= 1` 가로 장벽 → **BLUE** (오른쪽 col3 가 열림)
  - 외곽: bottom=RED, top=BLUE, left/right=GREEN

## 미로 풀이 (외길, serpentine)
```
 (위=BLUE 장벽)        goal ★
   +----+----+----+----+   row3
   |              ┌──────  ← BLUE 열림(col3)
   +----+----+----+    +   row2
   ──────┐              |  ← GREEN 열림(col0)
   +    +----+----+----+   row1
   |              ┌──────  ← RED 열림(col3)
   +----+----+----+    +   row0
 ▲ start(-1.5,-1.5)
```
경로: start → 우 → (1.5,-1.5) → 상(RED열림) → 좌 → (-1.5,-0.5)
→ 상(GREEN열림) → 우 → (1.5,0.5) → 상(BLUE열림) → goal(1.5,1.5)

> 벽 높이 0.5m > LiDAR(~0.18m)·카메라 높이 → 센서가 확실히 벽을 보고 색을 촬영.
> 코리도 폭 ≈ 1.0 − 0.15 = 0.85m, Burger(지름 ~0.14m)에 충분.

## 실행

### 1) 월드만 보기
```bash
gazebo --verbose worlds/color_maze.world
```

### 2) Burger 스폰까지
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch capstone_color_maze/launch/color_maze.launch.py
# 시작 위치 바꾸기: x_pose:=-1.5 y_pose:=-1.5
```
`turtlebot3_gazebo`, `turtlebot3_description`, `gazebo_ros` 패키지 필요.

## 다음 단계 (색 매핑 캡스톤)
1. LiDAR SLAM(`slam_toolbox`)으로 점유격자 생성
2. 카메라 HSV 분류 + LiDAR→이미지 투영(TF2)으로 벽 셀에 색 라벨 부여
3. 색 라벨을 free 공간으로 inflate → Nav2 **커스텀 ColorLayer** 코스트맵에 주입
4. "초록 장벽 통로로", "빨강 회피" 같은 **시맨틱 목표** 내비게이션 데모
