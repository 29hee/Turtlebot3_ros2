# 실로봇 실행 가이드 (TurtleBot3 Burger + 카메라)

자율주행 SLAM 매핑 → 색 안내 런타임까지 실로봇 전체 명령. 시뮬은 `README.md` 참고.

> **먼저 본인 환경에 맞게 바꿀 값**
> `<로봇IP>` · `<워크스페이스>`(turtlebot3_ws) · `<클론경로>`(co_project 위치) · `ROS_DOMAIN_ID`(로봇·PC 동일)
> **카메라 토픽은 반드시 `/camera/image_raw`** (색 노드가 이걸 구독).

---

## 0) 공통 환경 — 모든 터미널(로봇·PC 양쪽)
```bash
export TURTLEBOT3_MODEL=burger_cam
export ROS_DOMAIN_ID=30                       # 로봇과 PC 같은 값!
source /opt/ros/humble/setup.bash
source <워크스페이스>/install/setup.bash       # 예: ~/turtlebot3_ws/install/setup.bash
```

## 1) 로봇(라즈베리파이) 측 — SSH
```bash
ssh ubuntu@<로봇IP>
# (0번 환경 source 후)
ros2 launch turtlebot3_bringup robot.launch.py          # 라이다 + 모터 + odom TF

# 카메라 노드 (별도 터미널) — 토픽명을 /camera/image_raw 로 맞춤:
ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw
#   ↑ USB캠 기준. Pi캠이면 해당 노드. 토픽만 동일하게.
```

---

## 2) PC — Phase 1: 자율주행 SLAM 매핑 (한 번)
> 시뮬 맵 재사용 금지 → 실제 공간을 새로 매핑. `scan_explorer`가 **자율로 로봇을 움직이므로**
> 매핑 중엔 **주변에 사람을 비우고, Ctrl-C(비상정지) 대기**할 것.

```bash
cd <클론경로>/co_project/pkgs/capstone_color_maze

# (터미널 A) SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# (터미널 B) 색 누적 매퍼
python3 scripts/color_mapper.py --ros-args -p use_sim_time:=false

# (터미널 C) ★ 자율 탐사 주행 (텔레옵 대신) — 둘레 벽타기 + 주기적 느린 360° 스핀
python3 scripts/scan_explorer.py --duration 480 --perimeter-frac 1.0 --spin-speed 0.3
#   --perimeter-frac 1.0 : 내부 웨이포인트(시뮬 방 전용 좌표) 안 씀 → 어떤 방이든 generic.
#   --spin-speed 0.3     : 회전은 느리게(빠르면 SLAM 맵이 뒤틀림).
#   --duration 480       : 방 크기에 맞게 조정(작은 방 300, 큰 방 600+).
#   끝나면 자동 정지. 중간에 멈추려면 Ctrl-C.

# (터미널 D) 보면서 (맵 + 라이다 + 카메라 + 색 마커)
rviz2 -d config/maze.rviz

# (터미널 E) 충분히 돌았으면 점유격자 저장
ros2 run nav2_map_server map_saver_cli -f maps/color_room

# 색맵 결과 확인(색당 벽 개수 — RGB 각 3개 나와야 이상적)
python3 -c "import sys;sys.path.insert(0,'scripts');import yaml;from maze_common import resolve_target_walls;d=yaml.safe_load(open('maps/color_landmarks.yaml'));[print(c,len(resolve_target_walls(d,c)),'개') for c in('RED','GREEN','BLUE')]"
```
> 누락된 색 벽이 있으면 그 구역을 더 보도록 `--duration`을 늘려 재매핑.
> 벽타기 거리가 안 맞으면(벽에 너무 붙거나 멀면) `scan_explorer.py`의 `target_right`/`front_stop` 조정.

---

## 3) PC — Phase 2: 런타임 색 안내 (상시 구동)
```bash
cd <클론경로>/co_project/pkgs/capstone_color_maze

# (터미널 A) 스택 상시 구동 — 실로봇: gazebo 안 띄움 / 실시간 / 시작 시 자기위치추정
ros2 launch launch/bringup.launch.py \
    start_gazebo:=false use_sim_time:=false relocalize:=true \
    map:=$(pwd)/maps/color_room.yaml \
    landmarks:=$(pwd)/maps/color_landmarks.yaml
#   relocalize:=true → 시작 시 제자리 회전으로 위치 수렴.
#   (또는/추가로 RViz "2D Pose Estimate"로 초기 위치를 찍어줘도 됨)

# (터미널 B) 보면서
rviz2 -d config/maze.rviz

# (터미널 C) 색 지정 — 재시작 없이 반복
ros2 launch launch/mission.launch.py color:=RED
ros2 launch launch/mission.launch.py color:=GREEN
ros2 launch launch/mission.launch.py color:=BLUE
#   동등:  ros2 topic pub --once /target_color std_msgs/msg/String "{data: RED}"

# (터미널 D) 도착 신호
ros2 topic echo /maze_done
```

---

## 4) 점검 / 디버그
> 대부분 별도 터미널에서 그때그때 띄워 확인. 점검 순서: **센서 입력 → 자기위치 → 색 인지**.

```bash
# ── 라이다 정상? ─────────────────────────────────────────────
ros2 topic hz /scan
#   /scan 발행 주파수. 약 5Hz(LDS-01/02) 나오면 정상.
#   안 뜨면 → 로봇 bringup(로봇-T1) 미기동 or ROS_DOMAIN_ID 불일치.

# ── 카메라 정상? ─────────────────────────────────────────────
ros2 topic hz /camera/image_raw
#   카메라 프레임 주파수(보통 10~30Hz). 안 뜨면 → 카메라 노드 없음 or 토픽명 불일치
#   (로봇-T2 의 -r /image_raw:=/camera/image_raw remap 확인).

# ── 현재 색을 얼마나 보고 있나? ──────────────────────────────
ros2 topic echo /target_coverage
#   color_confirm 이 내는 값 = target 색이 화면을 차지하는 비율(0.0~1.0).
#   벽 앞 정면이면 0.3(임계) 이상으로 올라가야 confirm. /target_color 미지정이면 0.

# ── 자기위치 추정 살아있나? ──────────────────────────────────
ros2 lifecycle get /amcl
#   AMCL 상태. "active [3]" 면 정상. inactive/unconfigured 면 Nav2 라이프사이클 미기동.

# ── 로봇이 맵 어디에 있다고 생각하나? ────────────────────────
ros2 run tf2_ros tf2_echo map base_link
#   map→base_link = AMCL 추정 현재 위치/자세를 실시간 출력. Translation 이 실제와
#   비슷해야 정상. 에러면 → 위치추정 미수렴(relocalize 또는 RViz "2D Pose Estimate").

# ── 카메라 영상 가볍게 보기 (WiFi 느릴 때) ──────────────────
ros2 run rqt_image_view rqt_image_view /camera/image_raw/compressed
#   압축 영상 뷰어. RViz Image(raw)는 대역폭이 커 WiFi에서 끊길 때 대안.

# ── 색 검출(HSV 마스크)을 눈으로 보정 ───────────────────────
python3 scripts/color_detector.py --ros-args -p show:=true
#   원본 + R/G/B 마스크 창. 실조명에서 색이 잡히는지/흰벽이 색으로 새지 않는지 확인
#   → maze_common.py 의 COLOR_RANGES(특히 S_min) 튜닝 후 재실행.
```

## 5) 종료 (좀비 정리)
```bash
for p in '[s]lam_toolbox' '[a]mcl' '[m]ap_server' '[c]ontroller_server' '[p]lanner_server' \
         '[b]t_navigator' '[l]ifecycle' '[c]olor_confirm' '[m]aze_tour' '[c]olor_mapper' '[s]can_explorer'; do
  pkill -9 -f "$p"; done; ros2 daemon stop; ros2 daemon start
```

---

## ⚠️ 실로봇 필수 체크 (안 맞으면 조용히 실패)
1. **`ROS_DOMAIN_ID` 로봇=PC 동일** — 다르면 통신 자체 안 됨
2. **카메라 토픽 `/camera/image_raw`** — 다르면 색 노드가 영원히 0% (1번 remap 확인)
3. **`use_sim_time:=false`** 어디서나 — true면 TF/시간 꼬여 전멸
4. **초기 위치** — `relocalize:=true` 또는 RViz "2D Pose Estimate"
5. **시뮬 맵 재사용 금지** — 실제 공간 새로 매핑(Phase 1) 필수
6. **`scan_explorer`는 실로봇을 자율로 움직임** — 매핑 중 사람 비우고 비상정지(Ctrl-C) 대기.
   내부 웨이포인트는 시뮬 방 전용이라 실공간에선 **`--perimeter-frac 1.0`**(둘레만) 권장.
7. **`CONFIRM_THRESHOLD`(현재 30%)** 는 어안렌즈 기준 — 실카메라 화각 다르면 confirm 너무
   쉽거나 어려울 수 있음 → `color_confirm` 의 `threshold` 파라미터로 조정.
