# 실로봇 실행 가이드 (TurtleBot3 Burger + 카메라)

자율주행 SLAM 매핑 → 색 안내 런타임까지 실로봇 전체 명령. 시뮬은 `README.md` 참고.

> **먼저 본인 환경에 맞게 바꿀 값**
> `<로봇IP>` · `<워크스페이스>`(turtlebot3_ws) · `ROS_DOMAIN_ID`(로봇·PC 동일)
> (co_project 경로는 이 노트북 기준 `/home/user/workspace/co_project` 로 이미 박아둠.)
> **카메라 배선:** v4l2(로봇)는 `/camera/image_raw_rot` 로 발행 → image_upright(PC)가 표준 토픽
> `/camera/image_raw` 를 똑바로 세워 채움(색 노드는 이걸 구독). v4l2 를 `/camera/image_raw` 로 직접
> remap 하지 말 것 — publisher 가 겹쳐 영상이 꼬인다.

---

## 0) 공통 환경 — 모든 터미널(로봇·PC 양쪽)
```bash
export TURTLEBOT3_MODEL=burger_cam
export ROS_DOMAIN_ID=30                       # 로봇과 PC 같은 값!
source /opt/ros/humble/setup.bash
source <워크스페이스>/install/setup.bash       # 이 노트북(PC): /home/user/workspace/install/setup.bash · 로봇(Pi): ~/turtlebot3_ws/install/setup.bash
```

> **숫자 인식 의존성(색 노드 돌리는 PC만, 1회).** 숫자는 `digit_recognizer.py` 가 EasyOCR 로 읽는다
> (과거 MNIST CNN·Tesseract 폐기, EasyOCR 단일화).
> ```bash
> pip3 install easyocr "numpy<2"   # easyocr 가 numpy 2.x 를 끌어올려 cv_bridge 를 깨므로 numpy<2 고정
> ```
> ⚠ numpy 가 2.x 면 cv_bridge 가 `_ARRAY_API not found`/segfault 로 죽어 vision_node 등이 안 뜬다.
>   증상 보이면:  `pip3 install "numpy<2"`
> easyocr 미설치여도 색 인식은 동작하지만, 색+숫자 필수라 숫자가 -1 이면 맵은 빈다.

## 1) 로봇(라즈베리파이) 측 — SSH
```bash
ssh ubuntu@<로봇IP>
# (0번 환경 source 후)
ros2 launch turtlebot3_bringup robot.launch.py          # 라이다 + 모터 + odom TF

# 카메라 노드 (별도 터미널) — 우리 버거는 카메라가 '거꾸로' 장착됨.
#   원본을 _rot 으로 빼면(거꾸로), PC 의 image_upright(아래 1.5)가 똑바로 세워 표준 토픽을 채운다.
ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw_rot
#   ↑ USB캠 기준. Pi캠이면 해당 노드. 토픽만 /camera/image_raw_rot 로 동일하게.
#   ⚠ /camera/image_raw 로 직접 remap 금지 — image_upright 와 publisher 가 겹쳐 영상이 꼬인다.
```

---

## 1.5) PC — 카메라 상하반전 보정 (Phase 1·2 내내 켜둠)
> 거꾸로 장착된 카메라를 '소스에서 1회' 회전해 표준 토픽 `/camera/image_raw` 를 똑바로 채운다.
> 이 노드 하나만 거치면 RViz·color_confirm·OCR·rqt 등 모든 구독자가 정상 방향을 본다.
> **로봇이 아니라 PC(이 노트북)에서** 돌린다 — Pi 에 OpenCV/cv_bridge·repo 를 안 깔아도 되고 Pi CPU 도 아낀다.
> image_upright 가 `/camera/image_raw` 의 **유일한 publisher** → 죽으면 색 인지가 통째로 멈춘다(꼭 켜둘 것).
```bash
cd /home/user/workspace/co_project/pkgs/capstone_color_maze
python3 scripts/image_upright.py
#   좌우만/상하만 뒤집힌 장착이면:  python3 scripts/image_upright.py --ros-args -p flip:=h   (또는 v)
#   켜두면 Phase 1·2 가 이 한 노드를 공유. 똑바로 선 압축영상은 /camera/image_raw/compressed 로도 나온다.
```

---

## 1.7) PC — 사전 점검 (매핑 시작 전 1회, 권장)
> 그동안 런타임에서 하나씩 터지던 '조용한 실패'(numpy 충돌·카메라 안뜸/느림·로봇 미연결·
> TF 없음·클럭 skew)를 시작 전에 한 번에 잡는다. ❌ 부터 해결하고 매핑 시작.
```bash
cd /home/user/workspace/co_project/pkgs/capstone_color_maze
python3 scripts/preflight.py
#   [1]의존성 [2]/camera·/scan Hz [3]TF [4]Pi↔PC 클럭 [5]로봇 cmd_vel 수신 을 점검.
#   카메라가 5Hz 미만이면 색을 놓치니 v4l2 해상도↓/fps↑ 로 올릴 것.
```

---

## 2) PC — Phase 1: 자율주행 SLAM 매핑 (한 번)
> 시뮬 맵 재사용 금지 → 실제 공간을 새로 매핑. `scan_explorer`가 **자율로 로봇을 움직이므로**
> 매핑 중엔 **주변에 사람을 비우고, Ctrl-C(비상정지) 대기**할 것.

```bash
cd /home/user/workspace/co_project/pkgs/capstone_color_maze

# (터미널 A) SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

# (터미널 B) ★ 단일 디코더 — 영상을 '한 번만' 풀어 /detected_color, /color_signal 발행.
#   (이거 하나가 색 계산 담당 → mapper·explorer·digit 가 영상 대신 이 신호를 구독 = CPU 절약)
python3 scripts/vision_node.py --ros-args -p use_sim_time:=false

# (터미널 C) 색 누적 매퍼 — /detected_color + 근접 라이다거리로 격자 투표(근접 max_range 0.8m).
#   ★ '색+숫자 둘 다' 인식된 칸만 저장(무조건). 숫자 못 읽은 칸은 보류 → 재접근 필요.
python3 scripts/color_mapper.py --ros-args -p use_sim_time:=false

# (터미널 D) ★ 숫자 인식기(EasyOCR) — 필수(상시). 근접일 때만 OCR → /detected_digit.
#   안 띄우거나 easyocr 미설치면 저장 0(색+숫자 필수라). → pip3 install easyocr 먼저.
python3 scripts/digit_recognizer.py --ros-args -p use_sim_time:=false

# (터미널 E) ★ 색-반응 탐사 주행 — 벽타며 색 발견 시 패널 ~0.3m 접근→정지(dwell)해서 근접 기록.
#   둘레 한 바퀴 → 중앙 진입 → 섬 벽타기. '같은 자리 빙빙' 방지(진행 워치독/방문격자/loop감지) 내장.
python3 scripts/maze_explorer.py --duration 600 --ros-args -p use_sim_time:=false
#   끝나면 자동 정지(미방문 소진 또는 시간 상한). 중간에 멈추려면 Ctrl-C.

# (터미널 F) 매핑 품질 라이브 점검 — 색별 벽수/숫자/누락 출력(기본 기대 = 색당 3개, 총 9).
python3 scripts/quality_monitor.py
#   '합계: 9/9 … ✅ 전부 확보' 가 떠야 완성. '미발견/숫자미상/부족/중복' 이 보이면 더 돌 것.

# (터미널 G) 보면서 (맵 + 라이다 + 카메라 + 색 마커)
rviz2 -d config/maze.rviz

# (터미널 H) 충분히 돌았으면 점유격자 저장
ros2 run nav2_map_server map_saver_cli -f maps/color_room
```
> quality_monitor 가 모든 색·숫자를 잡았다고 보일 때 저장한다. 누락 있으면 maze_explorer 를
> 다시 돌리거나 `--duration` 을 늘린다. 벽 추종 거리는 `maze_explorer` 의 `target_right`/`front_stop`,
> 접근 정지거리는 `standoff`(기본 0.3m), 끼임 탈출은 `stuck_dist`/`stuck_win` 으로 조정.

---

## 3) PC — Phase 2: 런타임 색 안내 (상시 구동)
```bash
cd /home/user/workspace/co_project/pkgs/capstone_color_maze

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
#   카메라 프레임 주파수(보통 10~30Hz). 이 토픽은 image_upright 가 채우는 '똑바로 선' 영상.
#   안 뜨면 → ① v4l2 가 /camera/image_raw_rot 으로 발행 중인지, ② image_upright 가 떠 있는지 확인.
#   (원본 거꾸로 영상은 /camera/image_raw_rot 에서 hz 확인 가능)

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
#   image_upright 가 내는 '똑바로 선' 압축 영상(대역폭 절약). 평소엔 이걸 본다.
#   원본(거꾸로) 생존만 확인하려면:  rqt_image_view /camera/image_raw_rot/compressed

# ── 색 검출(HSV 마스크)을 눈으로 보정 ───────────────────────
python3 scripts/color_detector.py --ros-args -p show:=true
#   원본 + R/G/B 마스크 창. 실조명에서 색이 잡히는지/흰벽이 색으로 새지 않는지 확인
#   → maze_common.py 의 COLOR_RANGES(특히 S_min) 튜닝 후 재실행.
#   숫자 인식은 별도 노드(EasyOCR):  python3 scripts/digit_recognizer.py --ros-args -p show:=true
#   (1.5 의 image_upright 가 떠 있어야 숫자가 똑바로 들어감)
```

## 5) 종료 (좀비 정리)
```bash
for p in '[s]lam_toolbox' '[a]mcl' '[m]ap_server' '[c]ontroller_server' '[p]lanner_server' \
         '[b]t_navigator' '[l]ifecycle' '[c]olor_confirm' '[m]aze_tour' '[c]olor_mapper' '[s]can_explorer' \
         '[m]aze_explorer' '[v]ision_node' '[d]igit_recognizer' '[q]uality_monitor' '[i]mage_upright'; do
  pkill -9 -f "$p"; done; ros2 daemon stop; ros2 daemon start
```

---

## ⚠️ 실로봇 필수 체크 (안 맞으면 조용히 실패)
1. **`ROS_DOMAIN_ID` 로봇=PC 동일** — 다르면 통신 자체 안 됨
2. **카메라 토픽 `/camera/image_raw`** — 다르면 색 노드가 영원히 0% (1번 remap 확인).
   우리 버거는 카메라가 **거꾸로** 장착 → v4l2 는 `/camera/image_raw_rot` 으로 빼고
   **`image_upright.py` 를 반드시 띄워** 표준 토픽을 똑바로 세울 것(안 띄우면 표준 토픽 발행자 0).
3. **`use_sim_time:=false`** 어디서나 — true면 TF/시간 꼬여 전멸
4. **초기 위치** — `relocalize:=true` 또는 RViz "2D Pose Estimate"
5. **시뮬 맵 재사용 금지** — 실제 공간 새로 매핑(Phase 1) 필수
6. **`scan_explorer`는 실로봇을 자율로 움직임** — 매핑 중 사람 비우고 비상정지(Ctrl-C) 대기.
   내부 웨이포인트는 시뮬 방 전용이라 실공간에선 **`--perimeter-frac 1.0`**(둘레만) 권장.
7. **`CONFIRM_THRESHOLD`(현재 30%)** 는 어안렌즈 기준 — 실카메라 화각 다르면 confirm 너무
   쉽거나 어려울 수 있음 → `color_confirm` 의 `threshold` 파라미터로 조정.
