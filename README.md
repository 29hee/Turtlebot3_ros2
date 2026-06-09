# 🎨 미술관 자율주행 안내 휠체어

> ROS2 Humble · TurtleBot3(burger_cam) · Gazebo · 색 기반 시맨틱 내비게이션
>
> **팀원용 인수인계 문서** — 내일 바로 착수할 수 있도록 정리. 상세 설계는 [`pkgs/capstone_color_maze/ARCHITECTURE.md`](pkgs/capstone_color_maze/ARCHITECTURE.md) 참고.

---

## 1. 한눈에 보기

거동이 불편한 미술관 방문객이 **보고 싶은 전시 구역을 색으로 선택**하면, 자율주행 휠체어가
**관람객·장애물을 피해** 그 구역 작품 앞 **정위치까지 안내하고 멈춰** 감상하게 한다.

핵심 아이디어는 **운영을 2개 모드로 나눈 것**:

| 모드 | 시점 | 하는 일 | 비전 활용 |
|---|---|---|---|
| **① 매핑(큐레이션)** | 개관 전·무인 | 자율탐사로 한 바퀴 돌며 작품 색/위치 자동 등록 → 시맨틱 맵 저장 | **실시간** 작품 감지 |
| **② 서비스(안내)** | 개관 중·탑승 | 저장 맵에서 목표 조회 → Nav2 안내 + 관람객 회피 | **실시간** 관람객 회피 |

> 💡 **왜 2-모드인가**: 작품(고정)은 사전맵으로, 관람객(이동)은 실시간으로. 특별전으로 작품이 바뀌어도 **하루 전 한 바퀴 재매핑하면 적응** → "고정 데이터셋"이 아닌 **재학습형** 시스템이 차별점.

---

## 2. 동작별 적용 기술

| 동작 | 기술 | 강의 |
|---|---|---|
| 작품 색 인지 | OpenCV HSV / YOLO | 04, 05.02 |
| 색 → 3D 위치 변환 | 카메라 + 라이다 + TF 센서융합 | 09 |
| 지도 생성 / 자기위치 | SLAM(slam_toolbox) / AMCL | 11, 12 |
| 자율주행 / 경로계획 | Nav2 | 13, (14.03 A*/RRT) |
| 작품 앞 정면 정렬·정지 | PID 제어 | 14.01 |
| 관람객 추적·회피 | Kalman 필터 | 05.02 |
| 자연어 작품 안내 | VLM/LLM | (확장) |

---

## 3. 빠른 시작 (검증 완료 경로)

> ⚠️ **환경 주의**: 로봇 모델은 반드시 `burger_cam`(카메라 포함). turtlebot3_ws는 **대문자 W** 경로.

### 공통 헤더 (새 터미널마다 붙여넣기)
```bash
export TURTLEBOT3_MODEL=burger_cam                                 # 표준 burger엔 카메라 없음 → 반드시 burger_cam
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash                          # ← 본인 turtlebot3 워크스페이스 경로로 수정
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
cd <클론한 경로>/co_project/pkgs/capstone_color_maze               # ← 본인 클론 경로로 수정 (이하 명령은 이 위치 기준)
```
> 📌 아래 모든 명령은 **`capstone_color_maze` 디렉터리에서 실행**하는 것을 기준으로 합니다.

### ① 매핑 모드 — 색맵 구축
```bash
ros2 launch launch/mapping.launch.py        # gazebo(color_room)+SLAM+탐사+색매핑
# 방을 충분히 돈 뒤 점유격자맵 저장:
ros2 run nav2_map_server map_saver_cli -f maps/color_room
# color_landmarks.yaml 은 자동 누적. 필터 결과 확인:
python3 -c "import sys;sys.path.insert(0,'scripts');import yaml;from maze_common import resolve_target_walls;d=yaml.safe_load(open('maps/color_landmarks.yaml'));[print(c,len(resolve_target_walls(d,c)),'개') for c in('RED','GREEN','BLUE')]"
```

### ② 서비스 모드 — 색 안내 주행
```bash
ros2 launch launch/runtime.launch.py \
  target_color:=RED start_gazebo:=true \
  map:=$(pwd)/maps/color_room.yaml
```
- 띄운 뒤 `ros2 lifecycle get /amcl` → `active [3]` 확인
- RViz에서 라이다가 벽에 안 붙으면 **`2D Pose Estimate`로 정합** 잡기
- 색을 바꿀 땐 **runtime을 그 색으로 통째로** 재실행 (maze_tour와 color_confirm 색 자동 일치)

### 막히면 — 클린 재시작 (좀비 프로세스 정리)
```bash
for p in '[g]zserver' '[g]zclient' '[s]lam_toolbox' '[a]mcl' '[m]ap_server' '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator' '[l]ifecycle_manager' '[m]aze_tour' '[c]olor_confirm' '[w]all_follower' '[c]olor_mapper' '[r]viz2'; do pkill -9 -f "$p"; done; ros2 daemon stop; ros2 daemon start
```

---

## 4. 현재 코드 구조 (→ 목표 역할)

| 현재 파일 | 목표 역할 | 다음 작업 |
|---|---|---|
| `scripts/color_detector.py` | `art_detector` | HSV + **YOLO** 작품 식별 |
| `scripts/color_mapper.py` | `semantic_mapper` | 실시간 자동등록 강화 |
| `scripts/maze_tour.py` | `guide_manager` | 단일/투어 모드, `/visit_request` |
| `scripts/color_confirm.py` | `arrival_confirm` | 임계 현실화 + **PID 정렬** |
| `scripts/maze_common.py` | 공유 유틸 | 유지 |
| `scripts/wall_follower.py` | `explore` | frontier 탐사로 대체 |
| `launch/mapping.launch.py` | 매핑 모드 | explore+art_detector+semantic_mapper |
| `launch/runtime.launch.py` | 서비스 모드 | +crowd_avoider |

---

## 5. ✅ 진행 체크리스트

### Phase A — MVP (✅ 완료, 2026-06-03)
- [x] `color_room.world` 단순 전시실 환경 구성
- [x] SLAM 매핑 + 색맵(`color_landmarks.yaml`) 자동 구축
- [x] **색 → Nav2 좌표 타게팅 주행·도착 검증 완료**
- [x] AMCL 위치추정 정합 (2D Pose Estimate)
- [x] `maze_tour` 색벽 순회 + `color_confirm` 도착확인 골격
- [x] 단일/투어 안내 모드 사양 확정

### Phase B — 2-모드 + 강의 망라 (⬜ 다음 단계)
- [ ] ⭐ **`color_confirm` 임계 현실화/ROI** — 광각 카메라 점유율 ~14% 대응 *(가장 쉬움 · 데모 완성 직결 · 첫날 추천)*
- [ ] `arrival_confirm`: PID 정면정렬·부드러운 정지 (14.01)
- [ ] `art_detector`: YOLO 작품 식별 추가 (05.02)
- [ ] `semantic_mapper`: 실시간 자동등록 강화 (커버리지·중복제거)
- [ ] `guide_manager`: 단일/투어 모드 + `/visit_request` 커스텀 메시지
- [ ] `input_adapter`: 입력 방식 결정(음성/YOLO-OCR/QR) → 공통 요청 토픽
- [ ] `explore`: frontier 자율탐사로 `wall_follower` 대체

### Phase C — 차별화 (⬜)
- [ ] `crowd_avoider`: Kalman 관람객 추적 + 동적 회피 (05.02)
- [ ] `color_room`에 움직이는 관람객(actor) 추가
- [ ] (선택) 커스텀 planner A*/RRT를 Nav2 plugin으로 (14.03)
- [ ] 다중 방문객 요청 스케줄링

### Phase D — AI·실세계 (⬜)
- [ ] `docent`: VLM/LLM 자연어 작품 안내
- [ ] sim2real 실로봇(TurtleBot3) 이식
- [ ] 환경 변화 시 자동 재매핑 데모

---

## 6. ⚠️ 알려진 이슈 / 주의사항

- **confirm 60% 임계가 비현실적**: `burger_cam` 카메라는 `horizontal_fov=3.183rad(≈182°)` 초광각(sdf 확인)이라, 작품을 정면에서 봐도 화면 점유율 ~14%. 60% 절대 도달 불가 → **임계 현실화(0.12~0.15) 또는 중앙 ROI 방식**으로 변경 필요. (Phase B 첫 태스크)
  - ✅ **모델 확인 결과**: 표준 `burger`엔 카메라가 없음(sdf 카메라 태그 0개) → 카메라 사용엔 `burger_cam`이 맞음. fov 3.183rad라 광각도 사실(오히려 초광각).
- **🔴 클론 시 맵 누락 (공유 전 필수)**: `color_room.pgm/yaml`이 git 미추적, `color_landmarks.yaml`도 변경 미커밋 상태. 클론한 팀원은 **서비스 모드를 바로 못 돌림**(매핑 모드부터 돌려 맵 생성 필요). → 공유 전 `git add maps/color_room.* maps/color_landmarks.yaml && git commit` 필요. (292)
- **색 불일치 주의**: `maze_tour`와 `color_confirm`의 `target_color`는 항상 같아야 함. `maze_tour`만 따로 색 바꿔 돌리면 confirm이 0% → 무한 실패. → runtime을 통째로 띄우면 자동 일치.
- **AMCL 정합**: 대칭/단순 환경에서 amcl이 틀린 위치에 수렴할 수 있음. 라이다가 벽에 안 붙으면 RViz `2D Pose Estimate`로 수동 정합.
- **경로**: 작업 코드 = `co_project/pkgs/capstone_color_maze` / turtlebot3_ws = `/home/user/Workspace`(대문자) / 구버전 `ros2_project/...`는 참고만.
- **pkill 주의**: `pkill -f` 패턴이 자기 명령줄을 매칭해 셸이 죽을 수 있음 → 위처럼 `[g]zserver` 대괄호 트릭 사용.

---

## 7. 문서 안내
- 📐 시스템 설계 상세: [`pkgs/capstone_color_maze/ARCHITECTURE.md`](pkgs/capstone_color_maze/ARCHITECTURE.md)
- 📋 프로젝트 계획서(제출용): [`프로젝트_계획서.md`](프로젝트_계획서.md)
- 🧾 ROS2 명령어 치트시트: [`commands.md`](commands.md)
