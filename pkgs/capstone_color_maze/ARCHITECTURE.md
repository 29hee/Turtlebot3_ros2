# 🎨 미술관 자율주행 안내 휠체어 — 시스템 아키텍처

> ROS2 Humble · TurtleBot3(burger_cam) · Gazebo · 색 기반 시맨틱 내비게이션
> 최종 갱신: 2026-06-03

---

## 1. 프로젝트 개요

거동이 불편한 미술관 방문객이 **보고 싶은 전시 구역을 색으로 선택**하면, 자율주행 휠체어가
**관람객·장애물을 피해** 그 구역 작품 앞 **정위치까지 안내하고 멈춰** 감상하게 한다.

- **방문객**: 휠체어 탑승. 색(구역)으로 목적지 지정.
- **공간**: 개방형 전시실(`color_room.world`). 벽면 색 구역(작품) + 관람객/전시대(장애물).
- **로봇**: 라이다 SLAM/AMCL(자기위치) + 카메라(작품 인지) + Nav2(주행).

---

## 2. 핵심 설계 원칙

| 원칙 | 내용 | 이유 |
|---|---|---|
| **정적 = 사전맵** | 작품 위치는 미리 구축한 시맨틱 맵에서 조회 | 미술관 작품은 큐레이션돼 위치 known |
| **동적 = 실시간** | 관람객은 실시간 감지·추적·회피 | 사람 위치는 미리 못 외움 |
| **운영 2-모드** | 개관 전 매핑 / 개관 중 서비스로 분리 | 실시간 비전과 사전맵을 시간축으로 양립 |
| **범용성** | 환경 변화 시 자율 재매핑으로 적응 | 고정 데이터셋이 아닌 "재학습형" 시스템 |

---

## 3. 운영 2-모드

### ① 큐레이션/매핑 모드 (개관 전 · 무인)
```
 자율탐사(frontier explore)로 미술관 한 바퀴
   │
   ├─▶ 실시간 작품 감지 (카메라: HSV / YOLO)
   │
   ├─▶ 검출 → 라이다 거리 + TF 융합으로 3D map 좌표 투영
   │
   ├─▶ 같은 작품 중복 등록 방지 (클러스터링/격자투표)
   │
   └─▶ 시맨틱 맵 자동 구축·갱신 → 저장
         · 점유격자맵 (color_room.pgm/yaml)  ← SLAM
         · 색-좌표 시맨틱맵 (color_landmarks.yaml)
```
> **실시간 색감지의 정당한 자리.** "헤매는 탐색"이 아니라 무인 자율 큐레이션.
> 특별전으로 작품이 바뀌면 → 하루 전 한 바퀴 재매핑 → 즉시 적응.

### ② 서비스/안내 모드 (개관 중 · 방문객 탑승)
```
 방문객 입력: (목표색, 안내모드)
   │
   ├─▶ 시맨틱 맵에서 해당 색 작품 좌표 조회 (사전맵)
   │
   ├─▶ Nav2 자율주행 (전역경로 + 장애물회피)
   │     └─▶ 실시간 관람객 감지·추적(Kalman) → 동적 회피
   │
   ├─▶ 작품 앞 정위치 도착 + 정면 정렬(PID) + 작품 확인(카메라)
   │
   ├─▶ 부드러운 정지 + 도착 안내 (+ VLM 자연어 설명)
   │
   └─▶ 감상(dwell) → 다음 작품(투어) / 새 목적지 대기(단일)
```

### 안내 모드 (서비스 모드 내 선택)
- **단일 안내형**: 원하는 한 작품으로 데려다주고 정지 → 다음 입력 대기
- **도슨트 투어형**: 선택 색 구역 작품들을 순서대로 순회하고 마지막에 정지

---

## 4. 시스템 아키텍처 (노드 구성)

```
                        ┌─────────────────── 공유 인프라 ───────────────────┐
                        │  Gazebo(센서) · SLAM/AMCL(위치) · Nav2(주행)       │
                        │  maze_common(기하·필터 유틸) · 시맨틱맵 파일        │
                        └───────────────────────────────────────────────────┘
                              ▲                                   ▲
        ┌─────────────────────┴────────┐          ┌──────────────┴───────────────────┐
        │   ① 매핑 모드 노드            │          │   ② 서비스 모드 노드               │
        ├──────────────────────────────┤          ├────────────────────────────────────┤
        │ explore      (자율탐사)       │          │ input_adapter (입력→/visit_request) │
        │ art_detector (실시간 작품검출)│          │ guide_manager (요청→목표선택/순회)  │
        │ semantic_mapper(투영·등록·저장)│         │ arrival_confirm(도착확인·PID정렬)   │
        │ slam_toolbox (지도생성)       │          │ crowd_avoider (관람객 추적·회피)    │
        │                               │          │ docent        (VLM 자연어 안내)     │
        └──────────────────────────────┘          └────────────────────────────────────┘
```

### 주요 인터페이스 (토픽/액션/메시지)
| 이름 | 타입 | 방향 | 용도 |
|---|---|---|---|
| `/visit_request` | (custom) `{color, mode}` | input_adapter → guide_manager | 방문객 목적지 요청 |
| `/camera/image_raw` | sensor_msgs/Image | 카메라 → 검출/확인 | 작품 인지 |
| `/scan`, `/tf` | LaserScan, TF | 센서 → 투영 | 거리·좌표변환 |
| `navigate_to_pose` | nav2 action | guide_manager → Nav2 | 목표 주행 |
| `/target_coverage`, `/target_confirmed` | Float32, Bool | arrival_confirm → guide_manager | 도착 작품 확인 |
| `/tour_done` | Bool | guide_manager | 안내 완료 신호 |
| `color_landmarks.yaml` | 파일 | 매핑 → 서비스 | 색-좌표 시맨틱맵 |

---

## 5. 기술 스택 ↔ 강의 매핑

| 강의(주차) | 적용 모듈 | 모드 |
|---|---|---|
| 04 OpenCV / 05.01 Calibration | HSV 색검출, 카메라 보정 | 공통 |
| **05.02 YOLO** | art_detector 작품 식별 | 매핑 |
| **05.02 Kalman** | crowd_avoider 관람객 추적 | 서비스 |
| 06–08 ROS2 노드/토픽/서비스/액션/Launch/Custom-msg | 전 노드, `/visit_request` | 공통 |
| 09 TF2/Quaternion | 색→3D 좌표 투영, 포즈 | 공통 |
| 10 Gazebo URDF/Plugin | color_room, burger_cam | 공통 |
| 11 SLAM / 12 AMCL | slam_toolbox / 위치추정 | 매핑 / 서비스 |
| 13 Nav2 | 자율주행 | 서비스 |
| **14.01 PID** | arrival_confirm 정면 정렬·부드러운 정지 | 서비스 |
| 14.03 A*/RRT | (선택) 커스텀 planner | 서비스 |

> 골격(06–13,15 + 04,05.01) 망라 + YOLO·Kalman·PID 추가 시 ~95% 커버.

---

## 6. Phase 로드맵

| Phase | 내용 | 난이도 | 상태 |
|---|---|---|---|
| **A. MVP** | 색→Nav2 타게팅 + 모드선택 + 도착확인/정지 | 中 | ✅ 검증 완료 |
| **B. 2-모드 + 강의망라** | 매핑모드(실시간 작품 자동등록) / 서비스모드 분리 · +PID·YOLO·Kalman | 中上 | ⬜ |
| **C. 차별화** | 동적 관람객 회피 통합 · (선택) 커스텀 planner/BT · 다중요청 스케줄 | 上 | ⬜ |
| **D. AI·실세계** | VLM 자연어 도슨트 · sim2real 실로봇 · 환경변화 자동 재매핑 | 最上 | ⬜ |

---

## 7. 기존 코드 자산 → 목표 아키텍처 매핑

| 현재 파일 | 목표 역할 | 변경 방향 |
|---|---|---|
| `scripts/color_detector.py` | → `art_detector` | HSV + **YOLO** 작품 식별 추가 |
| `scripts/color_mapper.py` | → `semantic_mapper` | 센서융합 투영 유지, **실시간 자동등록** 강화 |
| `scripts/maze_tour.py` | → `guide_manager` | 단일/투어 모드, `/visit_request` 수신 |
| `scripts/color_confirm.py` | → `arrival_confirm` | 임계 현실화/ROI + **PID 정면정렬** |
| `scripts/maze_common.py` | 공유 유틸 | 유지 (기하·클러스터·필터) |
| `scripts/wall_follower.py` | → `explore` | frontier 자율탐사로 대체 |
| `launch/mapping.launch.py` | 매핑 모드 런치 | explore + art_detector + semantic_mapper |
| `launch/runtime.launch.py` | 서비스 모드 런치 | localization + guide + confirm + crowd_avoider |
| `worlds/color_room.world` | 전시실 환경 | 작품 배치 + 동적 관람객(actor) 추가 |

---

## 8. 디렉토리 구조 (목표)

```
capstone_color_maze/
├── ARCHITECTURE.md          ← (이 문서)
├── worlds/   color_room.world
├── maps/     color_room.{pgm,yaml}, color_landmarks.yaml
├── config/   nav2_maze.yaml
├── launch/   mapping.launch.py, runtime.launch.py
├── scripts/
│   ├── maze_common.py        (공유 유틸)
│   ├── explore.py            (매핑: 자율탐사)
│   ├── art_detector.py       (매핑: 작품 검출 HSV/YOLO)
│   ├── semantic_mapper.py    (매핑: 투영·등록·저장)
│   ├── guide_manager.py      (서비스: 요청·목표·순회)
│   ├── arrival_confirm.py    (서비스: 도착확인·PID정렬)
│   ├── crowd_avoider.py      (서비스: 관람객 추적·회피)
│   └── docent.py             (서비스: VLM 안내 — Phase D)
└── tests/    test_maze_logic.py
```

---

## 부록: 검증 완료 사항 (2026-06-03)
- color_room에서 **색→Nav2 타게팅**(좌표 주행·도착·AMCL 정합) 전부 성공 → 본 접근 타당성 입증.
- 미해결: `color_confirm` 60% 임계는 burger_cam 광각 카메라엔 비현실적(정면 패널도 ~14%) → 임계 현실화/ROI/정면정렬(PID) 필요.
