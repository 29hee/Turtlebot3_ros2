# 재매핑 가이드 & 색 커버리지 체크리스트

현재 맵은 **`color_room.world`** 기준으로 재매핑된 결과입니다.

| 파일 | 상태 |
|---|---|
| `color_room.pgm` / `color_room.yaml` | ✅ 현재 월드와 일치 — **런타임이 사용**(`runtime.launch.py`) |
| `color_landmarks.yaml` | ✅ color_mapper 가 누적한 색 시맨틱맵 (단, 아래 **커버리지 구멍** 주의) |
| `color_maze.pgm` / `color_maze.yaml` | ⚠️ 옛 미로 월드 산출물(STALE). 더 이상 쓰지 않음 — 참고용으로만 남김 |

## ⚠️ 알려진 색 커버리지 구멍 (재매핑으로 보완 필요)

`color_landmarks.yaml` 을 ground-truth 와 대조하면 일부 패널이 누락/약함:

- **RED (-2.40, 0.80) 서쪽 패널 — 누락**: landmarks 에 해당 클러스터가 없음.
  → target RED 순회 시 3개 중 2개만 방문된다.
- **BLUE (-1.50, 2.40) 북쪽 패널 — 약함**: `(-0.75, 2.55) votes 11` 한 셀뿐 →
  `maze_common.VOTE_FLOOR=15` 필터에 걸려 **탈락**. → target BLUE 도 2/3개.

> ⚠️ 주의: `maze_tour` 의 '엄격 완료'는 **landmarks 에 아예 없는 벽은 실패로 잡지 못한다**.
> 즉 누락 패널은 조용히 빠진 채 "전부 확인" 성공 처리될 수 있다. 반드시 아래 체크리스트로 검증할 것.

**보완 방법(택1 이상):**
1. **재매핑 커버리지 개선(권장):** `scan_explorer` 가 서쪽/북쪽 벽을 더 정면(face-on)·근거리에서
   보도록 주행 시간(`--duration`)·스핀 간격(`--drive`)을 늘린다. 누락 구역을 집중적으로 스캔.
2. **임계 완화:** `maze_common.VOTE_FLOOR` 를 낮춰 약한 클러스터를 살린다.
   단 노이즈 셀도 함께 살아나므로 재매핑 품질 개선이 우선.

## 재매핑 절차 (원격 PC)

```bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch <pkg>/launch/mapping.launch.py     # gazebo(color_room) + slam + scan_explorer + color_mapper
# 방을 충분히(특히 서/북 벽) 돈 뒤 점유격자 저장:
ros2 run nav2_map_server map_saver_cli -f <pkg>/maps/color_room
# color_landmarks.yaml 은 color_mapper 가 자동 누적 저장.
```
> 회전은 느리게(0.3 rad/s) — 빠르면 slam_toolbox 가 못 따라가 맵이 뒤틀린다(실환경 검증).

## 재매핑 후 sanity-check (ground-truth 패널 좌표)

`color_landmarks.yaml` 이 색마다 **3개 벽**으로 수렴해야 한다:

```
RED:   (-1.50,-2.40), ( 2.40, 1.20), (-2.40, 0.80)
GREEN: ( 0.00,-2.40), ( 2.40,-1.20), ( 1.00, 2.40)
BLUE:  ( 1.50,-2.40), (-1.50, 2.40), (-2.40,-1.00)
```

> 런타임은 `maze_common` 의 필터(0.5m 단일연결 병합 + 색별 상대임계 + 절대바닥)로 노이즈를 거른다.
> 색당 3개가 안 나오면 위 '커버리지 구멍' 절차로 보완.

## 실로봇 전환 시 추가 보정 (시뮬→실물)

시뮬레이션은 `Gazebo/Red·Green·Blue` 순색(채도 거의 255) 기준이라, 실물에선 다음을 재보정해야 한다:

- **HSV 범위(`maze_common.COLOR_RANGES`)**: 실물 패널·조명에서 `color_detector.py` 를
  `show:=true` 로 띄워 마스크를 보며 `S_min` 부터 조정.
- **confirm 임계(`CONFIRM_THRESHOLD`)**: 현재 0.30 은 burger_cam 182° 어안 기준.
  실카메라는 화각이 좁아 같은 벽이 프레임을 더 크게 채우므로, **0.30 → 0.60 까지 올리며**
  벽 도착 confirm 동작을 단계적으로 검증(사양 최종 목표 60%).
