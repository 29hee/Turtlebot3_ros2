# ⚠️ 이 maps/ 는 STALE — 재매핑 필요

`color_maze.pgm` / `color_maze.yaml` / `color_landmarks.yaml` 은 **옛 미로 월드**
(`worlds/color_maze.world`, 모자이크 벽)의 산출물입니다. 월드가 **흰벽 방 +
RGB 단색 패널 + 장애물**(`worlds/color_room.world`)로 교체되어 더 이상 일치하지 않습니다.

## 원격 PC 에서 재매핑하세요

```bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch <pkg>/launch/mapping.launch.py        # gazebo(color_room) + slam + wall_follower + color_mapper
# 충분히 방을 돈 뒤 점유격자 저장:
ros2 run nav2_map_server map_saver_cli -f <pkg>/maps/color_maze
# color_landmarks.yaml 은 color_mapper 가 자동 누적 저장.
```

## 재매핑 후 sanity-check (ground-truth 패널 좌표)

`generate_world.py` 실행 시 출력되는 좌표 근처로 `color_landmarks.yaml` 이 수렴해야 합니다:

```
RED:   (-1.50,-2.40), (2.40, 1.20), (-2.40, 0.80)
GREEN: ( 0.00,-2.40), (2.40,-1.20), ( 1.00, 2.40)
BLUE:  ( 1.50,-2.40), (-1.50, 2.40), (-2.40,-1.00)
```

> 색당 패널 3개 → 재매핑·클러스터링이 잘 되면 색마다 약 3개 벽이 잡혀야 한다.
> 런타임은 `maze_common` 의 필터(0.5m 단일연결 병합 + 색별 상대임계)로 노이즈를 거른다.
