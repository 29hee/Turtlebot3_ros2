#!/usr/bin/env python3
"""
color_room.world 생성기 (Gazebo Classic 11)

컨셉(미로 아님): 흰 벽으로 둘러싼 정사각형 방.
  - 둘레: 흰색 벽 4개 (5x5m 내부, x,y ∈ [-2.5, 2.5])
  - 벽지(wallpaper): 벽 '안쪽 면'에 붙은 단색 RGB 패널. 색당 3개 = 총 9개.
      · 각 패널은 단일색(R/G/B 중 하나) 1.0(폭) x 0.5(높이) m, 두께 0.04m.
      · 벽 안쪽 면에 flush 로 부착 → 카메라가 평평한 단색 사각형을 본다.
      · '벽=색 하나' 이므로 런타임 full-frame 60% 확인이 물리적으로 성립한다.
  - 장애물: 방 중앙에 무채색(흰/회색) 원기둥+상자를 '비대칭'으로 4~5개.
      · 비대칭이어야 AMCL 이 방향을 구별(회전 대칭 헷갈림 방지) → 위치추정 보조.
      · 무채색이라 색 판정(color_detector/color_confirm)을 방해하지 않는다.

좌표/색 ground-truth 는 main() 끝에서 출력한다. 재매핑 후 color_landmarks.yaml 이
이 좌표 근처로 수렴하는지 sanity-check 에 쓰라.

생성:
    python3 generate_world.py        # worlds/color_room.world 작성 (의존성 없음)
"""
import os

# ── 치수 ──────────────────────────────────────────────────────────────
HALF = 2.5          # 내부 반폭 (방 5x5m, x,y ∈ [-HALF, HALF])
WALL_T = 0.15       # 벽 두께 [m]
WALL_H = 0.5        # 벽/패널 높이 [m] (LiDAR ~0.18m, 카메라보다 높게)
WALL_LEN = 2 * HALF + WALL_T   # 모서리에서 만나도록 약간 길게

PANEL_W = 1.0       # 패널 폭 [m]
PANEL_T = 0.04      # 패널 두께 [m]
PANEL_INSET = 0.02  # 벽 안쪽 면에서 안으로 들어오는 양 [m] (flush)

GZ = {'RED': 'Gazebo/Red', 'GREEN': 'Gazebo/Green', 'BLUE': 'Gazebo/Blue',
      'WHITE': 'Gazebo/White', 'GREY': 'Gazebo/Grey'}

# 벽 안쪽 면 좌표
INNER = HALF - WALL_T / 2.0          # = 2.425
PANEL_FACE = INNER - PANEL_INSET     # 패널 안쪽 면이 닿는 위치

# ── 패널 배치 (색당 3개, 4개 벽에 분산) ───────────────────────────────
# wall: 'bottom'(y=-HALF, +y향), 'top'(+y, -y향), 'left'(x=-HALF,+x향), 'right'(-x향)
# 각 패널: (이름, 색, 벽, 벽 위 위치 along)
PANELS = [
    # bottom 벽 (y = -HALF, 카메라는 +y 방향에서 봄)
    ('p_red_b',   'RED',   'bottom', -1.5),
    ('p_green_b', 'GREEN', 'bottom',  0.0),
    ('p_blue_b',  'BLUE',  'bottom',  1.5),
    # right 벽 (x = +HALF)
    ('p_green_r', 'GREEN', 'right',  -1.2),
    ('p_red_r',   'RED',   'right',   1.2),
    # top 벽 (y = +HALF)
    ('p_blue_t',  'BLUE',  'top',    -1.5),
    ('p_green_t', 'GREEN', 'top',     1.0),
    # left 벽 (x = -HALF)
    ('p_blue_l',  'BLUE',  'left',   -1.0),
    ('p_red_l',   'RED',   'left',    0.8),
]

# ── 장애물 (무채색, 비대칭) ───────────────────────────────────────────
# (이름, 모양, x, y, 치수...)  cylinder: r,h / box: sx,sy,h
OBSTACLES = [
    ('obs_cyl_a', 'cylinder', -0.8,  0.6, 0.18, 0.6),
    ('obs_box_a', 'box',       1.0, -0.7, 0.30, 0.30, 0.6),
    ('obs_cyl_b', 'cylinder',  0.9,  1.1, 0.15, 0.5),
    ('obs_box_b', 'box',      -1.2, -1.0, 0.25, 0.50, 0.5),
    ('obs_cyl_c', 'cylinder',  0.3, -0.4, 0.12, 0.45),
]

START = (-2.0, -2.0)   # 로봇 스폰 권장 위치(빈 코너). 런치 x_pose/y_pose 와 일치시킬 것.


def wall(name, cx, cy, length, axis, color):
    """단색 둘레 벽 1개 (collision + visual 박스)."""
    if axis == 'x':
        size = f"{length} {WALL_T} {WALL_H}"
    else:
        size = f"{WALL_T} {length} {WALL_H}"
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {WALL_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{size}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{color}</name></script></material>
        </visual>
      </link>
    </model>"""


def panel_pose_size(wall_name, along):
    """패널 중심 좌표(cx,cy)와 박스 size 문자열을 벽별로 계산."""
    if wall_name == 'bottom':
        cx, cy = along, -PANEL_FACE
        size = f"{PANEL_W} {PANEL_T} {WALL_H}"
    elif wall_name == 'top':
        cx, cy = along, PANEL_FACE
        size = f"{PANEL_W} {PANEL_T} {WALL_H}"
    elif wall_name == 'left':
        cx, cy = -PANEL_FACE, along
        size = f"{PANEL_T} {PANEL_W} {WALL_H}"
    elif wall_name == 'right':
        cx, cy = PANEL_FACE, along
        size = f"{PANEL_T} {PANEL_W} {WALL_H}"
    else:
        raise ValueError(wall_name)
    return cx, cy, size


def panel(name, color, wall_name, along):
    cx, cy, size = panel_pose_size(wall_name, along)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {WALL_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="v">
          <geometry><box><size>{size}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{GZ[color]}</name></script></material>
        </visual>
      </link>
    </model>""", cx, cy


def obstacle(spec):
    name, shape = spec[0], spec[1]
    cx, cy = spec[2], spec[3]
    if shape == 'cylinder':
        r, h = spec[4], spec[5]
        geom = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
    else:  # box
        sx, sy, h = spec[4], spec[5], spec[6]
        geom = f"<box><size>{sx} {sy} {h}</size></box>"
    z = (spec[5] if shape == 'cylinder' else spec[6]) / 2.0
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry>{geom}</geometry></collision>
        <visual name="v">
          <geometry>{geom}</geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{GZ['GREY']}</name></script></material>
        </visual>
      </link>
    </model>"""


def main():
    parts = []
    landmarks = []   # (color, x, y) ground-truth

    # 둘레 흰 벽 4개
    parts.append(wall('wall_bottom', 0.0, -HALF, WALL_LEN, 'x', GZ['WHITE']))
    parts.append(wall('wall_top',    0.0,  HALF, WALL_LEN, 'x', GZ['WHITE']))
    parts.append(wall('wall_left',  -HALF, 0.0,  WALL_LEN, 'y', GZ['WHITE']))
    parts.append(wall('wall_right',  HALF, 0.0,  WALL_LEN, 'y', GZ['WHITE']))

    # RGB 패널
    for name, color, wname, along in PANELS:
        block, cx, cy = panel(name, color, wname, along)
        parts.append(block)
        landmarks.append((color, cx, cy))

    # 장애물
    for spec in OBSTACLES:
        parts.append(obstacle(spec))

    body = "\n\n".join(parts)
    doc = f"""<?xml version="1.0" ?>
<!-- 자동 생성: generate_world.py — 흰벽 5x5m 방 + RGB 단색 패널 9개 + 무채색 장애물 -->
<sdf version="1.6">
  <world name="color_room">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

{body}

  </world>
</sdf>
"""
    out_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "worlds")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "color_room.world")
    with open(out_path, "w") as f:
        f.write(doc)

    print(f"wrote {out_path}")
    print(f"room {2*HALF}x{2*HALF}m, walls WHITE h={WALL_H}m, "
          f"{len(PANELS)} panels, {len(OBSTACLES)} obstacles, start={START}")
    print("\n# ground-truth 패널 좌표 (재매핑된 color_landmarks.yaml sanity-check 용)")
    for color in ('RED', 'GREEN', 'BLUE'):
        pts = [f"({x:.2f},{y:.2f})" for c, x, y in landmarks if c == color]
        print(f"  {color}: {', '.join(pts)}")


if __name__ == "__main__":
    main()
