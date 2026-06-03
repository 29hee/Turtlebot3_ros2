#!/usr/bin/env python3
"""
color_maze.world 생성기 (Gazebo Classic 11)

핵심: 벽 하나를 '동일 크기의 정사각 타일' 모자이크로 구성한다.
  - 모든 타일 크기 동일: TILE x TILE (면) x THICK (두께)
  - 한 벽 면에 빨강/초록/파랑 타일이 섞임  → 벽마다 색 패턴이 생김
  - 충돌(collision)은 벽 전체 박스 1개, 시각(visual)만 타일로 잘게 쪼갬

미로 구조(serpentine, 외길) 는 그대로:
  외곽 4x4m, 내부 1m 격자, 가로 장벽 3줄(y=-1,0,1) 에 한 칸씩 통로.
"""

import random

BLOCK = 0.5      # 길이방향 블록 폭 (m) - 모든 블록 동일
THICK = 0.15     # 벽 두께 (m)
HEIGHT = 0.4     # 벽 높이 (m). 블록 0.5(폭) x 0.4(높이) -> 가로로 긴 직사각형
PALETTE = ["Gazebo/Red", "Gazebo/Green", "Gazebo/Blue"]


def pick_colors(n, seed):
    """블록 색을 정한다. 순환(R,G,B,R,G,B) 아님: seed 기반으로 뽑되
    인접 블록끼리만 다르게 강제 -> 벽마다 비반복 배색."""
    rng = random.Random(seed)
    cols = []
    for _ in range(n):
        choices = [c for c in range(3) if not cols or c != cols[-1]]
        cols.append(rng.choice(choices))
    return cols


def blocks_for_wall(length, axis, seed):
    """벽을 길이방향으로 동일 크기 블록(가로로 긴 직사각형)으로 채운 visual 리스트.
    각 블록은 풀높이 단색 -> 세로 방향 색 동일.
    axis='x': 벽이 x방향으로 뻗음(가로벽) / axis='y': y방향(세로벽)
    """
    n_len = round(length / BLOCK)
    colors = pick_colors(n_len, seed)
    out = []
    for i in range(n_len):
        off_len = -length / 2 + BLOCK / 2 + i * BLOCK
        color = PALETTE[colors[i]]
        if axis == 'x':
            pose = f"{off_len:.4f} 0 0 0 0 0"
            size = f"{BLOCK} {THICK} {HEIGHT}"
        else:  # 'y'
            pose = f"0 {off_len:.4f} 0 0 0 0"
            size = f"{THICK} {BLOCK} {HEIGHT}"
        out.append(f"""        <visual name="b_{i}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{color}</name></script></material>
        </visual>""")
    return "\n".join(out)


def wall(name, cx, cy, length, axis, seed):
    """타일 모자이크 벽 model 1개."""
    if axis == 'x':
        coll_size = f"{length} {THICK} {HEIGHT}"
    else:
        coll_size = f"{THICK} {length} {HEIGHT}"
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {HEIGHT/2} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{coll_size}</size></box></geometry></collision>
{blocks_for_wall(length, axis, seed)}
      </link>
    </model>"""


def main():
    walls = []
    # ---- 외곽 정사각형 (length 4.0) ----
    walls.append(wall("wall_outer_bottom", 0, -2, 4.0, 'x', 0))
    walls.append(wall("wall_outer_top",    0,  2, 4.0, 'x', 1))
    walls.append(wall("wall_outer_left",  -2,  0, 4.0, 'y', 2))
    walls.append(wall("wall_outer_right",  2,  0, 4.0, 'y', 0))

    # ---- 내부 가로 장벽 (각 1m 세그먼트) ----
    # 장벽1 y=-1 : col0,1,2 (col3=x1.5 열림)
    for j, cx in enumerate((-1.5, -0.5, 0.5)):
        walls.append(wall(f"wall_b1_c{j}", cx, -1.0, 1.0, 'x', j))
    # 장벽2 y=0 : col1,2,3 (col0=x-1.5 열림)
    for j, cx in enumerate((-0.5, 0.5, 1.5)):
        walls.append(wall(f"wall_b2_c{j}", cx, 0.0, 1.0, 'x', j + 1))
    # 장벽3 y=1 : col0,1,2 (col3=x1.5 열림)
    for j, cx in enumerate((-1.5, -0.5, 0.5)):
        walls.append(wall(f"wall_b3_c{j}", cx, 1.0, 1.0, 'x', j + 2))

    body = "\n\n".join(walls)
    doc = f"""<?xml version="1.0" ?>
<!-- 자동 생성: generate_world.py  (블록 {BLOCK}x{HEIGHT}m 가로직사각형, 세로 동일색, 순환아님) -->
<sdf version="1.6">
  <world name="color_maze">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.7 0.7 1</background>
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
    import os
    out_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "worlds", "color_maze.world")
    with open(out_path, "w") as f:
        f.write(doc)
    n_blocks_internal = round(1.0 / BLOCK)
    print(f"wrote {out_path}")
    print(f"block = {BLOCK}(폭) x {HEIGHT}(높이) m 가로직사각형, 내부벽 1개당 블록 {n_blocks_internal}개 (세로 동일색, 순환아님)")


if __name__ == "__main__":
    main()
