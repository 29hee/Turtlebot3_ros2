#!/usr/bin/env python3
"""
test_maze_logic.py
maze_common.py 순수 로직 회귀 테스트 — ROS2/OpenCV/numpy/yaml 불필요.
개발 머신에서 `python3 test_maze_logic.py` 또는 `pytest` 로 실행 가능.

검증 대상(사양 매핑):
  - is_confirmed          : confirm 임계 경계(AC5). 현재 임계=0.30(임시),
                            사양 최종 목표는 0.60 — 아래 테스트 참고.
  - select_target_walls   : no-match 경로(AC7), target 벽 추출(AC4)
  - order_walls           : 모든 벽 순회 순서(AC4/6)
  - approach_pose         : 벽 앞 접근 포즈 기하
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from maze_common import (   # noqa: E402
    is_confirmed, normalize_color, approach_pose, order_walls,
    select_target_walls, resolve_target_walls, cluster_cells, filter_clusters,
    CONFIRM_THRESHOLD,
)


def test_confirm_threshold_boundary():
    # 현재 임계 = 0.30 (임시). burger_cam 초광각 FOV 탓에 60%에 도달 못 해 우선 30%로 통일.
    # ⚠️ 사양 최종 목표는 0.60 — 실카메라 화각 + ROI/PID 정렬 적용 후 0.30→0.60 까지
    #    올리며 검증해야 한다(maze_common.CONFIRM_THRESHOLD 의 TODO 참고). 그때 이 값도 갱신.
    assert CONFIRM_THRESHOLD == 0.30
    # is_confirmed 는 임계 경계 '>= 임계' 를 포함한다(경계 동작은 임계값과 무관하게 고정).
    assert is_confirmed(CONFIRM_THRESHOLD) is True            # 경계 포함
    assert is_confirmed(CONFIRM_THRESHOLD + 0.01) is True
    assert is_confirmed(CONFIRM_THRESHOLD - 0.0001) is False
    assert is_confirmed(0.0) is False
    # 사양 목표(0.60) 로 올렸을 때의 경계 동작도 미리 고정해 둔다(임계를 인자로 직접 지정).
    assert is_confirmed(0.60, threshold=0.60) is True
    assert is_confirmed(0.5999, threshold=0.60) is False


def test_normalize_color():
    assert normalize_color('red') == 'RED'
    assert normalize_color(' Green ') == 'GREEN'
    assert normalize_color('BLUE') == 'BLUE'
    assert normalize_color('purple') is None
    assert normalize_color(None) is None


def test_select_target_walls_nomatch():
    lm = {'RED': [{'x': 1.0, 'y': 0.0, 'votes': 5}], 'GREEN': [], 'BLUE': []}
    # 존재하는 색
    assert len(select_target_walls(lm, 'red')) == 1
    # 빈 색 / 없는 색 / 잘못된 색 → no-match(빈 리스트)
    assert select_target_walls(lm, 'green') == []
    assert select_target_walls(lm, 'blue') == []
    assert select_target_walls(lm, 'purple') == []
    assert select_target_walls({}, 'red') == []


def test_order_walls_nearest_neighbor():
    walls = [
        {'x': 2.0, 'y': 0.0}, {'x': 0.5, 'y': 0.0}, {'x': 1.0, 'y': 0.0},
    ]
    order = order_walls(walls, (0.0, 0.0))
    xs = [w['x'] for w in order]
    assert xs == [0.5, 1.0, 2.0]          # 가까운 순으로 한붓그리기
    assert len(order) == len(walls)        # 모든 벽 포함(누락 없음)


def test_approach_pose_geometry():
    wx, wy, standoff = 1.0, 0.0, 0.45
    ax, ay, yaw = approach_pose(wx, wy, standoff, center=(0.0, 0.0))
    # 접근점은 벽에서 standoff 만큼 떨어져 있다
    assert abs(math.hypot(wx - ax, wy - ay) - standoff) < 1e-6
    # 접근점은 벽보다 중심(0,0)에 가깝다(자유공간 쪽)
    assert math.hypot(ax, ay) < math.hypot(wx, wy)
    # yaw 는 접근점에서 벽을 바라본다(+x 벽이면 yaw≈0)
    assert abs(yaw - 0.0) < 1e-6


def test_approach_pose_degenerate_center():
    # 벽이 정확히 중심이면 +x 폴백
    ax, ay, yaw = approach_pose(0.0, 0.0, 0.45, center=(0.0, 0.0))
    assert abs(ax - 0.45) < 1e-6 and abs(ay) < 1e-6


def test_cluster_cells_merges_adjacent():
    cells = [
        {'x': 0.0, 'y': 0.0, 'votes': 5},
        {'x': 0.3, 'y': 0.0, 'votes': 5},   # 0.3<=0.5 → 위와 병합
        {'x': 2.0, 'y': 0.0, 'votes': 5},   # 멀리 → 별도
    ]
    clusters = cluster_cells(cells, merge_dist=0.5)
    assert len(clusters) == 2
    merged = [c for c in clusters if c['votes'] == 10][0]
    assert abs(merged['x'] - 0.15) < 1e-6     # 표 가중평균 중심
    assert abs(merged['y'] - 0.0) < 1e-6


def test_filter_clusters_relative_and_floor():
    clusters = [{'x': 0, 'y': 0, 'votes': 100},
                {'x': 1, 'y': 0, 'votes': 30},
                {'x': 2, 'y': 0, 'votes': 8}]
    # thr = max(floor=15, 0.15*100=15) = 15 → 8 탈락, 100/30 채택
    kept = filter_clusters(clusters, frac=0.15, floor=15)
    assert sorted(c['votes'] for c in kept) == [30, 100]
    assert filter_clusters([]) == []


def test_resolve_target_walls_clusters_and_ids():
    lm = {
        'RED': [
            {'x': -1.5, 'y': -2.4, 'votes': 40}, {'x': -1.3, 'y': -2.4, 'votes': 30},  # wall A
            {'x': 2.4, 'y': 1.2, 'votes': 50},                                          # wall B
            {'x': -2.4, 'y': 0.8, 'votes': 45},                                         # wall C
            {'x': 0.0, 'y': 0.0, 'votes': 3}, {'x': 1.0, 'y': 1.0, 'votes': 5},        # 노이즈
        ],
        'GREEN': [], 'BLUE': [],
    }
    walls = resolve_target_walls(lm, 'red')
    assert len(walls) == 3                       # 노이즈 2개 제거, 인접 셀 병합
    assert [w['id'] for w in walls] == [1, 2, 3]  # 위치정렬 안정 id
    # 위치 사전식 정렬: (-2.4,0.8)=1, (-1.41,-2.4)=2, (2.4,1.2)=3
    byid = {w['id']: w for w in walls}
    assert abs(byid[3]['x'] - 2.4) < 1e-6 and abs(byid[3]['y'] - 1.2) < 1e-6
    assert byid[2]['votes'] == 70                # wall A 병합 합산
    # no-match
    assert resolve_target_walls(lm, 'green') == []
    assert resolve_target_walls(lm, 'purple') == []


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    failed = 0
    for t in tests:
        try:
            t()
            print(f'  PASS  {t.__name__}')
        except AssertionError as e:
            failed += 1
            print(f'  FAIL  {t.__name__}: {e}')
    print(f'\n{len(tests) - failed}/{len(tests)} passed')
    return failed


if __name__ == '__main__':
    sys.exit(1 if _run_all() else 0)
