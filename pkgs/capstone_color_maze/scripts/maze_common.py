#!/usr/bin/env python3
"""
maze_common.py
색미로 순회(color-tour) 파이프라인의 '순수 로직' 모음.

여기에는 ROS2 / OpenCV / numpy / yaml 에 의존하지 않는 함수만 둔다. 덕분에
원격 PC 가 아닌 개발 머신에서도 pytest 로 기하·임계·순회 로직을 검증할 수 있다.
(HSV 마스크 계산 같은 cv2 의존 코드는 각 노드 안에 둔다.)

공유 상수:
  COLOR_RANGES   : color_detector / color_confirm / color_mapper 가 같은 값을 쓰도록 단일 출처.
  CONFIRM_THRESHOLD : 런타임 벽 확인 임계(프레임 대비 마스크 비율). 사양: >= 0.60.
"""
import math

# ── HSV 색 범위 (OpenCV: H 0~179, S 0~255, V 0~255) ──────────────────────────
# 단일 출처(single source). color_detector / color_confirm / color_mapper 가 모두
# 여기서 import 한다 — 값 drift 방지.
#
# 실물 미로 기준 튜닝: 색 벽은 R/G/B 만, 나머지 벽은 '흰색'.
#   · Hue 는 R/G/B 가 ~120° 간격이라 겹칠 일이 적음 → 각 색 중심 ±소폭으로 좁게.
#   · S_min 이 '흰 벽/허연 반사와 색 벽을 가르는' 핵심. 너무 낮으면 흐릿한 색·글레어
#     가장자리까지 잡힘 → 100~110 으로 올려 '진한 색만' 통과시킨다.
#   · V_min 은 그림자/어두운 면 노이즈 컷(60~70).
# 조명/카메라가 바뀌면 color_detector.py(-p show:=true)로 마스크 보며 S_min 부터 재보정.
COLOR_RANGES = {
    # 빨강은 Hue 가 0 부근에서 끊겨 양끝 두 구간을 OR.
    'RED':   [((0,   150, 80),  (10,  255, 255)),
             ((170, 150, 80),  (179, 255, 255))],
    'GREEN': [((40,  130, 80),  (80,  255, 255))],
    'BLUE':  [((100, 150, 80),  (128, 255, 255))],
}

VALID_COLORS = ('RED', 'GREEN', 'BLUE')

# 사양 목표: "target-color HSV mask must cover at least 60% of the camera frame".
# 현재값 = 0.30 (임시). burger_cam 은 fov 약 182° 초광각이라 작품을 정면에서 봐도
# 프레임 점유율이 낮아 60%에 도달하지 못한다 → 우선 30%로 통일해 파이프라인을 굴린다.
# ⚠️ TODO(사양 복귀): 실카메라 화각 + ROI/PID 정면정렬을 붙인 뒤, 임계를 0.30 → 0.60 까지
#    단계적으로 올리며 각 단계에서 confirm 동작을 검증해야 한다(최종 목표는 60%).
#    임계를 바꾸면 tests/test_maze_logic.py::test_confirm_threshold_boundary 도 같이 갱신할 것.
CONFIRM_THRESHOLD = 0.30


def normalize_color(name):
    """'red' / ' Red ' / 'RED' → 'RED'. 유효하지 않으면 None."""
    if name is None:
        return None
    c = str(name).strip().upper()
    return c if c in VALID_COLORS else None


def parse_target(s):
    """'RED_1' / 'RED 1' / 'RED' → (color, digit).
    digit 없으면 None. 색이 유효하지 않으면 (None, None).
    예: 'RED_1' → ('RED', 1),  'GREEN' → ('GREEN', None)
    """
    if not s:
        return None, None
    parts = str(s).strip().upper().replace(' ', '_').split('_', 1)
    color = normalize_color(parts[0])
    if color is None:
        return None, None
    digit = None
    if len(parts) == 2:
        try:
            digit = int(parts[1])
        except ValueError:
            pass
    return color, digit


def is_confirmed(coverage, threshold=CONFIRM_THRESHOLD):
    """프레임 대비 마스크 비율 coverage(0~1)가 임계 이상이면 True.
    사양은 '>= 60%' 이므로 경계값(정확히 0.60)은 확인으로 친다."""
    return coverage >= threshold


def approach_pose(wall_x, wall_y, standoff, center=(0.0, 0.0)):
    """벽 점(wall_x, wall_y)에서 미로 중심(center) 쪽 자유공간으로 standoff[m]
    떨어진 접근 포즈를 만든다. yaw 는 벽을 바라보도록(=중심 반대 방향) 정한다.

    반환: (ax, ay, yaw)
    벽이 정확히 중심이면 방향이 모호하므로 +x 방향으로 폴백한다.
    """
    cx, cy = center
    vx, vy = cx - wall_x, cy - wall_y   # 벽 -> 중심(자유공간) 방향
    n = math.hypot(vx, vy)
    if n < 1e-3:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = vx / n, vy / n
    ax, ay = wall_x + standoff * ux, wall_y + standoff * uy
    yaw = math.atan2(wall_y - ay, wall_x - ax)   # 접근점 -> 벽 (벽을 바라봄)
    return ax, ay, yaw


def order_walls(walls, start_xy):
    """target 색 벽 목록을 start_xy 에서 출발하는 nearest-neighbor 탐욕 순회로 정렬.

    walls: [{'x':..,'y':..,'votes':..}, ...]  (votes 는 있어도 없어도 됨)
    start_xy: (x, y)
    반환: 같은 dict 들을 방문 순서대로 담은 새 리스트.

    마지막 원소가 '마지막으로 확인할 벽' 후보가 된다(런타임은 거기서 정지).
    """
    remaining = list(walls)
    ordered = []
    cx, cy = start_xy
    while remaining:
        nxt = min(remaining, key=lambda w: math.hypot(w['x'] - cx, w['y'] - cy))
        ordered.append(nxt)
        remaining.remove(nxt)
        cx, cy = nxt['x'], nxt['y']
    return ordered


def select_target_walls(landmarks, target_color):
    """landmarks(dict, color-keyed)에서 target_color 의 '원시 셀' 목록을 뽑는다.
    target_color 정규화가 실패하거나 해당 색 키가 비면 빈 리스트.

    landmarks 예: {'RED':[{'x':..,'y':..,'votes':..}], 'GREEN':[...], ...}
    주의: 이건 격자 투표 '셀' 단위(노이즈 포함). 실제 '벽'은 resolve_target_walls 로.
    """
    c = normalize_color(target_color)
    if c is None:
        return []
    return list(landmarks.get(c) or [])


# ── 셀 → 벽: 클러스터링 + 노이즈 필터 + 안정 인덱스 ──────────────────────────
# color_landmarks.yaml 은 0.30m 격자 '셀' 단위 투표라, 한 벽이 여러 셀로 쪼개지고
# 1~몇 표짜리 노이즈 셀도 섞인다. 런타임은 다음으로 '진짜 벽'만 추린다:
#   1) 같은 색 셀을 거리 MERGE_DIST 단일연결(single-linkage) 병합 → 클러스터
#      (중심 = 표 가중평균, votes = 합산)
#   2) 색별 상대임계(VOTE_FRAC * 그 색 최대 클러스터 표) + 절대바닥(VOTE_FLOOR) 로 필터
#   3) 위치 정렬로 '안정적인' 1-based 색별 id 부여  ("RED #3" 처럼 로그/지칭용)
MERGE_DIST = 0.5     # 같은 벽으로 볼 셀 간 거리 [m] (grid 0.30 < 0.5 < 벽 간격)
VOTE_FRAC = 0.15     # 그 색 최대 클러스터 표의 이 비율 이상이어야 채택
VOTE_FLOOR = 15      # 그와 별개로 최소 이만큼은 득표해야 채택(절대 바닥)


def cluster_cells(cells, merge_dist=MERGE_DIST):
    """같은 색 셀들을 단일연결 병합. cells: [{'x','y','votes'}].
    반환: [{'x','y','votes'}] (병합 클러스터; 중심=표 가중평균, votes=합)."""
    pts = [c for c in cells if c.get('votes', 0) > 0]
    n = len(pts)
    if n == 0:
        return []
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i in range(n):
        for j in range(i + 1, n):
            if math.hypot(pts[i]['x'] - pts[j]['x'], pts[i]['y'] - pts[j]['y']) <= merge_dist:
                parent[find(i)] = find(j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(pts[i])

    out = []
    for g in groups.values():
        tv = sum(c['votes'] for c in g)
        cx = sum(c['x'] * c['votes'] for c in g) / tv
        cy = sum(c['y'] * c['votes'] for c in g) / tv
        # digit: 클러스터 내 최다 득표 셀의 digit 채택
        cells_with_digit = [c for c in g if c.get('digit') is not None]
        entry = {'x': cx, 'y': cy, 'votes': tv}
        if cells_with_digit:
            entry['digit'] = max(cells_with_digit, key=lambda c: c['votes'])['digit']
        out.append(entry)
    return out


def filter_clusters(clusters, frac=VOTE_FRAC, floor=VOTE_FLOOR):
    """색별 상대임계 + 절대바닥으로 노이즈 클러스터 제거."""
    if not clusters:
        return []
    mx = max(c['votes'] for c in clusters)
    thr = max(floor, frac * mx)
    return [c for c in clusters if c['votes'] >= thr]


def resolve_target_walls(landmarks, target_color,
                         merge_dist=MERGE_DIST, frac=VOTE_FRAC, floor=VOTE_FLOOR):
    """color_landmarks.yaml(dict) + target_color → '진짜 벽' 목록.

    각 벽 dict: {'x','y','votes','id'}.  id 는 위치 정렬 기반의 안정적 1-based 색별
    인덱스(예: RED 의 3번째 벽 = id 3). 방문 순서(order_walls)와는 별개로 고정이라
    "빨강 3번에 도착" 같은 로그/지칭에 쓴다.

    해당 색 벽이 없으면 빈 리스트(= no-match).
    """
    cells = select_target_walls(landmarks, target_color)
    walls = filter_clusters(cluster_cells(cells, merge_dist), frac, floor)
    # 안정 id: 위치 사전식 정렬(재현 가능). 방문 순서가 아니라 '신원' 부여.
    walls.sort(key=lambda w: (round(w['x'], 3), round(w['y'], 3)))
    for i, w in enumerate(walls, 1):
        w['id'] = i
    return walls
