#!/usr/bin/env python3
"""
maze_explorer.py — 색-반응형 매핑 탐사 주행 (scan_explorer 대체).

[왜 새로 쓰나]
구 scan_explorer 는 ①시간 기반 + ②하드코딩 시뮬 웨이포인트 + ③막히면 제자리 회전 이라
실공간에서 '같은 자리 빙빙' 이 났다. 또 색을 멀리서 스치듯 봐 색·숫자 매핑이 부실했다.
→ 본 노드는 '벽을 타며 색을 찾고, 색을 보면 그 패널 정면 ~0.3m 까지 비주얼 서보로
  접근해 멈춰서(dwell) 그 근접 위치에서만 색·숫자를 쌓게' 한다. 기록 자체는 옆에서 도는
  color_mapper(색·digit 격자투표) + digit_recognizer(EasyOCR) 가 담당하고, 이 노드는
  '좋은 관측 위치로 로봇을 데려가는' 역할만 한다(역할 분리).

[같은 자리 빙빙 금지 — 필수 안전장치]
  A. 방문 격자 메모리 + 미방문 지향(하드코딩 웨이포인트 폐기).
  B. 진행 워치독: 윈도 동안 이동거리 < stuck_dist 면 '갇힘' → 강제 탈출 기동.
  C. loop/orbit 감지: 시작 셀로 충분히 이동 후 복귀 시 그 루프 종료 → 다음 미방문 구역.
  D. TF 끊김 시 무한 제자리 회전 금지(짧게만, 이후 느린 전진으로 칸 바꿔 재수렴).
  E. 종료는 시간이 아니라 '미방문 소진' (시간은 안전 상한일 뿐).

[2국면]  PERIMETER(둘레 벽타기) → loop closure → INWARD(중앙 진입) → ISLAND(섬 벽타기).
  어느 국면이든 색을 보면 APPROACH→CAPTURE→RESUME 가 끼어든다.

전제: TF 'map' 프레임(slam_toolbox), 회전은 느리게(맵 안 깨지게).
실행:  python3 maze_explorer.py --ros-args -p use_sim_time:=true
"""
import math
import os
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray, Int32, Bool
import tf2_ros

from maze_common import id_to_color, resolve_target_walls, VALID_COLORS
# id_to_color: color_id → 'RED' 등 (vision_node 와 동일 표)
# resolve_target_walls/VALID_COLORS: 품질 게이트에서 색+숫자 '벽' 수 집계용


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class MazeExplorer(Node):
    def __init__(self, duration):
        super().__init__('maze_explorer')
        # use_sim_time 은 --ros-args -p 로 넘기면 rclpy 가 자동 선언하므로 여기서 선언하지 않는다
        # (중복 선언하면 ParameterAlreadyDeclaredException 으로 노드가 죽는다).

        # ── 주행/벽타기 파라미터 ─────────────────────────────────────
        self.declare_parameter('v_fwd', 0.08)        # 전진 속도 [m/s] (느리게 — 색 놓치지 않게)
        self.declare_parameter('target_right', 0.6)   # 오른쪽 벽 유지 거리 [m] (클수록 벽과 멀리)
        self.declare_parameter('front_stop', 0.45)    # 전방 정지 임계 [m]
        self.declare_parameter('spin_speed', 0.25)    # 회전 각속도 [rad/s] (느리게! SLAM 보호)
        # ── 색 접근(비주얼 서보) 파라미터 ───────────────────────────
        # 작은 색 조각(≈5%)만 보여도 그쪽으로 정렬·접근하도록 낮게 둔다.
        self.declare_parameter('seen_ratio', 0.03)    # ROI 색 점유율 ≥ 이 값이면 '발견' → 접근
        self.declare_parameter('standoff', 0.30)      # 패널 앞 정지 거리(라이다 정면) [m]
        self.declare_parameter('capture_secs', 6.0)   # 근접 dwell 상한 [s] (숫자 확실 확인 시 조기복귀)
        self.declare_parameter('dedup_dist', 0.6)     # 이 거리 내 이미 캡처한 패널이면 재접근 스킵 [m]
        # ── 안티-스턱 파라미터 ───────────────────────────────────────
        self.declare_parameter('visit_res', 0.4)      # 방문 격자 한 변 [m]
        self.declare_parameter('stuck_win', 8.0)      # 진행 점검 윈도 [s]
        self.declare_parameter('stuck_dist', 0.2)     # 윈도 동안 이 거리 미만 이동이면 '갇힘' [m]
        self.declare_parameter('escape_secs', 3.0)    # 탈출 기동 지속 [s]
        self.declare_parameter('loop_close_dist', 0.5)  # 시작 셀 복귀 판정 거리 [m]
        # ★ 조기종료 차단: 충분히 탐색하기 전엔 'loop 완료'로 안 본다(작은 원에 끝나던 문제).
        self.declare_parameter('min_visited_for_loop', 40)  # 이 방문셀 수 전엔 loop closure 금지
        self.declare_parameter('loop_left_dist', 1.5)       # 시작셀에서 이만큼 벗어나야 '떠남' 인정 [m]
        self.declare_parameter('tf_lost_spin', 1.5)   # TF 끊김 시 제자리 회전 최대 [s]
        # 색 접근이 이 시간 안에 근접(standoff) 도달 못 하면 포기하고 벽타기 복귀.
        #   (어안렌즈로 색 중심 정렬이 안 돼 제자리 회전만 하는 무한루프 방지.)
        self.declare_parameter('approach_timeout', 14.0)
        # ── 센서/odom 생존 워치독 ────────────────────────────────────
        #   라이다(/scan)나 위치(TF map→base_link)가 끊기면(예: turtlebot3_node 배터리로
        #   사망, 라이다 멈춤) '탐사 중'으로 착각하며 헛돌지 않게 즉시 정지+경고한다.
        self.declare_parameter('sensor_timeout', 3.0)    # /scan 이 이 시간 끊기면 정지 [s]
        self.declare_parameter('pose_lost_limit', 12.0)  # TF 위치가 이 시간 끊기면 정지+경고 [s]
        # ── 매핑 종료 품질 게이트 ────────────────────────────────────
        #   자연 종료(둘레/섬 한 바퀴) 시 color_landmarks.yaml 의 '색+숫자 벽' 수가
        #   min_quality_walls 미만이면 종료하지 않고 재탐사(놓친 숫자 재수집). 0=비활성.
        #   시간 상한(total)은 그대로 안전망이라 무한루프 없음.
        self.declare_parameter('min_quality_walls', 1)   # 종료 전 필요한 색+숫자 벽 최소수(0=끔)
        self.declare_parameter('max_resweeps', 2)        # 품질 미달 시 재탐사 최대 횟수
        # 2-pass 매핑: true 면 Phase1(탐사+색좌표)만 하고 색 접근/숫자 dwell 안 함.
        #   탐사 완료 시 /phase1_done 발행하고 idle → digit_finalizer(Phase2)가 정면방문해 숫자 확정.
        self.declare_parameter('two_pass', False)
        # 접근 중 색이 잠깐 NONE 으로 '깜빡'해도 이 시간 안엔 중단하지 않는다(오실레이션 방지).
        #   어안렌즈+낮은 점유율(3%)이라 색이 자주 깜빡 → 즉시중단하면 제자리 빙빙이 난다.
        self.declare_parameter('approach_lost_grace', 1.5)   # 색 끊김 허용 [s]
        # 접근 스톨 감지: 시간은 넉넉히 주되(approach_timeout), 그 안에서 '실제로 안 다가가면'
        #   (윈도 동안 이동 < min_move = 제자리 회전만) 빨리 포기해 한 지점 빙빙을 끊는다.
        self.declare_parameter('approach_stall_win', 5.0)    # 이 시간 이동 거의 0이면 포기 [s]
        self.declare_parameter('approach_min_move', 0.08)    # 그 윈도 최소 이동 [m]
        # 캡처/스킵 직후 '그 타겟을 벗어날 때까지' 색 무시하고 전진(같은 타겟 즉시 재트리거 차단).
        self.declare_parameter('moveon_secs', 4.0)           # 벗어나기 최대 시간 [s]
        self.declare_parameter('moveon_dist', 0.5)           # 벗어나기 목표 이동거리 [m]
        # ★ 정면 정렬: APPROACH 도착(가까워짐) 후 숫자 읽기 전, 제자리에서 고개를 돌려
        #   '패널(색) 한가운데'를 보도록 color blob cx 를 0 으로 맞춘다(옆에서 비스듬히 읽기 방지).
        self.declare_parameter('align_cx_tol', 0.10)         # 색 중심 허용오차(|cx|)
        self.declare_parameter('align_secs', 8.0)            # 정면 재배치 최대 시간 [s]
        # ★ 정면 재배치: 색까지 라이다 거리가 이 값 이하로 가까워지면, 라이다로 벽 법선을 구해
        #   '패널 바로 앞(수직·정면)' 으로 이동한 뒤 숫자를 읽는다(대각선 읽기 제거).
        self.declare_parameter('frontal_start', 0.6)         # 정면 재배치 시작 거리 [m]
        self.declare_parameter('frontal_pos_tol', 0.12)      # 정면 목표 위치 허용오차 [m]
        # 색 blob 화면중심(cx: -1~+1)을 방위각으로 환산하는 스케일 — 타겟추정/중복판정 정확도용.
        #   cx=±1 ≈ ±이 각도. 어안 ROI 기준 대략값(같은 패널을 옆에서 봐도 같은 좌표로 추정).
        self.declare_parameter('cx_fov_deg', 60.0)
        # 색 접근 거리 게이트: 그 색 방향(라이다)까지 이 거리보다 멀면 접근 안 함.
        #   멀리 스쳐 보인 색(중앙 박스/복도 끝)으로 헛걸음하지 않게. color_mapper max_range 와 짝.
        self.declare_parameter('approach_max_dist', 1.2)
        # 색별 목표 개수 — 이만큼 색+숫자 벽을 이미 확보한 색은 더 접근/인식 안 함(완료 색 무시).
        self.declare_parameter('per_color_target', 3)

        self.total = duration
        self.v_fwd = float(self.get_parameter('v_fwd').value)
        self.target_right = float(self.get_parameter('target_right').value)
        self.front_stop = float(self.get_parameter('front_stop').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)
        self.seen_ratio = float(self.get_parameter('seen_ratio').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.capture_secs = float(self.get_parameter('capture_secs').value)
        self.dedup_dist = float(self.get_parameter('dedup_dist').value)
        self.visit_res = float(self.get_parameter('visit_res').value)
        self.stuck_win = float(self.get_parameter('stuck_win').value)
        self.stuck_dist = float(self.get_parameter('stuck_dist').value)
        self.escape_secs = float(self.get_parameter('escape_secs').value)
        self.loop_close_dist = float(self.get_parameter('loop_close_dist').value)
        self.tf_lost_spin = float(self.get_parameter('tf_lost_spin').value)
        self.min_visited_for_loop = int(self.get_parameter('min_visited_for_loop').value)
        self.loop_left_dist = float(self.get_parameter('loop_left_dist').value)
        self.approach_timeout = float(self.get_parameter('approach_timeout').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.pose_lost_limit = float(self.get_parameter('pose_lost_limit').value)
        self.min_quality_walls = int(self.get_parameter('min_quality_walls').value)
        self.max_resweeps = int(self.get_parameter('max_resweeps').value)
        self.two_pass = bool(self.get_parameter('two_pass').value)
        self._phase1_done = False
        self.approach_lost_grace = float(self.get_parameter('approach_lost_grace').value)
        self.approach_stall_win = float(self.get_parameter('approach_stall_win').value)
        self.approach_min_move = float(self.get_parameter('approach_min_move').value)
        self.moveon_secs = float(self.get_parameter('moveon_secs').value)
        self.moveon_dist = float(self.get_parameter('moveon_dist').value)
        self.align_cx_tol = float(self.get_parameter('align_cx_tol').value)
        self.align_secs = float(self.get_parameter('align_secs').value)
        self.frontal_start = float(self.get_parameter('frontal_start').value)
        self.frontal_pos_tol = float(self.get_parameter('frontal_pos_tol').value)
        self._frontal_goal = None      # (tx,ty,tyaw) 정면 목표 포즈
        self.cx_fov_deg = float(self.get_parameter('cx_fov_deg').value)
        self.approach_max_dist = float(self.get_parameter('approach_max_dist').value)
        self.per_color_target = int(self.get_parameter('per_color_target').value)
        self._color_counts = {}        # {color: 색+숫자 벽 수} 캐시
        self._color_counts_t = None    # 캐시 갱신 시각
        self._appr_pose0 = None        # 접근 시작 위치(스톨 감지 기준)
        self._appr_stall_t = None      # 스톨 윈도 기준 시각
        self._moveon_pose0 = None      # MOVEON 시작 위치(벗어난 거리 측정)
        self._resweeps = 0
        self._last_color_t = None   # 마지막으로 색을 본 시각(접근 유예 판정)
        self._appr_cx = 0.0         # 마지막 유효 색 중심 cx(깜빡 동안 서보 방향 유지)
        # color_mapper 와 '같은' 색맵 경로(품질 게이트에서 읽음) — launch 가 map_name 으로 맞춰줌.
        _here = os.path.dirname(os.path.realpath(__file__))
        _def_lm = os.path.join(os.path.dirname(_here), 'maps', 'color_landmarks.yaml')
        self.declare_parameter('landmarks_path', _def_lm)
        self._landmarks_path = self.get_parameter('landmarks_path').value

        # ── IO ───────────────────────────────────────────────────────
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # 영상은 직접 안 푼다 — vision_node 가 푼 색 신호만 구독(단일 디코딩).
        self.create_subscription(Float32MultiArray, '/color_signal', self.on_signal, 10)
        # 숫자(digit_recognizer): -1=없음, ≥0=신뢰도 임계 이상으로 확실히 읽힌 숫자.
        #   CAPTURE 중 이게 들어오면 '확실히 발견' → 조기 복귀에 쓴다.
        self.create_subscription(Int32, '/detected_digit', self.on_digit, 10)
        self._digit = -1
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 진척 상황 방송(quality_monitor/로그용): "PERIMETER", "CAPTURE RED@(x,y)" 등
        self.pub_phase = self.create_publisher(String, '/explorer_phase', 10)
        # Phase1(탐사) 완료 신호 — 늦게 뜬 digit_finalizer 도 받게 latched(transient_local).
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_phase1_done = self.create_publisher(Bool, '/phase1_done', _latched)

        # ── 상태 ─────────────────────────────────────────────────────
        self.scan = None
        self.color = 'NONE'          # 최근 우세색
        self.color_cx = 0.0          # 색 blob 중심 x (정규화 -1~+1, +는 화면 오른쪽)
        self.color_cov = 0.0         # ROI 색 점유율
        self.phase = 'PERIMETER'     # PERIMETER | INWARD | ISLAND | (+ APPROACH/CAPTURE/ESCAPE 인터럽트)
        self.interrupt = None        # 'APPROACH' | 'CAPTURE' | 'ESCAPE' | None
        self.start = self.now()
        self.phase_start = self.start
        self.captured = []           # [(x,y), ...] 이미 캡처한 패널 map 좌표(중복 방지)
        self.skipped = []            # [(x,y), ...] 접근 시간초과로 건너뛴 위치(재트리거 방지)
        self.inward_target = None    # 내부(섬) 진입 목표 = 둘레 경로 무게중심(≈방 중앙)
        self.visited = set()         # 방문 격자 셀 인덱스
        self.start_cell = None       # 둘레 loop closure 기준 시작 셀
        self.left_start = False      # 시작 셀을 충분히 벗어났는가(복귀 판정 게이트)
        # 진행 워치독용 표본
        self.wd_pose = None
        self.wd_time = self.start
        # /color_signal 생존 감시 — 안 들어오면 vision_node 가 죽은 것(색 못 봄 → approach 불가)
        self._last_signal_time = None
        self._last_sig_warn = 0.0
        self._last_phase_pub = 0.0
        # 센서/odom 워치독 상태
        self._last_scan_t = None       # 마지막 /scan 수신 시각
        self._pose_lost_since = None   # TF 위치 끊김 시작 시각
        self._last_sensor_warn = 0.0
        self._last_pose_warn = 0.0
        self._need_reset = False       # 지속 TF 끊김 → 복구 시 탐사 재초기화 플래그
        self.get_logger().info(
            f"maze_explorer 시작 — 색-반응 매핑, total(상한)={self.total:.0f}s, "
            f"standoff={self.standoff}m, stuck<{self.stuck_dist}m/{self.stuck_win:.0f}s")
        self.timer = self.create_timer(0.05, self.on_timer)

    # ── 시간/센서 ────────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now()

    def elapsed(self, since):
        return (self.now() - since).nanoseconds / 1e9

    def on_scan(self, msg):
        self.scan = msg
        self._last_scan_t = self.now()   # 센서 워치독용 생존 표시

    def on_signal(self, msg):
        """vision_node 의 [color_id, cx_norm, coverage] → 비주얼 서보/발견 신호.
        seen_ratio 미만 점유율은 NONE 으로 취급(탐사기측 발견 임계)."""
        d = msg.data
        self._last_signal_time = self.now()   # 신호 생존 표시
        if len(d) < 3:
            return
        cov = float(d[2])
        self.color = id_to_color(d[0]) if cov >= self.seen_ratio else 'NONE'
        self.color_cx = float(d[1])
        self.color_cov = cov
        if self.color != 'NONE':
            self._last_color_t = self.now()    # 색 깜빡 유예 기준
            self._appr_cx = self.color_cx      # 깜빡 동안에도 이 방향으로 계속 서보

    def on_digit(self, msg):
        self._digit = int(msg.data)            # ≥0 = 신뢰도 임계 이상으로 확실히 읽힌 숫자

    def sector_min(self, deg_lo, deg_hi):
        s = self.scan
        if s is None:
            return float('inf')
        n = len(s.ranges)
        vals = []
        d = deg_lo
        while d <= deg_hi:
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                vals.append(r)
            d += 1
        return min(vals) if vals else float('inf')

    def color_range(self):
        """현재 보는 색 방향(cx)의 라이다 거리 [m]. 그 방향 벽까지 거리로 본다(접근 거리게이트용)."""
        deg = math.degrees(-self.color_cx * self.cx_fov_deg)
        d = self.sector_min(deg - 12, deg + 12)
        if not math.isfinite(d):
            d = self.sector_min(-15, 15)
        return d

    def nearest_front_angle(self):
        """전방 ±90° 에서 가장 가까운 점(=벽)의 방위각[rad]. 정면(수직) 정렬에 쓴다. 없으면 None."""
        s = self.scan
        if s is None:
            return None
        n = len(s.ranges)
        best_r, best_a = float('inf'), None
        a = -90
        while a <= 90:
            idx = int(round((math.radians(a) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max and r < best_r:
                best_r, best_a = r, math.radians(a)
            a += 2
        return best_a

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def cell(self, x, y):
        return (int(math.floor(x / self.visit_res)), int(math.floor(y / self.visit_res)))

    def visited_centroid(self):
        """방문한 격자 셀들의 중심 = 둘레를 돈 경로의 무게중심 ≈ 방 중앙.
        SLAM 원점(0,0)은 '로봇 시작 위치'이지 방 중앙이 아니므로, 내부(섬) 진입 목표로
        이걸 쓴다(시작점으로 되돌아가 엉뚱한 벽을 섬으로 오인하던 문제 해결)."""
        if not self.visited:
            return (0.0, 0.0)
        xs = [(gx + 0.5) * self.visit_res for (gx, gy) in self.visited]
        ys = [(gy + 0.5) * self.visit_res for (gx, gy) in self.visited]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    # ── 주행 프리미티브 ──────────────────────────────────────────────
    def wall_follow_cmd(self):
        """오른손 벽타기 한 스텝."""
        front = self.sector_min(-20, 20)
        right = self.sector_min(-100, -80)
        cmd = Twist()
        if front < self.front_stop:
            cmd.angular.z = 0.4            # 전방 막힘 → 좌회전(천천히)
        elif right > self.target_right + 0.45:   # 유지거리 + 여유 넘으면 '벽 잃음'
            cmd.linear.x = 0.07            # 오른벽 잃음 → 오른쪽으로 붙기
            cmd.angular.z = -0.3
        else:
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.35, min(0.35, -1.0 * err))
        return cmd

    def approach_cmd(self):
        """비주얼 서보: 색 blob 을 화면 중앙에 두고 라이다 정면거리 standoff 까지 전진.
        (도착여부, Twist). 색을 잃으면 도착 아님으로 반환(상위에서 복귀 처리)."""
        cmd = Twist()
        # 색이 잠깐 NONE 으로 깜빡해도 '마지막 본 방향(_appr_cx)'으로 계속 서보한다.
        #   (즉시 0 으로 만들면 깜빡마다 직진/정지가 튀어 수렴이 깨진다.)
        cx = self._appr_cx
        front = self.sector_min(-15, 15)
        # 중심 정렬: cx>0(오른쪽)이면 우회전(-z). 게인 작게(천천히 그쪽을 본다).
        cmd.angular.z = max(-0.35, min(0.35, -0.6 * cx))
        if front <= self.standoff:
            return True, Twist()              # 도착(정지)
        # 정렬도에 비례해 전진 — 단 '바닥값(floor)'을 둬 |cx| 가 커도 제자리회전만 하지 않고
        #   늘 천천히 다가간다(어안렌즈라 패널이 화면 끝 cx≈0.9 로 잡혀도 전진 유지). 허용폭 0.8.
        align = max(0.25, 1.0 - abs(cx) / 0.8)
        cmd.linear.x = self.v_fwd * align
        return False, cmd

    def publish_phase(self, text):
        self.pub_phase.publish(String(data=text))

    def switch(self, phase=None, interrupt='__keep__'):
        if phase is not None:
            self.phase = phase
        if interrupt != '__keep__':
            # 인터럽트(접근/정렬/캡처 등 정지성 작업) 종료 → 워치독 기준점 리셋.
            #   안 그러면 캡처 동안 '안 움직임'을 갇힘으로 오판해 헛 ESCAPE 친다.
            if interrupt is None:
                self.wd_pose = None
            self.interrupt = interrupt
        self.phase_start = self.now()

    # ── 안티-스턱: 진행 워치독 ──────────────────────────────────────
    def watchdog_stuck(self, pose):
        """윈도 동안 이동거리가 stuck_dist 미만이면 True(갇힘). 표본 갱신 포함."""
        if pose is None:
            return False
        if self.wd_pose is None:
            self.wd_pose, self.wd_time = pose, self.now()
            return False
        if self.elapsed(self.wd_time) >= self.stuck_win:
            moved = math.hypot(pose[0] - self.wd_pose[0], pose[1] - self.wd_pose[1])
            self.wd_pose, self.wd_time = pose, self.now()
            if moved < self.stuck_dist:
                return True
        return False

    # ── 메인 루프 ────────────────────────────────────────────────────
    def on_timer(self):
        # 0) 안전 상한(시간) — 종료는 본래 미방문 소진이지만, 폭주 방지용 상한.
        if self.elapsed(self.start) > self.total:
            if self.two_pass:
                self._finish_phase1(f'시간 상한({self.total:.0f}s) 도달 → Phase1 종료')
            else:
                self.stop_and_quit(f'시간 상한({self.total:.0f}s) 도달 → 정지')
            return
        if self.scan is None:
            return

        # ★ 센서 워치독: /scan 이 sensor_timeout 이상 끊기면 정지(stale scan 으로 헛돌기 방지).
        if (self._last_scan_t is not None
                and self.elapsed(self._last_scan_t) > self.sensor_timeout):
            self.pub.publish(Twist())
            now_s = self.elapsed(self.start)
            if now_s - self._last_sensor_warn > 5.0:
                self._last_sensor_warn = now_s
                self.get_logger().error(
                    f'/scan {self.sensor_timeout:.0f}s+ 끊김 — 라이다/로봇 bringup 확인. 정지.')
            return

        # ★ 색 신호(/color_signal) 생존 감시 — 이게 없으면 색을 못 봐 'approach' 자체가 불가.
        #   (조용히 벽만 도는 대신 큰 소리로 알려 vision_node 누락/사망을 즉시 드러낸다.)
        now_s = self.elapsed(self.start)
        # 카메라가 느리면(1~2Hz) 정상에도 몇 초 공백이 난다 → 8s 임계 + 시작 유예로 오발 방지.
        stale = (self._last_signal_time is None
                 or self.elapsed(self._last_signal_time) > 8.0)
        if now_s > 12.0 and stale and now_s - self._last_sig_warn > 15.0:
            self._last_sig_warn = now_s
            self.get_logger().warn(
                "/color_signal 8s+ 끊김 — vision_node 생존/카메라 Hz 확인 "
                "(카메라가 매우 느리면 정상일 수 있음).")

        # 현재 상태를 1초마다 방송(quality_monitor/사용자가 실시간으로 뭐 하는지 보게)
        if now_s - self._last_phase_pub > 1.0:
            self._last_phase_pub = now_s
            tag = self.interrupt or self.phase
            self.publish_phase(f"{tag} (색:{self.color}, 캡처:{len(self.captured)})")

        pose = self.get_pose()

        # 1) TF 위치 끊김 처리. ★ TF(map→base_link) 없으면 '무조건 정지' — 장님 회전/주행 금지.
        #    (원인 보통: slam map→odom 미발행=Pi↔PC 클럭 어긋남으로 스캔 드롭, 또는 odom 끊김.)
        if pose is None:
            if self._pose_lost_since is None:
                self._pose_lost_since = self.now()
            lost_for = self.elapsed(self._pose_lost_since)
            self.pub.publish(Twist())          # 정지 — 위치 모르고 돌면 위험·무의미
            now_s = self.elapsed(self.start)
            if lost_for > 3.0 and now_s - self._last_pose_warn > 5.0:
                self._last_pose_warn = now_s
                self.get_logger().error(
                    f'map→base_link {lost_for:.0f}s 없음 — slam map→odom 미발행(Pi↔PC 클럭 어긋남→'
                    f'스캔 드롭?) 또는 odom 끊김. 정지 대기. (chronyc makestep / slam·배터리 확인)')
            if lost_for > self.pose_lost_limit:
                self._need_reset = True        # 지속 끊김 → 복구되면 재초기화
            return
        # 위치 정상 — 지속 끊김 후 복구면 탐사 재초기화하고 재개(누적 색 캡처는 보존).
        if self._pose_lost_since is not None:
            lost_was = self.elapsed(self._pose_lost_since)
            self._pose_lost_since = None
            if self._need_reset:
                self._need_reset = False
                self.get_logger().info(f'TF 복구(끊김 {lost_was:.0f}s) → 탐사 재초기화 후 재개')
                self.phase = 'PERIMETER'
                self.interrupt = None
                self.left_start = False
                self.start_cell = None
                self.wd_pose = None
                self.phase_start = self.now()

        # 방문 격자 기록
        self.visited.add(self.cell(pose[0], pose[1]))
        if self.start_cell is None:
            self.start_cell = self.cell(pose[0], pose[1])

        # 2) 갇힘 감지(B) → ESCAPE 인터럽트
        #    CAPTURE(정지 dwell 정상) / APPROACH(색 중심 정렬 위해 제자리 회전 가능)는 제외 —
        #    APPROACH 는 자체 approach_timeout 으로 무한루프를 막는다(아래).
        if self.interrupt not in ('CAPTURE', 'APPROACH', 'ALIGN') and self.watchdog_stuck(pose):
            self.get_logger().warn('진행 워치독: 갇힘 감지 → 탈출 기동')
            self.publish_phase('ESCAPE')
            self.switch(interrupt='ESCAPE')

        # ── 인터럽트 처리(색 접근/캡처/탈출이 일반 국면보다 우선) ──
        if self.interrupt == 'ESCAPE':
            # 전방이 열린 쪽으로 틀어 직진해 루프/끼임을 깬다.
            c = Twist()
            if self.sector_min(-25, 25) < self.front_stop:
                left = self.sector_min(20, 70); right = self.sector_min(-70, -20)
                c.angular.z = 0.45 if left > right else -0.45
            else:
                c.linear.x = self.v_fwd
            self.pub.publish(c)
            if self.elapsed(self.phase_start) >= self.escape_secs:
                self.switch(interrupt=None)   # 일반 국면 복귀
            return

        if self.interrupt == 'APPROACH':
            arrived, cmd = self.approach_cmd()
            # 시간초과: 어안렌즈로 중심 정렬이 안 돼 standoff 도달 실패 → 포기하고 벽타기 복귀.
            #   이 자리를 skipped 로 기억해 바로 재트리거되지 않게 한다.
            if not arrived and self.elapsed(self.phase_start) >= self.approach_timeout:
                self.skipped.append(self._target_xy(pose))   # 로봇 위치 아닌 '패널 위치' 저장
                self.get_logger().warn(
                    f'접근 시간초과({self.approach_timeout:.0f}s) — 건너뜀 @({pose[0]:.2f},{pose[1]:.2f})')
                self._start_moveon(pose)
                return
            # 스톨 감지: 윈도 동안 '거의 안 움직였다' = 제자리 회전만 → 빨리 포기(한 지점 빙빙 차단).
            if not arrived and self._appr_stall_t is not None \
                    and self.elapsed(self._appr_stall_t) >= self.approach_stall_win:
                moved = math.hypot(pose[0] - self._appr_pose0[0], pose[1] - self._appr_pose0[1])
                if moved < self.approach_min_move:
                    self.skipped.append(self._target_xy(pose))   # 로봇 위치 아닌 '패널 위치' 저장
                    self.get_logger().warn(
                        f'접근 스톨(이동 {moved:.2f}m<{self.approach_min_move}m/{self.approach_stall_win:.0f}s) '
                        f'— 제자리회전 판정, 건너뜀 @({pose[0]:.2f},{pose[1]:.2f})')
                    self._start_moveon(pose)
                    return
                self._appr_pose0 = pose         # 움직였으면 다음 윈도 기준 갱신
                self._appr_stall_t = self.now()
            # 색이 approach_lost_grace 이상 끊겼을 때만 포기(짧은 깜빡은 무시 → 빙빙 방지).
            if (self._last_color_t is None
                    or self.elapsed(self._last_color_t) > self.approach_lost_grace):
                self.switch(interrupt=None)   # 색 진짜로 놓침 → 일반 국면
                return
            # ★ 충분히 가까워지면(또는 standoff 도달) → '정면 위치로 이동'(ALIGN) 시작.
            #   제자리에서 보는 게 아니라, 라이다 벽 법선으로 패널 바로 앞 수직 포즈를 잡아 그리로 간다.
            near = min(self.sector_min(-15, 15), self.color_range())
            if arrived or near <= self.frontal_start:
                self._frontal_goal = self._compute_frontal_goal(pose)
                self.publish_phase(f'ALIGN {self.color}')
                self.get_logger().info(f'패널 근접({self.color}, ~{near:.2f}m) → 정면 위치로 이동')
                self.switch(interrupt='ALIGN')
                return
            self.pub.publish(cmd)
            return

        if self.interrupt == 'ALIGN':
            # ★ 패널 '바로 앞(수직·정면)' 목표 포즈로 '이동'한다(제자리회전 아님).
            if self._frontal_goal is None:
                self._frontal_goal = self._compute_frontal_goal(pose)
            tx, ty, tyaw = self._frontal_goal
            x, y, yaw = pose
            timed_out = self.elapsed(self.phase_start) >= self.align_secs
            dist = math.hypot(tx - x, ty - y)
            if not timed_out and dist > self.frontal_pos_tol:
                # 정면 목표점으로 주행: 먼저 그쪽을 보고, 정렬되면 전진. 벽에 너무 붙으면 살짝 후진.
                herr = wrap(math.atan2(ty - y, tx - x) - yaw)
                front = self.sector_min(-15, 15)
                c = Twist()
                if abs(herr) > 0.35:
                    c.angular.z = max(-0.4, min(0.4, 0.9 * herr))
                elif front < 0.18:
                    c.linear.x = -0.05
                    c.angular.z = max(-0.3, min(0.3, 0.9 * herr))
                else:
                    c.linear.x = min(0.08, 0.5 * dist)
                    c.angular.z = max(-0.3, min(0.3, 0.9 * herr))
                self.pub.publish(c)
                return
            # 위치 도달(또는 시간초과) → 패널을 정면으로 보도록 최종 회전.
            yerr = wrap(tyaw - yaw)
            if not timed_out and abs(yerr) > self.align_cx_tol:
                c = Twist()
                c.angular.z = max(-0.4, min(0.4, 0.9 * yerr))
                self.pub.publish(c)
                return
            self.pub.publish(Twist())
            self.get_logger().info(
                f'패널 정면 도달(위치오차 {dist:.2f}m{", 시간초과" if timed_out else ""}) → 숫자 읽기')
            self.publish_phase(f'CAPTURE {self.color}')
            # _digit 는 APPROACH 시작 때 리셋됨 → APPROACH/ALIGN/CAPTURE 중 읽힌 숫자를 인정.
            self.switch(interrupt='CAPTURE')
            return

        if self.interrupt == 'CAPTURE':
            self.pub.publish(Twist())          # 정지 dwell — mapper/recognizer 가 기록
            # '확실히 발견' = digit_recognizer 가 신뢰도 임계(80%)를 넘겨 숫자(1·2·3)를 발행.
            #   그 즉시 캡처 완료로 보고 원래 가던 국면으로 복귀. 못 읽으면 상한까지 기다린 뒤 복귀.
            got_digit = self._digit >= 0
            if got_digit or self.elapsed(self.phase_start) >= self.capture_secs:
                self.captured.append(self._target_xy(pose))   # 로봇 위치 아닌 '패널 위치' 저장
                tag = f'숫자 {self._digit} 확인' if got_digit else '숫자 미확인(시간초과)'
                self.get_logger().info(
                    f'캡처 완료({tag}) @({pose[0]:.2f},{pose[1]:.2f}) — 총 {len(self.captured)}개')
                self._start_moveon(pose)       # 그 타겟 벗어날 때까지 전진 후 일반 복귀
            return

        if self.interrupt == 'MOVEON':
            # 방금 캡처/스킵한 타겟에서 moveon_dist/secs 만큼 벗어날 때까지 색 무시하고 전진.
            #   (벗어나기 전엔 APPROACH 트리거 안 됨 → 같은 타겟 즉시 재접근/무한루프 차단.)
            self.pub.publish(self.wall_follow_cmd())
            moved = math.hypot(pose[0] - self._moveon_pose0[0], pose[1] - self._moveon_pose0[1])
            if self.elapsed(self.phase_start) >= self.moveon_secs or moved >= self.moveon_dist:
                self.switch(interrupt=None)    # 충분히 벗어남 → 색 탐색 재개
            return

        # ── 색 발견 → 접근 트리거(중복/방금 캡처/먼 색 제외) ──
        #   2-pass: Phase1 은 색 접근/숫자 dwell 안 함 — 순수 탐사로 맵+색좌표만(color_mapper 가
        #   벽 지나며 색 기록). 정면 방문·숫자는 Phase2(digit_finalizer)가 Nav2 로 처리.
        #   ★ 거리게이트: 색까지 라이다 거리가 approach_max_dist 보다 멀면 무시(그쪽으로 안 감).
        #   ★ 완료색 무시: 그 색을 이미 목표 수(3)만큼 확보했으면 더 접근 안 함.
        if (not self.two_pass and self.color != 'NONE'
                and not self._is_color_complete(self.color)
                and self.color_range() <= self.approach_max_dist
                and not self._recently_captured(pose)):
            self.publish_phase(f'APPROACH {self.color}')
            self._appr_pose0 = pose            # 스톨 감지 기준 위치/시각
            self._appr_stall_t = self.now()
            self._digit = -1                   # 이 패널 시퀀스 동안 '새로' 읽힌 숫자만 인정
            self.switch(interrupt='APPROACH')
            return

        # ── 일반 국면: 벽타기 + loop closure → 중앙 진입 → 섬 ──
        self.run_phase(pose)

    def _target_xy(self, pose):
        """'색이 보이는 방향(cx)'으로 추정한 패널 map 좌표 = 로봇 + d·(yaw+bearing).
        정면이 아니라 색 방위를 쓰므로, 캡처 후 패널을 '옆에 두고' 지나가도 같은 좌표로
        추정 → 이미 잡은 패널로 인식해 재접근 안 함(같은 타겟 무한루프/재무는 것 차단)."""
        x, y, yaw = pose
        bearing = math.radians(-self.color_cx * self.cx_fov_deg)   # cx>0=오른쪽=음의 각
        deg = math.degrees(bearing)
        d = self.sector_min(deg - 12, deg + 12)    # 그 방위의 라이다 거리(벽=가까운 점)
        if not math.isfinite(d):
            d = self.sector_min(-15, 15)
        if not math.isfinite(d):
            d = self.standoff
        d = min(d, 1.2)                            # 너무 먼 추정은 신뢰 X
        ang = yaw + bearing
        return (x + d * math.cos(ang), y + d * math.sin(ang))

    def _wall_normal(self, pose, bearing_deg):
        """패널 방위 주변(±30°) 라이다 점들을 직선적합해 벽 법선(map frame, 벽→로봇 방향) 추정.
        못 구하면 None."""
        s = self.scan
        if s is None:
            return None
        n = len(s.ranges)
        xs, ys = [], []
        d = bearing_deg - 30
        while d <= bearing_deg + 30:
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                xs.append(r * math.cos(math.radians(d)))   # robot frame (x 전방)
                ys.append(r * math.sin(math.radians(d)))
            d += 2
        if len(xs) < 6:
            return None
        mx = sum(xs) / len(xs); my = sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in ys)
        sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(len(xs)))
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)       # 벽 방향(주축) 각
        nx, ny = -math.sin(theta), math.cos(theta)         # 법선(robot frame)
        if nx > 0:                                         # 로봇(-x)쪽 향하게
            nx, ny = -nx, -ny
        yaw = pose[2]
        return (nx * math.cos(yaw) - ny * math.sin(yaw),   # map frame
                nx * math.sin(yaw) + ny * math.cos(yaw))

    def _compute_frontal_goal(self, pose):
        """'패널 바로 앞(standoff, 수직)' 목표 포즈 (tx,ty,tyaw) 계산. tyaw=패널을 바라봄."""
        x, y, yaw = pose
        bearing = math.radians(-self.color_cx * self.cx_fov_deg)   # 색 방위
        pr = self.color_range()
        if not math.isfinite(pr):
            pr = self.standoff
        panel = (x + pr * math.cos(yaw + bearing), y + pr * math.sin(yaw + bearing))
        nrm = self._wall_normal(pose, math.degrees(bearing))
        if nrm is None:                                    # 법선 실패 → 로봇→패널 반대쪽 폴백
            nrm = (-math.cos(yaw + bearing), -math.sin(yaw + bearing))
        tx = panel[0] + self.standoff * nrm[0]
        ty = panel[1] + self.standoff * nrm[1]
        tyaw = math.atan2(panel[1] - ty, panel[0] - tx)    # 패널 바라봄(=벽에 수직)
        return (tx, ty, tyaw)

    def _start_moveon(self, pose):
        """캡처/스킵 직후 호출 — 그 타겟을 벗어날 때까지 색 무시하고 전진하는 MOVEON 진입."""
        self._moveon_pose0 = (pose[0], pose[1])
        self.publish_phase('MOVEON')
        self.switch(interrupt='MOVEON')

    def _recently_captured(self, pose):
        """현재 보고 있는 '패널 위치'가 이미 캡처/스킵한 패널 근처면 True(재접근 방지)."""
        tx, ty = self._target_xy(pose)
        for (cx, cy) in self.captured + self.skipped:
            if math.hypot(tx - cx, ty - cy) < self.dedup_dist:
                return True
        return False

    def run_phase(self, pose):
        x, y, _ = pose
        cur = self.cell(x, y)

        if self.phase == 'PERIMETER':
            # 시작 셀을 충분히 벗어났다가 다시 돌아오면 둘레 한 바퀴 완료로 본다(C).
            d_start = math.hypot(x - (self.start_cell[0] + 0.5) * self.visit_res,
                                 y - (self.start_cell[1] + 0.5) * self.visit_res)
            if not self.left_start and d_start > self.loop_left_dist:
                self.left_start = True
            # 충분히 돌았을 때만(방문셀 ≥ min_visited_for_loop) loop closure 인정 — 작은 원 종료 방지.
            if (self.left_start and d_start < self.loop_close_dist
                    and len(self.visited) >= self.min_visited_for_loop):
                self.inward_target = self.visited_centroid()   # (0,0)=시작점 아님 → 둘레 중심으로
                self.get_logger().info(
                    f'=== 둘레 한 바퀴(loop closure, 방문셀 {len(self.visited)}) → 내부'
                    f'({self.inward_target[0]:+.1f},{self.inward_target[1]:+.1f}) 진입 ===')
                self.publish_phase('INWARD')
                self.switch('INWARD')
                return
            self.pub.publish(self.wall_follow_cmd())

        elif self.phase == 'INWARD':
            # 맵 중앙(0,0)으로 직진하다 섬 벽을 만나면 섬 벽타기로 전환.
            front = self.sector_min(-20, 20)
            if front < self.front_stop:
                self.get_logger().info('=== 중앙에서 섬 벽 접촉 → 섬 벽타기 ===')
                self.publish_phase('ISLAND')
                self.island_start_cell = cur
                self.left_start = False
                self.start_cell = cur          # 섬 loop closure 기준 갱신
                self.switch('ISLAND')
                return
            tx, ty = self.inward_target if self.inward_target else (0.0, 0.0)
            c = Twist()
            herr = wrap(math.atan2(ty - y, tx - x) - pose[2])
            c.angular.z = max(-0.4, min(0.4, 0.9 * herr))
            c.linear.x = self.v_fwd if abs(herr) < 0.5 else 0.05
            # 내부 목표(둘레 무게중심) 근처인데 섬이 없으면 섬 없음 → 종료.
            if math.hypot(x - tx, y - ty) < 0.4:
                self.complete_or_continue('내부 중심 도달했으나 섬 없음 → 탐사 종료')
                return
            self.pub.publish(c)

        elif self.phase == 'ISLAND':
            d_start = math.hypot(x - (self.start_cell[0] + 0.5) * self.visit_res,
                                 y - (self.start_cell[1] + 0.5) * self.visit_res)
            if not self.left_start and d_start > self.loop_left_dist:
                self.left_start = True
            if (self.left_start and d_start < self.loop_close_dist
                    and len(self.visited) >= self.min_visited_for_loop):
                self.complete_or_continue('=== 섬 한 바퀴 완료 → 탐사 종료 ===')
                return
            self.pub.publish(self.wall_follow_cmd())

    # ── 품질 게이트 ──────────────────────────────────────────────────
    def _color_wall_counts(self):
        """color_landmarks.yaml → {색: '색+숫자' 확정 벽 수}. 파일 없음/깨짐은 빈 dict."""
        try:
            with open(self._landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return {}
        return {c: sum(1 for w in resolve_target_walls(data, c) if w.get('digit') is not None)
                for c in VALID_COLORS}

    def _count_quality_walls(self):
        """색+숫자 확정 벽 총수."""
        return sum(self._color_wall_counts().values())

    def _is_color_complete(self, color):
        """그 색이 목표 수(per_color_target)만큼 색+숫자 벽을 이미 확보 → 더 접근/인식 안 함.
        매번 파일 읽지 않게 3s 캐시."""
        if self.per_color_target <= 0:
            return False
        if self._color_counts_t is None or self.elapsed(self._color_counts_t) > 3.0:
            self._color_counts_t = self.now()
            self._color_counts = self._color_wall_counts()
        return self._color_counts.get(color, 0) >= self.per_color_target

    def _finish_phase1(self, reason):
        """2-pass: Phase1(탐사+색좌표) 완료 — /phase1_done 발행 후 '종료'한다.
        품질게이트/재탐사 안 함(숫자는 별도 finalize.launch 의 Phase2 가 정면 방문해 채운다).
        종료 → mapping.launch 가 점유맵 저장 + (two_pass) 스택 Shutdown → finalize 로 핸드오프.
        (color_mapper 는 save_period 마다 색좌표를 디스크에 써두므로 종료 전에 이미 저장돼 있다.)"""
        self._phase1_done = True
        self.publish_phase('PHASE1_DONE')
        self.pub_phase1_done.publish(Bool(data=True))
        self.stop_and_quit(
            f'{reason} → Phase1 완료(방문셀 {len(self.visited)}) → finalize.launch(Phase2) 로 진행')

    def complete_or_continue(self, reason):
        """자연 종료 시점의 품질 게이트. 기준 충족이면 종료, 미달이면 재탐사(상한까지),
        상한 도달이면 경고와 함께 그대로 종료. (시간 상한은 별도 안전망.)"""
        if self.two_pass:                         # 2-pass: 종료 대신 Phase1 완료 신호
            self._finish_phase1(reason)
            return
        if self.min_quality_walls <= 0:
            self.stop_and_quit(reason)            # 게이트 비활성 → 기존 동작
            return
        n = self._count_quality_walls()
        if n >= self.min_quality_walls:
            self.stop_and_quit(reason + f' (품질 OK: 색+숫자 벽 {n}개)')
            return
        if self._resweeps < self.max_resweeps:
            self._resweeps += 1
            self.get_logger().warn(
                f'품질 미달(색+숫자 벽 {n}/{self.min_quality_walls}) → 재탐사 '
                f'{self._resweeps}/{self.max_resweeps}: 놓친 숫자 재수집을 위해 다시 돈다')
            self.publish_phase(f'QUALITY_LOW {n}/{self.min_quality_walls}')
            # 캡처/스킵 기록을 비워 이미 본 패널도 재접근해 digit 을 다시 읽게 한다.
            self.captured.clear()
            self.skipped.clear()
            self.phase = 'PERIMETER'
            self.interrupt = None
            self.left_start = False
            self.start_cell = None
            self.phase_start = self.now()
            return
        self.stop_and_quit(
            reason + f' (품질 미달 {n}/{self.min_quality_walls}, 재탐사 상한 도달 — 그대로 종료)')

    def stop_and_quit(self, msg):
        self.pub.publish(Twist())
        self.get_logger().info(msg + f' (캡처 {len(self.captured)}개, 방문셀 {len(self.visited)})')
        self.publish_phase('DONE')
        # 타이머 콜백 안에서 shutdown 하면 executor 가 교착 → SystemExit 로 빠져나가 main 정리.
        self.timer.cancel()
        raise SystemExit


def _arg(name, default, cast=float):
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            return cast(sys.argv[i + 1])
    return default


def main():
    duration = _arg('--duration', 600.0)
    rclpy.init()
    node = MazeExplorer(duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
