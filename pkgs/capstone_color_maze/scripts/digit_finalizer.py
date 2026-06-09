#!/usr/bin/env python3
"""
digit_finalizer.py — 2-pass 매핑의 Phase 2(숫자 확정).

[흐름]  Phase1(maze_explorer two_pass): 탐사로 SLAM 맵 + '색 좌표만' 저장(color_mapper
  require_digit=false) → 완료 시 /phase1_done 발행.
  → 본 노드가 그걸 받아, color_landmarks.yaml 의 색 벽마다:
     ① Nav2(navigate_to_pose)로 '정면 접근 포즈'(벽에서 standoff, 벽을 바라봄)까지 주행
     ② 도착 후 라이다로 '벽에 완전 수직' 미세정렬(대각이 아닌 정면) ← 옆에서 비스듬히
        읽어 숫자를 자주 놓치던 문제를 여기서 제거
     ③ dwell 동안 정지 — digit_recognizer 가 정면에서 숫자를 읽고 color_mapper 가
        그 칸에 색+숫자로 확정 투표(=색맵 확정)
  모든 벽 방문 후 점유격자맵을 저장(맵 확정).

전제: slam_toolbox(/map·map→odom) + Nav2 navigation(navigate_to_pose) + vision_node +
  digit_recognizer + color_mapper(require_digit=false) 가 함께 떠 있음(mapping.launch 가 띄움).
실행: python3 digit_finalizer.py --ros-args -p use_sim_time:=false
"""
import math
import os
import subprocess
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

import yaml
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, Float32MultiArray
from nav2_msgs.action import NavigateToPose

from maze_common import (
    VALID_COLORS, resolve_target_walls, approach_pose, id_to_color,
)

KOR = {'RED': '빨강', 'GREEN': '초록', 'BLUE': '파랑'}


def default_landmarks_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


def default_map_save():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_room')


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class DigitFinalizer(Node):
    def __init__(self):
        super().__init__('digit_finalizer')

        self.declare_parameter('landmarks_path', default_landmarks_path())
        self.declare_parameter('map_save', default_map_save())
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff', 0.45)       # 벽 앞 정지 거리 [m]
        self.declare_parameter('dwell_secs', 4.0)      # 정면 dwell(숫자 읽기) [s]
        self.declare_parameter('align_tol_deg', 6.0)   # 수직정렬 허용오차 [deg]
        self.declare_parameter('align_secs', 6.0)      # 수직정렬 최대 시간 [s]
        self.declare_parameter('save_map', True)       # 끝나면 점유맵 저장
        self.declare_parameter('nav_timeout', 120.0)   # Nav2 결과 대기 상한[s] (무한대기 방지)
        # ★ Q1: 도착 후 '기대 색이 충분히/중앙에 보이는지' 확인(맞는 판 보고 있는지).
        self.declare_parameter('cx_fov_deg', 60.0)     # cx → 방위각 환산
        self.declare_parameter('confirm_cov', 0.04)    # 기대 색 점유율 ≥ 이 값이면 '그 판 맞음'
        self.declare_parameter('face_tol', 0.12)       # 색 중앙 허용오차(|cx|)
        self.declare_parameter('perp_tol_deg', 12.0)   # 수직 허용오차[deg] — 이 이하라야 센터보정 신뢰
        # ★ Q2: OCR 거리 — 못 읽으면 이 거리들을 차례로 시도(정면거리 맞춰가며).
        self.declare_parameter('ocr_dists', [0.45, 0.35, 0.55])
        # finalize.launch 단독 실행(=Phase1 과 별도 프로세스)이면 /phase1_done 을 기다리지 않는다.
        #   같은 런치에서 Phase1 과 함께 돌던 구방식 호환을 위해 기본 True.
        self.declare_parameter('wait_phase1', True)
        # 실로봇: 시작 시 제자리 회전으로 AMCL 전역추정 수렴(시작위치를 모르므로). 시뮬은 불필요.
        self.declare_parameter('relocalize', False)
        self.declare_parameter('reloc_secs', 8.0)      # relocalize 회전 시간 [s]
        self.declare_parameter('reloc_spin', 0.5)      # relocalize 회전 각속도 [rad/s]

        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.map_save = self.get_parameter('map_save').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.dwell_secs = float(self.get_parameter('dwell_secs').value)
        self.align_tol = math.radians(float(self.get_parameter('align_tol_deg').value))
        self.align_secs = float(self.get_parameter('align_secs').value)
        self.save_map = bool(self.get_parameter('save_map').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.cx_fov_deg = float(self.get_parameter('cx_fov_deg').value)
        self.confirm_cov = float(self.get_parameter('confirm_cov').value)
        self.face_tol = float(self.get_parameter('face_tol').value)
        self.perp_tol = math.radians(float(self.get_parameter('perp_tol_deg').value))
        self.ocr_dists = list(self.get_parameter('ocr_dists').value)
        self.wait_phase1 = bool(self.get_parameter('wait_phase1').value)
        self.relocalize = bool(self.get_parameter('relocalize').value)
        self.reloc_secs = float(self.get_parameter('reloc_secs').value)
        self.reloc_spin = float(self.get_parameter('reloc_spin').value)

        self.scan = None
        self._digit = -1
        self._phase1_done = False
        self._sig_color = 'NONE'   # /color_signal 최신 우세색
        self._sig_cx = 0.0
        self._sig_cov = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Int32, '/detected_digit', self._on_digit, 10)
        self.create_subscription(Float32MultiArray, '/color_signal', self._on_signal, 10)
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/phase1_done', self._on_phase1, _latched)
        self.pub_done = self.create_publisher(Bool, '/phase2_done', _latched)

        self.get_logger().info('digit_finalizer 대기 — /phase1_done 받으면 정면 방문·숫자 확정 시작')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _on_scan(self, msg):
        self.scan = msg

    def _on_signal(self, msg):
        d = msg.data
        if len(d) >= 3:
            self._sig_color = id_to_color(int(d[0]))
            self._sig_cx = float(d[1])
            self._sig_cov = float(d[2])

    def _on_digit(self, msg):
        self._digit = int(msg.data)

    def _on_phase1(self, msg):
        if msg.data and not self._phase1_done:
            self._phase1_done = True

    # ── 유틸 ──────────────────────────────────────────────────────
    def robot_xy(self, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def load_walls(self):
        """color_landmarks.yaml(색 좌표) → 색별 '벽' 목록을 합쳐 [(color,x,y,id), ...]."""
        try:
            with open(self.landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'색맵 로드 실패: {e}')
            return []
        walls = []
        for color in VALID_COLORS:
            for w in resolve_target_walls(data, color):
                walls.append({'color': color, 'x': w['x'], 'y': w['y'], 'id': w['id'],
                              'nx': w.get('nx'), 'ny': w.get('ny')})
        return walls

    def frontal_pose(self, w):
        """정면 접근 포즈. 시점방향(nx,ny)이 있으면 '본 면 쪽'에서 접근(중앙 박스 대응),
        없으면 중심(0,0) 쪽 폴백."""
        nx, ny = w.get('nx'), w.get('ny')
        if nx is not None and ny is not None and (abs(nx) + abs(ny)) > 1e-3:
            ax, ay = w['x'] + self.standoff * nx, w['y'] + self.standoff * ny
            yaw = math.atan2(w['y'] - ay, w['x'] - ax)   # 벽을 바라봄
            return ax, ay, yaw
        return approach_pose(w['x'], w['y'], self.standoff)

    def order_nn(self, walls, start):
        """현재 위치에서 nearest-neighbor 방문 순서."""
        remaining = list(walls)
        out = []
        cx, cy = start
        while remaining:
            nxt = min(remaining, key=lambda w: math.hypot(w['x'] - cx, w['y'] - cy))
            out.append(nxt)
            remaining.remove(nxt)
            cx, cy = nxt['x'], nxt['y']
        return out

    def nav_to(self, x, y, yaw, label):
        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 서버 없음(Nav2 미실행?)')
            return False
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(yaw)
        goal.pose = p
        self.get_logger().info(f'[{label}] 정면 접근 주행 → ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°)')
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[{label}] 목표 거부/응답없음')
            return False
        rfut = handle.get_result_async()
        # ★ 타임아웃 — Nav2 가 결과를 안 주면 무한 대기(행) 방지: 취소하고 실패 처리.
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=self.nav_timeout)
        res = rfut.result()
        if res is None:
            self.get_logger().error(f'[{label}] Nav2 결과 {self.nav_timeout:.0f}s 타임아웃 — 취소·건너뜀')
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            return False
        ok = res.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'[{label}] {"도착" if ok else "주행 실패"}')
        return ok

    def _nearest_front_angle(self):
        """전방 ±90° 에서 가장 가까운 점(=벽)의 방위각[rad]. 없으면 None."""
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

    def align_frontal(self):
        """라이다로 '가장 가까운 벽 점'을 정면(0°)에 오게 회전 → 벽에 완전 수직(정면)."""
        end = time.time() + self.align_secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            ang = self._nearest_front_angle()
            if ang is None:
                self.cmd_pub.publish(Twist())
                continue
            if abs(ang) <= self.align_tol:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'  정면 정렬 완료(오차 {math.degrees(ang):+.1f}°)')
                return True
            t = Twist()
            t.angular.z = max(-0.4, min(0.4, 1.2 * ang))   # 벽쪽으로 회전
            self.cmd_pub.publish(t)
        self.cmd_pub.publish(Twist())
        self.get_logger().warn('  정면 정렬 시간초과 — 그대로 dwell')
        return False

    def relocalize_spin(self):
        """시작 시 제자리 회전으로 AMCL 전역 추정을 수렴시킨다(실로봇: 시작위치 모름).
        동결맵+AMCL 조합이라 한 바퀴 돌면 스캔매칭으로 자기위치가 잡힌다."""
        self.get_logger().info(f'AMCL 수렴용 제자리 회전 {self.reloc_secs:.0f}s …')
        end = time.time() + self.reloc_secs
        t = Twist()
        t.angular.z = self.reloc_spin
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())

    def persist_digit(self, w, digit, obs=None):
        """확정한 숫자를 landmarks YAML 의 해당 색·좌표 엔트리에 '직접' 병합 기록.
        obs(=관측된 패널 위치 x,y,nx,ny)가 주어지면(수직 관측) 좌표도 센터 보정한다."""
        try:
            with open(self.landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f'  landmarks 로드 실패(숫자 기록 보류): {e}')
            return
        best, bestd = None, float('inf')
        for e in data.get(w['color'], []):
            dd = math.hypot(e.get('x', 1e9) - w['x'], e.get('y', 1e9) - w['y'])
            if dd < bestd:
                best, bestd = e, dd
        if best is None or bestd > 0.5:
            self.get_logger().warn(
                f'  landmarks 에서 {w["color"]} ({w["x"]:.2f},{w["y"]:.2f}) 매칭 실패 — 숫자 기록 보류')
            return
        best['digit'] = int(digit)
        # ★ 수직 관측이면 좌표·법선을 그 관측으로 센터 보정(맵에 반영). 매칭 벽 근처일 때만.
        if obs is not None and math.hypot(obs[0] - best['x'], obs[1] - best['y']) <= 0.5:
            best['x'], best['y'] = round(obs[0], 3), round(obs[1], 3)
            best['nx'], best['ny'] = round(obs[2], 3), round(obs[3], 3)
            self.get_logger().info(f'  좌표 센터 보정 → ({best["x"]:.2f},{best["y"]:.2f})')
        try:
            with open(self.landmarks_path, 'w') as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        except Exception as e:
            self.get_logger().warn(f'  landmarks 저장 실패: {e}')

    def front_range(self):
        """전방 ±8° 라이다 거리(중앙값). 없으면 inf."""
        s = self.scan
        if s is None:
            return float('inf')
        n = len(s.ranges)
        vals = []
        a = -8
        while a <= 8:
            idx = int(round((math.radians(a) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                vals.append(r)
            a += 1
        return sorted(vals)[len(vals) // 2] if vals else float('inf')

    def _range_at(self, bearing_rad):
        """라이다 bearing_rad 방향(±4°) 거리(중앙값). 없으면 None."""
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return None
        n = len(s.ranges)
        win = max(1, int(math.radians(4) / s.angle_increment))
        i0 = int(round((bearing_rad - s.angle_min) / s.angle_increment)) % n
        vals = []
        for k in range(-win, win + 1):
            r = s.ranges[(i0 + k) % n]
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                vals.append(r)
        return sorted(vals)[len(vals) // 2] if vals else None

    def observed_panel(self):
        """★ '지금 보이는 색의 진짜 위치(map)+법선' = 로봇 + cx방위 + 그 방향 라이다.
        숫자를 읽은 순간의 cx 로 패널 중심을 센터 보정한다. 실패 시 None."""
        if self._sig_color == 'NONE' or self._sig_cov < self.confirm_cov:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        bearing = math.radians(-self._sig_cx * self.cx_fov_deg)
        rng = self._range_at(bearing)
        if rng is None:
            return None
        ang = yaw + bearing
        px, py = t.x + rng * math.cos(ang), t.y + rng * math.sin(ang)
        return (px, py, -math.cos(ang), -math.sin(ang))   # x,y, nx,ny(벽→로봇)

    def wall_skew(self):
        """★ 전방 벽을 직선적합 → 벽 법선과 로봇 정면 사이 각[rad]. 0=완전 수직.
        cx 가 중앙이어도 이 값이 크면 '비스듬히 본 것'. 못 구하면 None."""
        s = self.scan
        if s is None:
            return None
        n = len(s.ranges)
        xs, ys = [], []
        a = -30
        while a <= 30:
            idx = int(round((math.radians(a) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                xs.append(r * math.cos(math.radians(a)))
                ys.append(r * math.sin(math.radians(a)))
            a += 2
        if len(xs) < 6:
            return None
        mx = sum(xs) / len(xs); my = sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y - my) ** 2 for y in ys)
        sxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(len(xs)))
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)   # 벽 방향(robot frame)
        nx, ny = -math.sin(theta), math.cos(theta)     # 법선
        if nx > 0:
            nx, ny = -nx, -ny                          # 로봇쪽(-x)
        return abs(math.atan2(ny, -nx))                # (-1,0)=완전수직 기준 각

    def face_color(self, color):
        """[Q1] 기대 색을 화면 중앙에 오게 회전 → 그 패널을 정면으로 본다(=맞는 판 확인).
        align_secs 안에 기대 색이 중앙(±face_tol)+충분점유(confirm_cov)면 True. 아예 못 보면 False."""
        end = time.time() + self.align_secs
        seen = False
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._sig_color == color and self._sig_cov >= self.confirm_cov:
                seen = True
                if abs(self._sig_cx) <= self.face_tol:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f'  {color} 정면 확인(cx={self._sig_cx:+.2f}, cov={self._sig_cov:.2f})')
                    return True
                t = Twist(); t.angular.z = max(-0.4, min(0.4, -0.8 * self._sig_cx))
                self.cmd_pub.publish(t)
            else:
                t = Twist(); t.angular.z = 0.25     # 기대 색 안 보임 → 천천히 좌우 탐색
                self.cmd_pub.publish(t)
        self.cmd_pub.publish(Twist())
        return seen

    def set_distance(self, target, secs=3.0):
        """[Q2] 정면 라이다 거리를 target[m]에 맞춘다(전/후진) — OCR 적정 거리 조정."""
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            err = self.front_range() - target
            if abs(err) <= 0.04:
                self.cmd_pub.publish(Twist()); return
            t = Twist(); t.linear.x = max(-0.06, min(0.06, 0.5 * err))
            self.cmd_pub.publish(t)
        self.cmd_pub.publish(Twist())

    def _read_once(self, secs):
        """secs 동안 정지하며 digit 다수결. -1=못읽음."""
        self._digit = -1
        seen = []
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._digit >= 0:
                seen.append(self._digit)
        return Counter(seen).most_common(1)[0][0] if seen else -1

    def dwell_read(self, color):
        """[Q2] OCR 적정 거리를 차례로 시도하며 숫자를 읽는다(못 읽으면 거리 바꿔 재시도).
        각 거리에서 기대 색을 다시 중앙에 두고(face) dwell. 성공한 숫자 반환, 다 실패면 -1."""
        for dist in self.ocr_dists:
            self.set_distance(dist)
            self.face_color(color)              # 거리 바꾼 뒤 다시 정면 중앙
            d = self._read_once(self.dwell_secs)
            if d >= 0:
                self.get_logger().info(f'  OCR 성공 @ ~{dist:.2f}m → 숫자 {d}')
                return d
            self.get_logger().info(f'  OCR 실패 @ ~{dist:.2f}m — 거리 바꿔 재시도')
        return -1

    # ── 메인 ──────────────────────────────────────────────────────
    def run(self):
        # 0) 실로봇: AMCL 수렴(시작위치 추정)부터.
        if self.relocalize:
            self.relocalize_spin()
        # 1) Phase1 핸드오프 — 단독 실행(finalize.launch)이면 대기 없이 바로 시작.
        if self.wait_phase1:
            self.get_logger().info('Phase1(탐사+색좌표) 완료 대기…')
            while rclpy.ok() and not self._phase1_done:
                rclpy.spin_once(self, timeout_sec=0.2)
            self.get_logger().info('=== /phase1_done 수신 → Phase2(정면 방문·숫자 확정) 시작 ===')
        else:
            self.get_logger().info('=== finalize 단독 실행 → Phase2(정면 방문·숫자 확정) 시작 ===')

        walls = self.load_walls()
        if not walls:
            self.get_logger().error('색 벽이 없음 — Phase1 색맵 비었나? 확정 중단.')
            return
        start = self.robot_xy() or (0.0, 0.0)
        order = self.order_nn(walls, start)
        self.get_logger().info(
            '방문 순서: ' + ' → '.join(f'{KOR.get(w["color"],w["color"])}#{w["id"]}' for w in order))

        done = []
        for w in order:
            kor = KOR.get(w['color'], w['color'])
            label = f'{kor}#{w["id"]} ({w["x"]:.2f},{w["y"]:.2f})'
            ax, ay, yaw = self.frontal_pose(w)
            if not self.nav_to(ax, ay, yaw, label):
                self.get_logger().warn(f'{label} 접근 실패 — 건너뜀')
                continue
            self.align_frontal()                       # 라이다로 벽에 거친 수직정렬
            # ★ Q1: 기대 색을 정면 중앙에 두고 '맞는 판 보고 있는지' 확인. 못 보면 엉뚱한 위치.
            if not self.face_color(w['color']):
                self.get_logger().warn(f'{label} → {kor} 색을 못 봄(좌표 어긋남/놓침) — 건너뜀')
                continue
            # ★ Q2: OCR 적정 거리 차례로 시도하며 숫자 읽기.
            d = self.dwell_read(w['color'])
            if d >= 0:
                # ★ 수직도 측정 — cx 중앙이어도 비스듬히 봤으면 좌표 센터보정은 신뢰 안 함.
                skew = self.wall_skew()
                perp = skew is not None and skew <= self.perp_tol
                obs = self.observed_panel() if perp else None
                tag = (f'수직 {math.degrees(skew):.0f}°→센터보정' if perp
                       else (f'비스듬 {math.degrees(skew):.0f}°→좌표유지' if skew is not None
                             else '수직측정실패→좌표유지'))
                self.get_logger().info(f'{label} → 숫자 {d} 확정(정면, {tag})')
                self.persist_digit(w, d, obs)          # 수직일 때만 좌표 센터보정
                done.append((w['color'], w['id'], d))
            else:
                self.get_logger().warn(f'{label} → 숫자 못 읽음(거리 조정해도) — 보류')

        # 2) 맵 확정: 점유격자맵 저장
        if self.save_map:
            try:
                self.get_logger().info(f'점유격자맵 저장 → {self.map_save}')
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_save],
                    timeout=30, check=False)
            except Exception as e:
                self.get_logger().warn(f'맵 저장 실패(수동 저장 필요): {e}')

        self.pub_done.publish(Bool(data=True))
        self.get_logger().info(
            f'=== Phase2 완료 — 정면 확정 {len(done)}/{len(order)}개. 맵 확정. /phase2_done ===')


def main(args=None):
    rclpy.init(args=args)
    node = DigitFinalizer()
    try:
        node.run()
        rclpy.spin(node)        # 끝나도 /phase2_done latched 유지
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
