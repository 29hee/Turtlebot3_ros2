#!/usr/bin/env python3
"""
phase2_visitor.py — Phase 2: 저장 맵 + AMCL + Nav2 로 색 후보 정면 방문

Phase 1 에서 만든 maps/color_candidates.yaml 을 읽어:
  1) 같은 색·근접 후보를 클러스터링 (한 벽의 여러 관측 합침)
  2) 각 후보로 Nav2 주행 — Phase 1 관측 yaw 를 이용해 정면에서 접근
  3) 정면 정지 후 coverage(색) + EasyOCR(숫자) 정밀 확인
  4) maps/color_landmarks.yaml 저장 + /phase2_done(Bool) 발행

전제 스택 (phase2_visit.launch.py 가 띄워줌):
  - map_server (Phase 1 에서 저장한 맵)
  - AMCL (로컬라이제이션)
  - Nav2 (navigate_to_pose 액션)
  - vision_node   → /color_signal
  - digit_recognizer → /detected_digit

실행 (단발):
  python3 phase2_visitor.py
  python3 phase2_visitor.py --ros-args -p relocalize:=true  # 실로봇
"""
import math
import os
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, Float32MultiArray, String
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose

from maze_common import VALID_COLORS, color_to_id, MERGE_DIST


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def default_candidates_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_candidates.yaml')


def default_landmarks_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


def cluster_candidates(candidates, merge_dist=MERGE_DIST):
    """같은 색 후보들을 단일연결 병합.
    반환: [{color, x, y, votes, approach_yaw}, ...]"""
    if not candidates:
        return []
    n = len(candidates)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i in range(n):
        for j in range(i + 1, n):
            same_color = candidates[i]['color'] == candidates[j]['color']
            close = math.hypot(candidates[i]['x'] - candidates[j]['x'],
                               candidates[i]['y'] - candidates[j]['y']) <= merge_dist
            if same_color and close:
                parent[find(i)] = find(j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(candidates[i])

    out = []
    for g in groups.values():
        tv = sum(c['votes'] for c in g)
        cx = sum(c['x'] * c['votes'] for c in g) / tv
        cy = sum(c['y'] * c['votes'] for c in g) / tv
        # yaw: 투표 가중 circular mean
        sin_s = sum(math.sin(c['approach_yaw']) * c['votes'] for c in g)
        cos_s = sum(math.cos(c['approach_yaw']) * c['votes'] for c in g)
        avg_yaw = math.atan2(sin_s, cos_s)
        out.append({
            'color': g[0]['color'],
            'x': cx, 'y': cy,
            'votes': tv,
            'approach_yaw': avg_yaw,
        })
    return out


def approach_from_yaw(wx, wy, approach_yaw, standoff):
    """Phase 1 관측 yaw 기반 접근 포즈.

    로봇이 approach_yaw 방향으로 향해 벽을 봤으므로,
    접근점은 벽에서 그 방향 반대쪽 standoff 거리.
    도착 후 로봇은 approach_yaw 방향으로 벽을 정면으로 바라본다.
    """
    ax = wx - math.cos(approach_yaw) * standoff
    ay = wy - math.sin(approach_yaw) * standoff
    return ax, ay, approach_yaw


class Phase2Visitor(Node):
    def __init__(self):
        super().__init__('phase2_visitor')

        self.declare_parameter('candidates_path', default_candidates_path())
        self.declare_parameter('landmarks_path', default_landmarks_path())
        self.declare_parameter('standoff', 0.35)
        self.declare_parameter('confirm_window', 5.0)    # 도착 후 색 확인 관측 시간 [s]
        self.declare_parameter('confirm_coverage', 0.12) # 이 점유율 이상이면 색 OK
        self.declare_parameter('confirm_min_true', 3)    # 이 횟수 이상 OK 면 확인
        self.declare_parameter('digit_window', 4.0)      # 추가 숫자 대기 시간 [s]
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # 실로봇 전용: 시작 시 AMCL 전역 재초기화 + 수렴 회전
        self.declare_parameter('relocalize', False)
        self.declare_parameter('relocalize_speed', 0.4)
        self.declare_parameter('relocalize_max_turns', 3.0)
        self.declare_parameter('relocalize_pos_std', 0.25)
        self.declare_parameter('relocalize_yaw_std', 0.35)

        self.candidates_path = self.get_parameter('candidates_path').value
        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.confirm_window = float(self.get_parameter('confirm_window').value)
        self.confirm_coverage = float(self.get_parameter('confirm_coverage').value)
        self.confirm_min_true = int(self.get_parameter('confirm_min_true').value)
        self.digit_window = float(self.get_parameter('digit_window').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.relocalize = bool(self.get_parameter('relocalize').value)
        self.relocalize_speed = float(self.get_parameter('relocalize_speed').value)
        self.relocalize_max_turns = float(self.get_parameter('relocalize_max_turns').value)
        self.relocalize_pos_std = float(self.get_parameter('relocalize_pos_std').value)
        self.relocalize_yaw_std = float(self.get_parameter('relocalize_yaw_std').value)

        # ── 상태 ──────────────────────────────────────────────────
        self._color_signal = [0.0, 0.0, 0.0]   # [color_id, cx_norm, coverage]
        self._detected_digit = -1
        self._amcl_cov = None
        self._scan = None

        # ── IO ────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.global_loc = self.create_client(Empty, '/reinitialize_global_localization')
        # color_confirm 에게 현재 target 색을 알리는 토픽 (color_confirm.py 가 구독)
        self.pub_target = self.create_publisher(String, '/target_color', 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_done = self.create_publisher(Bool, '/phase2_done', latched)

        self.create_subscription(Float32MultiArray, '/color_signal', self._on_signal, 10)
        self.create_subscription(Int32, '/detected_digit', self._on_digit, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'phase2_visitor 시작 — standoff={self.standoff}m, '
            f'confirm_window={self.confirm_window}s')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _on_signal(self, msg):
        if len(msg.data) >= 3:
            self._color_signal = list(msg.data[:3])

    def _on_digit(self, msg):
        self._detected_digit = int(msg.data)

    def _on_amcl(self, msg):
        self._amcl_cov = msg.pose.covariance

    def _on_scan(self, msg):
        self._scan = msg

    # ── AMCL 수렴 판정 ─────────────────────────────────────────────
    def _amcl_converged(self):
        c = self._amcl_cov
        if c is None:
            return False
        pos_std = max(math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)))
        yaw_std = math.sqrt(max(c[35], 0.0))
        return pos_std <= self.relocalize_pos_std and yaw_std <= self.relocalize_yaw_std

    def relocalize_in_place(self):
        """실로봇 시작용: AMCL 전역 재초기화 → 수렴할 때까지 제자리 회전."""
        if self.global_loc.wait_for_service(timeout_sec=3.0):
            self.global_loc.call_async(Empty.Request())
            self._amcl_cov = None
            self.get_logger().info('AMCL 전역 재초기화 (파티클 분산)')
        else:
            self.get_logger().warn('전역 재초기화 서비스 없음 — 회전만으로 수렴 시도')
        max_dur = self.relocalize_max_turns * 2 * math.pi / max(0.05, self.relocalize_speed)
        twist = Twist()
        twist.angular.z = self.relocalize_speed
        end = time.time() + max_dur
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._amcl_converged():
                break
        self.cmd_pub.publish(Twist())
        self.get_logger().info('자기위치 추정 완료 → 방문 시작')

    # ── 데이터 로드 ────────────────────────────────────────────────
    def load_candidates(self):
        try:
            with open(self.candidates_path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().error(
                f'color_candidates.yaml 없음: {self.candidates_path}\n'
                f'Phase 1 을 먼저 돌릴 것 (phase1_mapping.launch.py)')
            return []
        except Exception as e:
            self.get_logger().error(f'candidates 읽기 실패: {e}')
            return []
        raw = data.get('candidates', [])
        valid = [c for c in raw if c.get('color') in VALID_COLORS]
        self.get_logger().info(f'후보 로드: {len(valid)}개 ({self.candidates_path})')
        return valid

    def get_pose_full(self, timeout=3.0):
        """(x, y, yaw) 반환. TF 못 받으면 None."""
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                return t.x, t.y, yaw
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    # ── LiDAR 벽 법선 보정 ────────────────────────────────────────
    def find_wall_normal(self, pose):
        """정면 ±60° LiDAR 스캔에서 최소 거리 각도 = 벽에 수직 방향.
        반환: (wall_normal_yaw, wall_dist) 또는 None.

        평평한 벽일 때 LiDAR 호(arc)의 최소 거리 지점이 수직 방향.
        로봇이 사선에 있어도 항상 올바른 법선을 찾는다.
        """
        s = self._scan
        if s is None or len(s.ranges) == 0:
            return None
        _, _, robot_yaw = pose
        n = len(s.ranges)

        min_r, min_local_rad = float('inf'), 0.0
        for deg in range(-60, 61):
            local_rad = math.radians(deg)
            idx = int(round((local_rad - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if math.isfinite(r) and s.range_min < r < s.range_max and r < min_r:
                min_r = r
                min_local_rad = local_rad

        if not math.isfinite(min_r):
            return None

        wall_normal_yaw = robot_yaw + min_local_rad
        return wall_normal_yaw, min_r

    def refine_to_wall_normal(self, label):
        """러프 접근 후 LiDAR 벽 법선으로 정확한 정면 위치에 재이동.

        흐름:
          현재 pose → find_wall_normal → 벽 위치 계산
          → standoff 거리 정면 접근 포즈 → Nav2 재이동
        반환: True=성공, False=스캔 없음/재이동 실패(기존 위치 그대로 진행)
        """
        pose = self.get_pose_full()
        if pose is None:
            self.get_logger().warn(f'[{label}] 법선 보정: pose 없음 → 스킵')
            return False

        result = self.find_wall_normal(pose)
        if result is None:
            self.get_logger().warn(f'[{label}] 법선 보정: 스캔 없음 → 스킵')
            return False

        wall_normal_yaw, wall_dist = result
        x, y, _ = pose
        wx = x + math.cos(wall_normal_yaw) * wall_dist
        wy = y + math.sin(wall_normal_yaw) * wall_dist
        ax = wx - math.cos(wall_normal_yaw) * self.standoff
        ay = wy - math.sin(wall_normal_yaw) * self.standoff

        self.get_logger().info(
            f'[{label}] 벽 법선 보정: 거리={wall_dist:.2f}m, '
            f'법선={math.degrees(wall_normal_yaw):.0f}° → 재이동')
        return self.nav_to(ax, ay, wall_normal_yaw, f'{label} 법선')

    def get_robot_xy(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(yaw)
        return p

    def nav_to(self, x, y, yaw, label):
        """navigate_to_pose 동기 호출. 성공 여부 반환."""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 서버 없음 (Nav2 미실행?)')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        self.get_logger().info(
            f'[{label}] 주행 → ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°)')
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[{label}] 목표 거부됨')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ok = result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'[{label}] {"도착" if ok else "실패"}')
        return ok

    # ── 정면 정렬 ──────────────────────────────────────────────────
    def align_to_panel(self, expected_color, timeout=6.0, cx_thresh=0.15):
        """Nav2 도착 후 색 blob을 화면 중앙에 맞춰 제자리 회전 정렬.

        /color_signal 의 cx_norm 이 [-cx_thresh, +cx_thresh] 안에 들어오면 정면 완료.
        색이 안 보이면 천천히 회전하며 탐색.
        반환: True=정면 정렬 성공, False=timeout
        """
        expected_id = color_to_id(expected_color)
        end = time.time() + timeout

        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            sig = self._color_signal
            detected_id = int(sig[0]) if sig else 0
            cx = sig[1] if len(sig) >= 2 else 0.0
            cov = sig[2] if len(sig) >= 3 else 0.0

            cmd = Twist()
            if detected_id != expected_id or cov < self.confirm_coverage:
                # 색이 안 보임 → 천천히 좌회전으로 탐색
                cmd.angular.z = 0.20
                self.cmd_pub.publish(cmd)
                continue

            if abs(cx) <= cx_thresh:
                # 정면 완료
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'정면 정렬 완료 (cx={cx:+.2f}, cov={cov:.2f})')
                return True

            # cx 방향으로 정렬: cx>0(오른쪽)이면 우회전(angular.z 음수)
            cmd.angular.z = max(-0.35, min(0.35, -0.5 * cx))
            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())
        self.get_logger().warn(f'정면 정렬 timeout ({timeout:.0f}s)')
        return False

    # ── 색+숫자 관측 ───────────────────────────────────────────────
    def confirm_at_wall(self, expected_color):
        """정면 정렬 후 정지 상태에서 색+숫자를 관측.
        반환: (color_confirmed: bool, digit: int)  digit=-1이면 미인식.

        1) align_to_panel: cx_norm 기반 정면 정렬
        2) 색 확인: color_id == expected AND coverage >= 임계
        3) 숫자 확인: /detected_digit 최다 득표
        """
        self.pub_target.publish(String(data=expected_color))

        # Step 1: 정면 정렬
        aligned = self.align_to_panel(expected_color)
        if not aligned:
            self.get_logger().warn('정면 정렬 실패 — 현재 위치에서 관측 시도')

        expected_id = color_to_id(expected_color)
        true_count = 0
        digit_readings = []
        end = time.time() + self.confirm_window

        # Step 2+3: 색 coverage 확인 + 숫자 수집
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            sig = self._color_signal
            color_match = (len(sig) >= 1 and int(sig[0]) == expected_id)
            coverage = sig[2] if len(sig) >= 3 else 0.0
            if color_match and coverage >= self.confirm_coverage:
                true_count += 1
            d = self._detected_digit
            if d >= 0:
                digit_readings.append(d)

        color_ok = true_count >= self.confirm_min_true

        # 숫자가 아직 없으면 digit_window 만큼 더 기다린다
        if not digit_readings:
            end2 = time.time() + self.digit_window
            while time.time() < end2 and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                d = self._detected_digit
                if d >= 0:
                    digit_readings.append(d)
                    if len(digit_readings) >= 5:
                        break

        digit = Counter(digit_readings).most_common(1)[0][0] if digit_readings else -1
        return color_ok, digit

    # ── 수집 완료 확인 ─────────────────────────────────────────────
    def check_completeness(self, results):
        """결과에서 색별 digit 수집 완료 여부 확인.
        반환: (all_done: bool, report: str)
        """
        collected = {}  # color -> [digit, ...]
        for color, _x, _y, _v, digit in results:
            collected.setdefault(color, []).append(digit)

        ok_list, missing_list = [], []
        for color in VALID_COLORS:
            if color not in collected:
                missing_list.append(f'{color}(미방문)')
            elif all(d < 0 for d in collected[color]):
                missing_list.append(f'{color}(숫자미인식)')
            else:
                digits = [d for d in collected[color] if d >= 0]
                ok_list.append(f'{color}={digits}')

        report = '완료: ' + ', '.join(ok_list) if ok_list else '완료 없음'
        if missing_list:
            report += '  /  미완료: ' + ', '.join(missing_list)
        return len(missing_list) == 0, report

    # ── 저장 ──────────────────────────────────────────────────────
    def save_landmarks(self, results):
        """results: [(color, x, y, votes, digit), ...]"""
        data = {c: [] for c in VALID_COLORS}
        for color, x, y, votes, digit in results:
            entry = {'x': round(x, 3), 'y': round(y, 3), 'votes': votes}
            if digit >= 0:
                entry['digit'] = digit
            data[color].append(entry)
        save_dir = os.path.dirname(self.landmarks_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        try:
            with open(self.landmarks_path, 'w') as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
            self.get_logger().info(
                f'color_landmarks.yaml 저장: {len(results)}개 벽 → {self.landmarks_path}')
        except Exception as e:
            self.get_logger().error(f'저장 실패: {e}')

    # ── 메인 ──────────────────────────────────────────────────────
    def run(self):
        if self.relocalize:
            self.relocalize_in_place()

        raw = self.load_candidates()
        if not raw:
            self.pub_done.publish(Bool(data=False))
            return False

        walls = cluster_candidates(raw)
        self.get_logger().info(f'후보 {len(raw)}개 → 클러스터 {len(walls)}개')

        rxy = self.get_robot_xy()
        if rxy is None:
            self.get_logger().error('로봇 위치(TF map→base_link) 못 받음. AMCL 확인.')
            self.pub_done.publish(Bool(data=False))
            return False

        # nearest-neighbor 방문 순서 (이동 효율)
        remaining = list(walls)
        ordered = []
        cx, cy = rxy
        while remaining:
            nxt = min(remaining, key=lambda w: math.hypot(w['x'] - cx, w['y'] - cy))
            ordered.append(nxt)
            remaining.remove(nxt)
            cx, cy = nxt['x'], nxt['y']

        results = []
        for i, w in enumerate(ordered):
            color = w['color']
            ax, ay, yaw = approach_from_yaw(w['x'], w['y'], w['approach_yaw'], self.standoff)
            label = f"{color} {i + 1}/{len(ordered)} ({w['x']:.2f},{w['y']:.2f})"

            if not self.nav_to(ax, ay, yaw, label):
                self.get_logger().warn(f'{label} 접근 실패 — 건너뜀')
                continue

            # 러프 접근 후 LiDAR로 벽 법선 계산 → 정확한 정면 위치로 재이동
            # 실패해도 현재 위치에서 그냥 진행 (confirm_at_wall의 align이 보완)
            self.refine_to_wall_normal(label)

            color_ok, digit = self.confirm_at_wall(color)
            digit_str = str(digit) if digit >= 0 else '미인식'

            if color_ok:
                results.append((color, w['x'], w['y'], w['votes'], digit))
                self.get_logger().info(f'{label} ✓ 색 확인, 숫자={digit_str}')
            else:
                self.get_logger().warn(f'{label} ✗ 색 확인 실패 (coverage 부족)')

        self.save_landmarks(results)

        all_done, report = self.check_completeness(results)
        level = self.get_logger().info if all_done else self.get_logger().warn
        level(f'수집 결과 — {report}')

        self.pub_done.publish(Bool(data=all_done))
        self.get_logger().info(
            f'=== Phase 2 완료: {len(results)}/{len(ordered)}개 확인, '
            f'{"전체 수집 완료" if all_done else "일부 미완료"} → /phase2_done({all_done}) ===')
        return all_done


def main(args=None):
    rclpy.init(args=args)
    node = Phase2Visitor()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
