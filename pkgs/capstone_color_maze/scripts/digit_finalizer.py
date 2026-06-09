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
from std_msgs.msg import Bool, Int32
from nav2_msgs.action import NavigateToPose

from maze_common import (
    VALID_COLORS, resolve_target_walls, approach_pose,
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

        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.map_save = self.get_parameter('map_save').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.dwell_secs = float(self.get_parameter('dwell_secs').value)
        self.align_tol = math.radians(float(self.get_parameter('align_tol_deg').value))
        self.align_secs = float(self.get_parameter('align_secs').value)
        self.save_map = bool(self.get_parameter('save_map').value)

        self.scan = None
        self._digit = -1
        self._phase1_done = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Int32, '/detected_digit', self._on_digit, 10)
        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/phase1_done', self._on_phase1, _latched)
        self.pub_done = self.create_publisher(Bool, '/phase2_done', _latched)

        self.get_logger().info('digit_finalizer 대기 — /phase1_done 받으면 정면 방문·숫자 확정 시작')

    # ── 콜백 ──────────────────────────────────────────────────────
    def _on_scan(self, msg):
        self.scan = msg

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
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[{label}] 목표 거부')
            return False
        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        ok = rfut.result().status == GoalStatus.STATUS_SUCCEEDED
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

    def dwell_read(self):
        """정지 dwell — 정면에서 숫자를 읽는다(color_mapper 가 색+숫자 확정 투표).
        이 동안 본 digit 다수결을 로깅용으로 반환(-1=못읽음)."""
        self._digit = -1
        seen = []
        end = time.time() + self.dwell_secs
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._digit >= 0:
                seen.append(self._digit)
        return Counter(seen).most_common(1)[0][0] if seen else -1

    # ── 메인 ──────────────────────────────────────────────────────
    def run(self):
        # 1) Phase1 완료 대기
        self.get_logger().info('Phase1(탐사+색좌표) 완료 대기…')
        while rclpy.ok() and not self._phase1_done:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('=== /phase1_done 수신 → Phase2(정면 방문·숫자 확정) 시작 ===')

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
            self.align_frontal()                       # ★ 대각이 아닌 정면으로
            d = self.dwell_read()
            if d >= 0:
                self.get_logger().info(f'{label} → 숫자 {d} 확정(정면)')
                done.append((w['color'], w['id'], d))
            else:
                self.get_logger().warn(f'{label} → 숫자 못 읽음(정면에서도) — 보류')

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
