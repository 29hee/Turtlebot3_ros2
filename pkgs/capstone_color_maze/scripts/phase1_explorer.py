#!/usr/bin/env python3
"""
phase1_explorer.py — Phase 1: SLAM 한 바퀴 + 색 위치 후보 수집

SLAM(slam_toolbox)이 돌아가는 동안 오른손 벽타기로 미로 외곽을 한 바퀴 돌며
색 벽의 대략적인 맵 좌표를 격자 투표로 수집한다.

근접 접근(dwell)은 하지 않는다 — 이동 중 보이는 색을 그대로 기록.
원거리 관측이라 오차가 있지만, Phase 2에서 정면 방문+확인을 하므로 후보 수준으로 충분.

종료 조건:
  A. 루프 클로저(시작점 복귀)                         ← 우선
  B. 시간 상한(duration, 기본 600s)                   ← 안전 상한

종료 시: maps/color_candidates.yaml 저장 + /phase1_done(True) 발행.

출력 형식:
  candidates:
    - color: RED
      x: 1.50         # 맵 프레임 격자 중심 [m]
      y: 0.30
      votes: 12
      approach_yaw: 1.57  # 관측 평균 yaw → Phase 2 가 이 방향으로 정면 접근
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import tf2_ros
import yaml

from maze_common import id_to_color, VALID_COLORS


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_candidates_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_candidates.yaml')


class Phase1Explorer(Node):
    def __init__(self, duration):
        super().__init__('phase1_explorer')

        # ── 주행 파라미터 ──────────────────────────────────────────
        self.declare_parameter('v_fwd', 0.10)
        self.declare_parameter('target_right', 0.40)
        self.declare_parameter('front_stop', 0.45)
        self.declare_parameter('spin_speed', 0.30)
        # ── 색 관측 파라미터 ───────────────────────────────────────
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 1.5)   # Phase1은 원거리 후보도 수집
        self.declare_parameter('grid_res', 0.30)
        self.declare_parameter('min_votes', 3)     # Phase1: 후보 단계라 낮게
        self.declare_parameter('seen_ratio', 0.05) # vision_node coverage 임계
        # ── 루프 클로저 파라미터 ───────────────────────────────────
        self.declare_parameter('visit_res', 0.40)
        self.declare_parameter('loop_close_dist', 0.50)
        self.declare_parameter('leave_dist', 2.0)       # 이 거리 이상 벗어나야 "출발"로 인정
        self.declare_parameter('min_visited_cells', 20) # 이 셀 수 이상 방문해야 클로저 허용
        self.declare_parameter('save_period', 5.0)      # 후보 자동 저장 주기 [s]
        self.declare_parameter('save_path', default_candidates_path())

        self.total = duration
        self.v_fwd = float(self.get_parameter('v_fwd').value)
        self.target_right = float(self.get_parameter('target_right').value)
        self.front_stop = float(self.get_parameter('front_stop').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.grid_res = float(self.get_parameter('grid_res').value)
        self.min_votes = int(self.get_parameter('min_votes').value)
        self.seen_ratio = float(self.get_parameter('seen_ratio').value)
        self.visit_res = float(self.get_parameter('visit_res').value)
        self.loop_close_dist = float(self.get_parameter('loop_close_dist').value)
        self.leave_dist = float(self.get_parameter('leave_dist').value)
        self.min_visited_cells = int(self.get_parameter('min_visited_cells').value)
        self.save_period = float(self.get_parameter('save_period').value)
        self.save_path = self.get_parameter('save_path').value

        # ── 상태 ──────────────────────────────────────────────────
        self.scan = None
        self.color = 'NONE'
        self.color_cov = 0.0
        # {(gx, gy): {'RED': 0, 'GREEN': 0, 'BLUE': 0, 'yaw_sum': 0.0, 'count': 0}}
        self.votes = {}
        self.start = self.now()
        self.start_cell = None
        self.left_start = False
        self.visited = set()

        # ── IO ────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_done = self.create_publisher(Bool, '/phase1_done', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/color_signal', self.on_signal, 10)

        self.timer = self.create_timer(0.05, self.on_timer)
        self.create_timer(self.save_period, self.save)   # 주기적 자동 저장
        self.get_logger().info(
            f'phase1_explorer 시작 — 우측 벽타기 한 바퀴, '
            f'max_range={self.max_range}m, 시간상한={self.total:.0f}s\n'
            f'  루프 클로저: leave_dist={self.leave_dist}m, '
            f'min_cells={self.min_visited_cells}, close_dist={self.loop_close_dist}m\n'
            f'  Ctrl+C 로도 종료 가능 (맵이 완성됐다고 판단되면)')

    # ── 기본 유틸 ──────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now()

    def elapsed(self, since):
        return (self.now() - since).nanoseconds / 1e9

    def on_scan(self, msg):
        self.scan = msg

    def on_signal(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        cov = float(d[2])
        self.color = id_to_color(int(d[0])) if cov >= self.seen_ratio else 'NONE'
        self.color_cov = cov

    def front_range(self):
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return None
        n = len(s.ranges)
        i0 = int(round(-s.angle_min / s.angle_increment)) % n
        win = max(1, int(math.radians(5) / s.angle_increment))
        vals = [s.ranges[(i0 + k) % n] for k in range(-win, win + 1)
                if math.isfinite(s.ranges[(i0 + k) % n])
                and s.range_min <= s.ranges[(i0 + k) % n] <= s.range_max]
        return float(np.median(vals)) if vals else None

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

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def cell_of(self, x, y, res=None):
        r = res or self.grid_res
        return int(math.floor(x / r)), int(math.floor(y / r))

    def cell_center(self, gx, gy):
        return (gx + 0.5) * self.grid_res, (gy + 0.5) * self.grid_res

    # ── 색 관측 격자 투표 ──────────────────────────────────────────
    def record_candidate(self, pose):
        """정면에 보이는 색을 맵 좌표로 투영해 격자 투표."""
        if self.color not in VALID_COLORS:
            return
        d = self.front_range()
        if d is None or not (self.min_range <= d <= self.max_range):
            return
        x, y, yaw = pose
        wx = x + math.cos(yaw) * d
        wy = y + math.sin(yaw) * d
        key = self.cell_of(wx, wy)
        if key not in self.votes:
            self.votes[key] = {'RED': 0, 'GREEN': 0, 'BLUE': 0,
                               'yaw_sum': 0.0, 'count': 0}
        entry = self.votes[key]
        entry[self.color] += 1
        entry['yaw_sum'] += yaw
        entry['count'] += 1

    # ── 벽타기 ────────────────────────────────────────────────────
    def wall_follow_cmd(self):
        front = self.sector_min(-20, 20)
        right = self.sector_min(-100, -80)
        cmd = Twist()
        if front < self.front_stop:
            cmd.angular.z = 0.4
        elif right > 0.9:
            cmd.linear.x = 0.07
            cmd.angular.z = -0.3
        else:
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.35, min(0.35, -1.0 * err))
        return cmd

    # ── 메인 루프 ──────────────────────────────────────────────────
    def on_timer(self):
        if self.elapsed(self.start) > self.total:
            self.finish('시간 상한 도달')
            return
        if self.scan is None:
            return

        pose = self.get_pose()
        if pose is None:
            cmd = Twist()
            cmd.angular.z = self.spin_speed
            self.pub.publish(cmd)
            return

        x, y, yaw = pose
        vcell = self.cell_of(x, y, self.visit_res)
        self.visited.add(vcell)

        if self.start_cell is None:
            self.start_cell = vcell

        self.record_candidate(pose)

        d_start = math.hypot(
            x - (self.start_cell[0] + 0.5) * self.visit_res,
            y - (self.start_cell[1] + 0.5) * self.visit_res)
        if not self.left_start and d_start > self.leave_dist:
            self.left_start = True
            self.get_logger().info(f'출발 확인 (시작점에서 {d_start:.1f}m)')
        if (self.left_start
                and d_start < self.loop_close_dist
                and len(self.visited) >= self.min_visited_cells):
            self.finish('루프 클로저 완료')
            return

        self.pub.publish(self.wall_follow_cmd())

    # ── 저장 + 종료 ────────────────────────────────────────────────
    def save(self):
        candidates = []
        for (gx, gy), entry in self.votes.items():
            total = entry['RED'] + entry['GREEN'] + entry['BLUE']
            if total < self.min_votes:
                continue
            best_color = max(VALID_COLORS, key=lambda c: entry[c])
            if entry[best_color] == 0:
                continue
            cx, cy = self.cell_center(gx, gy)
            avg_yaw = entry['yaw_sum'] / entry['count']
            candidates.append({
                'color': best_color,
                'x': round(cx, 3),
                'y': round(cy, 3),
                'votes': entry[best_color],
                'approach_yaw': round(avg_yaw, 3),
            })
        data = {'candidates': candidates}
        save_dir = os.path.dirname(self.save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        try:
            with open(self.save_path, 'w') as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
            self.get_logger().info(
                f'color_candidates.yaml 저장: {len(candidates)}개 후보 → {self.save_path}')
        except Exception as e:
            self.get_logger().error(f'저장 실패: {e}')

    def finish(self, reason):
        self.pub.publish(Twist())
        self.save()
        self.pub_done.publish(Bool(data=True))
        confirmed = len([v for v in self.votes.values()
                         if v['RED'] + v['GREEN'] + v['BLUE'] >= self.min_votes])
        self.get_logger().info(
            f'Phase 1 완료: {reason} '
            f'(방문셀={len(self.visited)}, 후보셀={confirmed})')
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
    node = Phase1Explorer(duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.pub.publish(Twist())
        node.save()
        node.pub_done.publish(Bool(data=True))
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
