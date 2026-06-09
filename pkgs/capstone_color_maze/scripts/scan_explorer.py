#!/usr/bin/env python3
"""
scan_explorer.py - 벽면 + '월드 중앙'까지 카메라/라이다로 빠짐없이 매핑하는 탐사 주행.

[왜 필요한가]
wall_follower 는 벽을 오른쪽에 끼고 '나란히'만 달려 (1) 전방 카메라가 벽을 face-on
으로 못 보고, (2) 둘레만 돌아 중앙부 occlusion 면이 부실할 수 있다.
→ ①둘레는 '주기적 느린 360° 회전'으로 벽면을 정면 스캔하고,
  ②그 다음 '월드 중앙'으로 들어가 내부 지점들에서 스캔해 가운데까지 채운다.

[동작 = 2국면 상태기계]
  국면1 PERIMETER (앞 perimeter_frac 비율 시간):
    P_SPIN(제자리 360° 느린회전) ↔ P_DRIVE(오른손 벽타기) 반복 — 둘레 벽면 스캔
  국면2 INTERIOR (남은 시간):
    I_GOTO(TF map 좌표로 내부 웨이포인트까지 go-to-goal + 전방 반응회피)
      → 도착/막힘 시 I_SPIN(제자리 360°) → 다음 웨이포인트 ...
  total duration 초 후 정지.

[전제]
  TF 'map' 프레임 필요(slam_toolbox 가동 중). 회전은 느리게(맵 안 깨지게).
  color_mapper 를 함께 띄워야 색이 누적됨(mapping.launch.py 가 같이 실행).

실행(패키지화 전):
  python3 scan_explorer.py --duration 660 --drive 7 --spin-speed 0.3
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class ScanExplorer(Node):
    def __init__(self, duration, drive_secs, spin_speed, spin_overshoot,
                 perimeter_frac):
        super().__init__('scan_explorer')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.scan = None

        # TF (map -> base_link) 로 맵 좌표/자세 취득 (color_mapper 와 동일 방식)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 시간/국면 파라미터 ───────────────────────────────────────
        self.total = duration
        self.perimeter_secs = duration * perimeter_frac   # 둘레 국면 지속
        self.drive_secs = drive_secs                      # 한 번의 P_DRIVE 지속
        # 회전 각속도 [rad/s]. ★ 느리게! 빠르면 SLAM 스캔매칭이 깨져 맵이 뒤틀린다(실환경).
        self.spin_speed = spin_speed
        self.spin_secs = (2.0 * math.pi * spin_overshoot) / max(0.05, spin_speed)

        # ── 벽타기(P_DRIVE) 파라미터 ────────────────────────────────
        self.target_right = 0.45
        self.front_stop = 0.45
        self.lost_right = 0.9
        self.v_fwd = 0.15
        self.kp = 1.6

        # ── 내부(INTERIOR) 파라미터 ─────────────────────────────────
        # 맵 중앙(0,0) + 네 모서리(대각) + 사방 내부 지점. 모서리를 먼저 방문해
        # NW 사각지대(RED 서벽 -2.4,0.8 / BLUE 북벽 -1.5,2.4)를 ~1.2m 근거리에서 정면 스캔.
        # (이전 버전은 모서리 지점이 없어 NW 두 패널을 상시 누락했다 — 실측 확인.)
        self.waypoints = [(0.0, 0.0),
                          (-1.2, 1.2), (-1.2, -1.2), (1.2, 1.2), (1.2, -1.2),
                          (0.0, 1.2), (-1.2, 0.0), (0.0, -1.2), (1.2, 0.0)]
        self.wp_idx = 0
        self.arrive_tol = 0.55      # 웨이포인트 도착 판정 [m]
        self.front_avoid = 0.5      # 내부 전방 회피 임계 [m]
        self.goto_timeout = 25.0    # 한 웨이포인트 최대 시도 [s] (막히면 그자리 스핀)

        # ── 상태 ────────────────────────────────────────────────────
        self.start = self.get_clock().now()
        self.phase = 'P_SPIN'       # 시작 위치부터 한 바퀴 스캔
        self.phase_start = self.start
        self.spin_count = 0
        self.get_logger().info(
            f"scan_explorer 시작 — total={self.total:.0f}s "
            f"(둘레 {self.perimeter_secs:.0f}s → 내부 {self.total-self.perimeter_secs:.0f}s), "
            f"drive={self.drive_secs:.0f}s, spin={self.spin_secs:.1f}s@{self.spin_speed:.2f}rad/s")

    # ──────────────────────────────────────────────────────────────
    def on_scan(self, msg):
        self.scan = msg

    def elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds / 1e9

    def get_pose(self):
        """map->base_link → (x, y, yaw) 또는 None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def sector_min(self, deg_lo, deg_hi):
        """정면=0, 좌+ / 우- 구간의 최소 유효거리 [m]."""
        s = self.scan
        n = len(s.ranges)
        vals = []
        d = deg_lo
        while d <= deg_hi:
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and not math.isinf(r) and not math.isnan(r) and s.range_min < r < s.range_max:
                vals.append(r)
            d += 1
        return min(vals) if vals else float('inf')

    def drive_cmd(self):
        """오른손 벽타기 한 스텝 Twist."""
        front = self.sector_min(-20, 20)
        right = self.sector_min(-100, -80)
        cmd = Twist()
        if front < self.front_stop:
            cmd.angular.z = 0.7
        elif right > self.lost_right:
            cmd.linear.x = 0.12
            cmd.angular.z = -0.6
        else:
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.8, min(0.8, -self.kp * err))
        return cmd

    def goto_cmd(self, tx, ty, pose):
        """맵 좌표 (tx,ty) 로 향하는 go-to-goal + 전방 반응회피. (도착여부, Twist)."""
        x, y, yaw = pose
        dist = math.hypot(tx - x, ty - y)
        if dist < self.arrive_tol:
            return True, Twist()
        cmd = Twist()
        # 전방이 막히면 더 열린 쪽으로 제자리 회전(장애물 회피)
        if self.sector_min(-25, 25) < self.front_avoid:
            left = self.sector_min(20, 70)
            right = self.sector_min(-70, -20)
            cmd.angular.z = 0.6 if left > right else -0.6
            return False, cmd
        # 목표 방향으로 조향
        herr = wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd.angular.z = max(-0.8, min(0.8, 1.2 * herr))
        cmd.linear.x = self.v_fwd if abs(herr) < 0.5 else 0.05
        return False, cmd

    # ──────────────────────────────────────────────────────────────
    def on_timer(self):
        if self.elapsed(self.start) > self.total:
            self.pub.publish(Twist())
            self.get_logger().info(f'시간 종료({self.total:.0f}s) → 정지')
            # ★ 타이머 콜백 안에서 rclpy.shutdown() 을 부르면 executor 가 자기 자신을
            #   join 하려다 교착되어 프로세스가 안 끝난다(매핑 후 map_saver 무한대기 원인).
            #   대신 SystemExit 를 던져 rclpy.spin() 밖으로 빠져나간 뒤 main 에서 정리한다.
            self.timer.cancel()
            raise SystemExit
        if self.scan is None:
            return
        in_perimeter = self.elapsed(self.start) < self.perimeter_secs

        # ── 국면1: 둘레 ────────────────────────────────────────────
        if in_perimeter:
            if self.phase == 'P_SPIN':
                self._spin_step('P_DRIVE')
            elif self.phase == 'P_DRIVE':
                self.pub.publish(self.drive_cmd())
                if self.elapsed(self.phase_start) >= self.drive_secs:
                    self._switch('P_SPIN')
            else:  # 내부 상태였다면(있을 수 없음) 둘레로 복귀
                self._switch('P_SPIN')
            return

        # 둘레 → 내부 전환(최초 1회): 내부 스핀부터
        if self.phase.startswith('P_'):
            self.get_logger().info('=== 둘레 완료 → 월드 중앙(내부) 탐사 시작 ===')
            self._switch('I_GOTO')

        # ── 국면2: 내부 ────────────────────────────────────────────
        if self.phase == 'I_SPIN':
            self._spin_step('I_GOTO')
            return

        # I_GOTO
        pose = self.get_pose()
        if pose is None:
            # 위치를 못 받으면 안전하게 제자리 느린 회전(맵/색은 계속 쌓임)
            c = Twist(); c.angular.z = self.spin_speed; self.pub.publish(c)
            return
        tx, ty = self.waypoints[self.wp_idx]
        arrived, cmd = self.goto_cmd(tx, ty, pose)
        timeout = self.elapsed(self.phase_start) >= self.goto_timeout
        if arrived or timeout:
            why = '도착' if arrived else '시도초과'
            self.get_logger().info(
                f'내부 WP#{self.wp_idx+1}/{len(self.waypoints)} ({tx:+.1f},{ty:+.1f}) {why} → 스캔')
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self._switch('I_SPIN')
            return
        self.pub.publish(cmd)

    def _spin_step(self, next_phase):
        """제자리 360° 느린 회전 한 스텝. 완료되면 next_phase 로."""
        c = Twist(); c.angular.z = self.spin_speed
        self.pub.publish(c)
        if self.elapsed(self.phase_start) >= self.spin_secs:
            self.spin_count += 1
            self.get_logger().info(f'스캔 #{self.spin_count} 완료 → 다음 동작')
            self._switch(next_phase)

    def _switch(self, phase):
        self.phase = phase
        self.phase_start = self.get_clock().now()


def _arg(name, default, cast=float):
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            return cast(sys.argv[i + 1])
    return default


def main():
    duration = _arg('--duration', 660.0)
    # 스캔 사이 이동 시간 [s]. ↓ 작을수록 스핀이 촘촘 → 모든 패널을 ~1m 이내서 한 번은 스캔.
    # 10s(≈1.5m 간격)면 패널이 스핀 사이 빈틈에 빠져 누락됨(파랑 서/북). 7s(≈1m 간격)로 좁힘.
    drive_secs = _arg('--drive', 7.0)
    # ★ 회전은 '느리게'. 빠르면 slam_toolbox 가 못 따라가 맵이 뒤틀린다(실환경 검증). 0.3≈24s/바퀴.
    spin_speed = _arg('--spin-speed', 0.3)
    overshoot = _arg('--overshoot', 1.15)
    # 둘레:내부 시간 배분(0.6 → 둘레 60%). 둘레 벽타기가 북벽 등 외곽 패널을, 내부 9점
    # 링 스핀이 코너/사각지대를 보완. duration 을 넉넉히(720s) 주어 두 페이즈 다 완주시킨다.
    perimeter_frac = _arg('--perimeter-frac', 0.6)
    rclpy.init()
    node = ScanExplorer(duration, drive_secs, spin_speed, overshoot, perimeter_frac)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):   # SystemExit = --duration 경과로 정상 종료
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
