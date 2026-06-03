#!/usr/bin/env python3
"""
wall_follower.py - 오른손 벽타기로 미로를 한 바퀴 돌며 SLAM 매핑을 돕는 노드.

단순연결 미로(모든 벽이 외곽에 붙음)에서는 오른손 법칙만으로 전체 벽을
훑게 되므로 slam_toolbox 맵이 완성된다.

종료: Ctrl-C 또는 --duration 초 경과 시 자동 정지.
"""
import math
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self, duration):
        super().__init__('wall_follower')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.start = self.get_clock().now()
        self.duration = duration
        self.scan = None

        # 파라미터
        self.target_right = 0.45   # 오른쪽 벽 유지 거리(m)
        self.front_stop = 0.45     # 정면 이보다 가까우면 좌회전
        self.lost_right = 0.9      # 오른쪽이 이보다 멀면 벽 잃음 -> 우회전
        self.v_fwd = 0.15
        self.kp = 1.6

    def sector_min(self, deg_lo, deg_hi):
        """주어진 각도(deg, 로봇 정면=0, 좌+ / 우-) 구간의 최소 유효거리."""
        s = self.scan
        n = len(s.ranges)
        vals = []
        d = deg_lo
        while d <= deg_hi:
            ang = math.radians(d)
            idx = int(round((ang - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and not math.isinf(r) and not math.isnan(r) and s.range_min < r < s.range_max:
                vals.append(r)
            d += 1
        return min(vals) if vals else float('inf')

    def on_scan(self, msg):
        self.scan = msg

    def on_timer(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        if elapsed > self.duration:
            self.pub.publish(Twist())
            self.get_logger().info(f'시간 종료({self.duration}s) -> 정지')
            rclpy.shutdown()
            return
        if self.scan is None:
            return

        front = self.sector_min(-20, 20)
        right = self.sector_min(-100, -80)

        cmd = Twist()
        if front < self.front_stop:
            # 정면 막힘 -> 제자리 좌회전
            cmd.linear.x = 0.0
            cmd.angular.z = 0.7
        elif right > self.lost_right:
            # 오른쪽 열림(코너) -> 전진하며 우회전해서 벽 다시 잡기
            cmd.linear.x = 0.12
            cmd.angular.z = -0.6
        else:
            # 벽 따라가기: 오른쪽 거리 오차 보정
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.8, min(0.8, -self.kp * err))
        self.pub.publish(cmd)


def main():
    duration = 120.0
    for i, a in enumerate(sys.argv):
        if a == '--duration' and i + 1 < len(sys.argv):
            duration = float(sys.argv[i + 1])
    rclpy.init()
    node = WallFollower(duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
