#!/usr/bin/env python3
"""
sim_digit_pub.py — Gazebo 시뮬레이션용 가상 digit 발행기

test_panels.world 의 패널 위치를 알고 있어서,
로봇 정면 LiDAR 투영점이 패널에 가까우면 해당 digit 을 /detected_digit 에 발행.

실행:
  python3 sim_digit_pub.py
  python3 sim_digit_pub.py --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import tf2_ros

# test_panels.world 패널 위치 ↔ digit 대응표
PANELS = [
    {'color': 'RED',   'digit': 1, 'x': -0.800, 'y': -1.955},
    {'color': 'RED',   'digit': 2, 'x':  1.955, 'y': -0.600},
    {'color': 'GREEN', 'digit': 1, 'x':  0.800, 'y': -1.955},
    {'color': 'GREEN', 'digit': 2, 'x':  0.000, 'y':  1.955},
    {'color': 'GREEN', 'digit': 3, 'x': -1.955, 'y':  0.600},
]
HIT_DIST = 0.45   # 투영점↔패널 중심 거리 임계 [m]


def _yaw_from_tf(tf):
    q = tf.transform.rotation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


class SimDigitPub(Node):
    def __init__(self):
        super().__init__('sim_digit_pub')
        self.scan = None
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Int32, '/detected_digit', 10)
        self.create_subscription(LaserScan, '/scan',
                                 lambda msg: setattr(self, 'scan', msg),
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self._loop)
        self.get_logger().info('sim_digit_pub 시작')

    def _front_range(self):
        s = self.scan
        if s is None or not s.ranges:
            return None
        n   = len(s.ranges)
        i0  = int(round(-s.angle_min / s.angle_increment)) % n
        win = max(1, int(math.radians(5) / s.angle_increment))
        vals = [s.ranges[(i0+k) % n] for k in range(-win, win+1)
                if math.isfinite(s.ranges[(i0+k) % n])
                and s.range_min <= s.ranges[(i0+k) % n] <= s.range_max]
        return float(np.median(vals)) if vals else None

    def _loop(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        d = self._front_range()
        if d is None:
            return

        rx  = tf.transform.translation.x
        ry  = tf.transform.translation.y
        yaw = _yaw_from_tf(tf)

        # 정면 LiDAR 투영점
        px = rx + d * math.cos(yaw)
        py = ry + d * math.sin(yaw)

        for panel in PANELS:
            if math.hypot(px - panel['x'], py - panel['y']) < HIT_DIST:
                self.pub.publish(Int32(data=panel['digit']))
                return

        self.pub.publish(Int32(data=-1))


def main(args=None):
    rclpy.init(args=args)
    node = SimDigitPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
