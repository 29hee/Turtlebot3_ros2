#!/usr/bin/env python3
"""scan_restamp.py — /scan 의 헤더 stamp 를 '현재 시각(now)'으로 다시 찍어 /scan_synced 로 발행.

Pi↔PC 클럭 skew 로 스캔 stamp 가 PC 의 TF 보다 과거가 되면 Nav2 costmap/amcl 이
'timestamp earlier than transform cache' / 'queue is full' 로 스캔을 전부 드롭한다.
→ 그러면 위치추정/장애물갱신이 안 돼 planner 가 경로를 못 만든다.

이 릴레이는 스캔을 받는 즉시 PC 의 현재 시각으로 stamp 를 덮어써 /scan_synced 로 보낸다.
TF 도 PC 시각이므로 stamp 가 항상 일치 → 드롭 0. 시간 동기(chrony) 없이 동작한다.
(저속 주행 기준, 스캔 캡처~수신 지연으로 인한 stamp 오차는 무시 가능.)

amcl 의 scan_topic 과 costmap observation source 를 /scan_synced 로 가리키게 하면 된다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRestamp(Node):
    def __init__(self):
        super().__init__('scan_restamp')
        # 센서 QoS(best_effort) 로 구독/발행 — 라이다 표준.
        self.pub = self.create_publisher(LaserScan, 'scan_synced', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.cb, qos_profile_sensor_data)
        self._n = 0
        self.get_logger().info('scan_restamp 시작 — /scan → /scan_synced (stamp=now, 클럭 무시)')

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()   # 클럭 skew 무시: 현재 시각으로
        self.pub.publish(msg)
        self._n += 1
        if self._n % 200 == 0:
            self.get_logger().info(f'scan_synced 발행 {self._n}개 (stamp 재기록 중)')


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamp()
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
