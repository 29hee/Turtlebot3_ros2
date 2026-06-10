#!/usr/bin/env python3
"""scan_restamp.py — /scan 의 헤더 stamp 를 '최신 /odom 메시지의 stamp'에 맞춰 다시 찍어
/scan_synced 로 발행한다.

[왜 이렇게?]
Pi↔PC 클럭이 안 맞으면(시간 동기 안 함) Nav2/AMCL 이 깨진다. 그런데 단순히 'PC 현재시각(now)'
으로 스캔을 다시 찍으면, 로봇의 odom→base_link TF 는 여전히 'Pi 시각'이라서 scan(PC시각)↔
odom(Pi시각)의 시각이 어긋난다 → AMCL 이 스캔과 주행을 잘못 짝지어 추정이 '발산'(맵 밖으로).

해결: /scan 과 /odom 은 둘 다 로봇(Pi)에서 같은 시계로 나온다. 그래서 스캔 stamp 를
'가장 최근에 받은 odom 의 stamp' 로 맞춰주면:
  - scan ↔ odom→base_link TF 시각이 정확히 정렬 → AMCL 이 올바르게 수렴(발산 없음)
  - 그 시각의 TF 가 버퍼에 분명히 있으므로 costmap 의 'earlier than transform cache' 드롭도 사라짐
즉 PC 의 절대 시각을 끌어들이지 않고(클럭 동기 불필요) 로봇 내부 시각 일관성만 유지한다.

amcl 의 scan_topic / costmap observation source 를 /scan_synced 로 가리키면 된다(nav2_maze.yaml).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class ScanRestamp(Node):
    def __init__(self):
        super().__init__('scan_restamp')
        self._last_odom_stamp = None     # 가장 최근 odom 메시지의 stamp(=odom TF 시각)
        self._n = 0
        self.pub = self.create_publisher(LaserScan, 'scan_synced', qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.get_logger().info(
            'scan_restamp 시작 — /scan stamp 를 최신 /odom stamp 에 정렬 → /scan_synced '
            '(scan↔odom 시각 일치 → AMCL 발산 방지, 클럭 동기 불필요)')

    def odom_cb(self, msg):
        self._last_odom_stamp = msg.header.stamp

    def scan_cb(self, msg):
        if self._last_odom_stamp is not None:
            # 핵심: 스캔을 'odom 과 같은 시각'으로 정렬(둘 다 Pi 시각) → TF 짝맞춤 정확.
            msg.header.stamp = self._last_odom_stamp
        else:
            # 아직 odom 을 못 받았으면 현재시각으로(부팅 직후 한정).
            msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self._n += 1
        if self._n % 200 == 0:
            self.get_logger().info(f'scan_synced 발행 {self._n}개 (odom stamp 정렬 중)')


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
