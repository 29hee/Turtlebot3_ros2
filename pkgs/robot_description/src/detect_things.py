#!/usr/bin/env python3
import math
import rclpy

import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
# from numpy import inf, min, array

OBSTACLE_DIST_M = 0.5
FRONT_HALF_DEG = 15


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(Bool, '/is_obstacle', 10)

    def scan_callback(self, msg: LaserScan):
        try:
            dist = np.min(msg.ranges[165:195])
            # cmd = Twist()

            # 장애물 감지 여부 판단 (최소 거리 < 임계값)      
            if dist < OBSTACLE_DIST_M:
               self.obstacle_pub.publish(Bool(data=bool(dist < OBSTACLE_DIST_M)))
                # self.get_logger().info(f'장애물 감지! 최소 거리: {dist:.2f}m')
            else:
                self.obstacle_pub.publish(Bool(data=bool(dist < OBSTACLE_DIST_M)))
                # self.get_logger().info(f'장애물 없음. 최소 거리: {dist:.2f}m')  
            
            # 장애물 감지 여부를 Bool 메시지로 발행 
            
        except Exception as e:
            self.get_logger().error(f"[ERROR] scan_callback failed: {e}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
