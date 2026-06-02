# #!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# OBSTACLE_DIST_M = 0.5
# FRONT_HALF_DEG = 15  # 전방 ±15도


# class LidarTurn(Node):
#     def __init__(self):
#         super().__init__('lidar_turn')
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.is_obstacle_detected = False

#     def scan_callback(self, msg: LaserScan):
#         ranges = msg.ranges
#         total = len(ranges)

#         # 전방 ±15도 인덱스 (index 0 = 정면)
#         front_indices = list(range(0, FRONT_HALF_DEG + 1)) + list(range(total - FRONT_HALF_DEG, total))
#         front_ranges = [
#             ranges[i] for i in front_indices
#             if not math.isnan(ranges[i]) and not math.isinf(ranges[i]) and ranges[i] > 0.0
#         ]

#         twist = Twist()
#         if front_ranges and min(front_ranges) < OBSTACLE_DIST_M:
#             twist.linear.x = 0.0
#             twist.angular.z = 0.5  # 제자리 회전
#             if not self.is_obstacle_detected:
#                 self.get_logger().info(f'장애물 감지 {min(front_ranges):.2f}m → 회전')
#                 self.is_obstacle_detected = True
#         else:
#             twist.linear.x = 0.2   # 전진
#             twist.angular.z = 0.0
#             if self.is_obstacle_detected:
#                 self.get_logger().info('장애물 없음 → 전진')
#                 self.is_obstacle_detected = False

#         self.cmd_pub.publish(twist)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarTurn()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
