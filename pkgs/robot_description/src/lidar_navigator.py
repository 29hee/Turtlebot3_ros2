# #!/usr/bin/env python3
# import math
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan


# class LidarNavigate(Node):
#     def __init__(self):
#         super().__init__('lidar_navigator')
#         self.state = True
#         self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
#         self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

#     def scan_callback(self, msg):
#         dist = np.min(msg.ranges[175:185])
#         cmd = Twist()
#         if self.state:
#             if dist < 1.0:
#                 cmd.angular.z = 0.5
#                 self.state = True
#             else:
#                 cmd.linear.x = 0.5
#                 self.state = False
#         else:
#             if dist < 0.5:
#                 cmd.angular.z = 0.5
#                 self.state = True
#             else:
#                 cmd.linear.x = 0.5
#                 self.state = False
#         self.pub.publish(cmd)


# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(LidarNavigate())