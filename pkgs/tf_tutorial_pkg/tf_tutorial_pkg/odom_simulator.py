import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

import math

class OdomSimulator(Node):
    def __init__(self):
        super().__init__('odom_simulator')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.timer_callback) #20Hz
        self.radius = 1.0 #원 반경(m)
        self.omega = 0.5 #각속도(rad/s)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds / 1e9 #초 단위 시간

        #원 운동 계산 : 로봇이 odom 좌표게 중심을 돌면서 주행
        x = self.radius * math.cos(self.omega * t)
        y = self.radius * math.sin(self.omega * t)

        #접선 방향이 yaw(이동방향) 이 되도록 계산
        roll  = 0.0
        pitch = 0.0
        yaw  = self.omega * t + math.pi / 2 #접선 방향

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        trans = TransformStamped()
        trans.header.stamp = now.to_msg()       #현재 시간으로 타임스탬프 설정
        trans.header.frame_id = 'odom'          #odom 좌표계가 부모 프레임
        trans.child_frame_id = 'base_link' #base_link가 자식 프레임
        trans.transform.translation.x = x   
        trans.transform.translation.y = y
        trans.transform.translation.z = 0.0     #평면 주행이므로 z는 0
        trans.transform.rotation.x = qx     #쿼터니언으로 회전 설정
        trans.transform.rotation.y = qy
        trans.transform.rotation.z = qz
        trans.transform.rotation.w = qw

        self.br.sendTransform(trans)

def main(args=None):
    rclpy.init(args=args)
    node = OdomSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()