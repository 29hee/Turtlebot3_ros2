import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback) #10Hz

    def timer_callback(self):
        try:
            #map -> camera_link 좌표계의 변환을 요청
            trans = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time())
            t = trans.transform.translation
            self.get_logger().info(
                f'camera_link in map: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}'
                )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF 조회 실패 LookupException: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'TF 트리 미연결 ConnectivityException: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'시간 불일치 ExtrapolationException: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TfListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()