import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageCannyPublisher(Node):
    def __init__(self):
        super().__init__('image_canny_publisher')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_canny', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg,
                                           desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, 100, 150)
        edge_msg = self.bridge.cv2_to_imgmsg(edge, encoding='mono8')
        self.publisher_.publish(edge_msg)
        self.get_logger().info('캐니 이미지 발행 중!')


def main(args=None):
    rclpy.init(args=args)
    node = ImageCannyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()