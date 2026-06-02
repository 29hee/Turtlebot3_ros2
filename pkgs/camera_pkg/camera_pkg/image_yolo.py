import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO


class ImageYoloPublisher(Node):
    def __init__(self):
        super().__init__('image_yolo_publisher')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # YOLOv8 nano 모델 로드

        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_yolo', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg,
                                           desired_encoding='bgr8')
        results = self.model(frame)
        annotated_frame = results[0].plot()
        yolo_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')  # yolo 결과를 publish
        self.publisher_.publish(yolo_msg)
        self.get_logger().info('YOLO  이미지 발행 중!')


def main(args=None):
    rclpy.init(args=args)
    node = ImageYoloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()