import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import ObjectDetectionArray, ObjectDetection
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.bridge = CvBridge()

        self.model = YOLO('yolov8n.pt')

        self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.pub = self.create_publisher(ObjectDetectionArray, 'image_yolo', 10)
        self.image_pub = self.create_publisher(Image, 'image_yolo_viz', 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)[0]
        
        arr_msg = ObjectDetectionArray()
        arr_msg.header = msg.header

        for box in results.boxes:
            detection = ObjectDetection()

            detection.class_name = results.names[int(box.cls)]
            detection.confidence = float(box.conf)
            detection.bbox = [int(v) for v in box.xywh[0].tolist()]

            arr_msg.detections.append(detection)

        self.pub.publish(arr_msg)

        annotated = results.plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        img_msg.header = msg.header
        self.image_pub.publish(img_msg)

        self.get_logger().info(f'Published {len(arr_msg.detections)} detections')

def main (args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()