import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import rcl_interfaces.msg
from rcl_interfaces.msg import SetParametersResult
import time


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        self.declare_parameter('publish_rate', 15.0)  # 발행 주기 설정 (Hz)
        self.declare_parameter('size', [320, 240])  # 이미지 크기 설정 (width, height)
        self.declare_parameter('topic_name', 'image_raw')  # 토픽 이름 설정
        
        
        publish_rate = self.get_parameter('publish_rate').value
        self.size = self.get_parameter('size').value
        self.topic_name = self.get_parameter('topic_name').value

        self.get_logger().info(
            f'카메라 시작: {self.topic_name} , {self.size[0]}x{self.size[1]}, {publish_rate} Hz')
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.publisher_ = self.create_publisher(Image, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_rate' :
                self.rate = param.value
                self.get_logger().info(f'주기 변경: {self.rate} Hz')
                
                self.timer.cancel()  # 기존 타이머 취소
                self.timer = self.create_timer(1.0 / param.value, self.timer_callback)  # 새로운 타이머 생성

                # with open('../config/camera_params.yaml', 'r') as f:
                #     lines = f.readlines()
                # with open('../config/camera_params.yaml', 'w') as f:
                #     for line in lines:
                #         if 'publish_rate' in line and abs(float(line.split(':')[1].strip()) - self.rate) > 5:
                #             indent = line[:len(line) - len(line.lstrip())]
                #             f.write(f'{indent}publish_rate: {self.rate}\n')
                #         else:
                #             f.write(line)

            elif param.name == 'size' :
                self.size = param.value
                self.get_logger().info(f'이미지 크기 변경: {self.size[0]}x{self.size[1]}')
            elif param.name == 'topic_name' :
                self.topic_name = param.value
                self.get_logger().info(f'토픽 이름 변경: {self.topic_name}')
                self.publisher_ = self.create_publisher(Image, self.topic_name, 10)  # 새로운 퍼블리셔 생성
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            resized = cv2.resize(frame, tuple(self.size))
            img_msg = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')

            img_msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            img_msg.header.frame_id = "camera_link"


            self.get_logger().info('이미지 발행 중!')

            self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()

    try :
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()