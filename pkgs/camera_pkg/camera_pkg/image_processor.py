import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

import cv2
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.current_frame = None

        # 1. 서브스크라이버 생성: 타입=Image, 토픽명='image_raw', 콜백함수=image_callback 구독
        self.subscriber_ = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        # 2. 서비스 서버 생성: capture_callback 서비스 제공
        self.srv = self.create_service(
            Trigger, 'process_image', self.capture_callback)


    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('이미지 수신 완료!')
        cv2.imshow('Camera View', self.current_frame)
        cv2.waitKey(1)

    def capture_callback(self, request, response):
        if self.current_frame is not None:
            # 예시: 처리된 이미지를 저장
            cv2.imwrite('snapshot.png', self.current_frame)
            response.success = True
            response.message = '스냅샷이 저장되었습니다!'
            # self.get_logger().info('서비스 요청 처리 완료!')
        else:
            response.success = False
            response.message = '처리할 이미지가 없습니다.'
            # self.get_logger().warn('서비스 요청 처리 실패: 이미지 없음')
        return response
    

def main (args=None):
    rclpy.init(args=args)
    node = ImageProcessor()

    rclpy.spin(node)
    # except KeyboardInterrupt:
        # pass
    
    node.destroy_node()
    rclpy.shutdown()