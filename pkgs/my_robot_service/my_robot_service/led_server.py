import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import LedControl

class LedServiceServer(Node):
    def __init__(self):
        super().__init__('led_service_server')
        self.srv = self.create_service(
            LedControl, 'set_led', self.set_led_callback)
        self.get_logger().info('LED 서비스 서버가 시작되었습니다.')

    def set_led_callback(self, request, response):
        if request.state:
            self.get_logger().info('LED 켜기 요청 수신')
            response.success = True
            response.message = "LED를 성공적으로 켰습니다."
        else:
            self.get_logger().info('LED 끄기 요청 수신')
            response.success = True
            response.message = "LED를 성공적으로 껐습니다."
        return response  # 반드시 response 반환!

def main():
    rclpy.init()
    node = LedServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()

