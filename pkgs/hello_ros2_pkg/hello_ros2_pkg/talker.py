import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 기본 문자열 메시지 타입

class Talker(Node):
    def __init__(self):
        super().__init__('talker')          # 노드 이름을 'talker'로 초기화
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 0.5초마다 실행
        self.i = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! [{self.i}]'
        self.publisher_.publish(msg)                            # 메시지 발행
        self.get_logger().info(f'Publishing: "{msg.data}"')     # 로그 출력
        self.i += 1

def main(args=None):
    rclpy.init(args=args)    # ROS2 초기화
    node = Talker()
    rclpy.spin(node)         # 노드 실행 유지
    node.destroy_node()      # 종료 시 노드 파괴
    rclpy.shutdown()         # ROS2 종료
