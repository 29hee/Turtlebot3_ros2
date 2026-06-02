import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts  


class AddServiceClient(Node):
    def __init__(self):
        super().__init__('add_service_client')
        self.cli = self.create_client(
            AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 서버 기다리는 중...')
        
    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
def main():
    rclpy.init()
    node = AddServiceClient()
    response = node.send_request(10, 20)
    node.get_logger().info(f'결과: {response.sum}')
    node.destroy_node()
    rclpy.shutdown()