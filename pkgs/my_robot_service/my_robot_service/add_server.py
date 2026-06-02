import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddTwoInts  


class AddServiceServer(Node):
    def __init__(self):
        super().__init__('add_service_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.set_add_callback)

    def set_add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'요청: {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AddServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()