import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import LedControl

class LedServiceClient(Node):
    def __init__(self):
        super().__init__('led_service_client')
        self.cli = self.create_client(LedControl, 'set_led')
        # 서버가 준비될 때까지 1초마다 확인하며 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서버 대기 중...')
        self.req = LedControl.Request()

    def send_request(self, state):
        self.req.state = state
        self.future = self.cli.call_async(self.req)       # 비동기 요청
        rclpy.spin_until_future_complete(self, self.future)  # 블로킹 대기
        return self.future.result()

import os

def main():
    rclpy.init()
    client = LedServiceClient()

    # 이전 상태 읽기 (없으면 False로 시작)
    state_file = '/tmp/led_state.txt'
    if os.path.exists(state_file):
        with open(state_file) as f:
            current = f.read().strip() == 'True'
    else:
        current = False

    new_state = not current  # 토글

    response = client.send_request(new_state)
    client.get_logger().info(
        f'NOW LED {"ON" if new_state else "OFF"} → 결과: {response.success}, {response.message}')

    # 상태 저장
    with open(state_file, 'w') as f:
        f.write(str(new_state))

    client.destroy_node()
    rclpy.shutdown()

