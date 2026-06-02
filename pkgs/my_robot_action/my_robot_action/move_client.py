import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot

class RobotMoveClient(Node):
    def __init__(self):
        super().__init__('robot_move_client')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')

    def send_goal(self, distance):
        goal_msg = MoveRobot.Goal()
        goal_msg.target_distance = distance

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
                                goal_msg,
                                feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('서버가 님 거절함')
            return

        self.get_logger().info('목표가 수락되었습니다. ')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.current_distance
        self.get_logger().info(f'피드백: 현재 이동 거리 {distance}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.reached:
            self.get_logger().info(f'목표 거리에 도달! : {result.reached}')
        else:
            self.get_logger().info('목표 달성에 실패했습니다.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMoveClient()
    node.send_goal(5.0)
    rclpy.spin(node)