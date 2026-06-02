import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot

class RobotMoveServer(Node):
    def __init__(self):
        super().__init__('robot_move_server')
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback
            )
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('목표 거리 이동 시작    ...')
        feedback_msg = MoveRobot.Feedback()
        feedback_msg.current_distance = 0.0
        target = goal_handle.request.target_distance

        for i in range(1, int(target) + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveRobot.Result(reached=False)
            time.sleep(1.0)
            feedback_msg.current_distance = float(i)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'진행 중! : {i}/{target}')
            
        goal_handle.succeed()
        result = MoveRobot.Result()
        result.reached = True
        return result

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RobotMoveServer())
    rclpy.shutdown()