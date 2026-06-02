#!/usr/bin/env python3
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

TURN_DURATION_SEC = 2.0
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.5


class State(Enum):
    MOVING = 'moving'
    TURNING = 'turning'

class RobotMover(Node):
    """명령을 구독해서 이동만 수행함"""

    def __init__(self):
        super().__init__('robot_mover')
        self.state = State.MOVING
        self.turn_direction = 'left'
        self.turn_start_time = None
        self.cmd_sub = self.create_subscription(String, '/robot_command', self.move_command, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_sub = self.create_subscription(String, '/robot_state', self.state_callback, 10)
        
        # 초기 상태는 MOVING → 전진 명령 발행
        # self.subscriptions.append(self.state_sub)
        # self.get_logger().info('시작 → 전진')

    # /robot_command 수신 → 명령 실행
    def move_command(self, msg: String):
        if msg.data == 'forward':
            self.move_forward()
        elif msg.data == 'turning':
            self.move_turn('left')

    # /robot_state 수신 (로그용)
    def state_callback(self, msg: String):
        # self.get_logger().info(f'상태: {msg.data}')
        pass

    def move_forward(self):
        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        # self.get_logger().info('전진 중..')
        self.cmd_pub.publish(twist)

    def move_turn(self, direction: str):
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED
        # self.get_logger().info(f'{direction} 회전 중..')
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     main()
