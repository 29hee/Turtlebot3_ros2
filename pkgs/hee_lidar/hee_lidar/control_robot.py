#!/usr/bin/env python3
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

TURN_DURATION_SEC = 2.0
LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.5
TURN_DIRECTION = 'left'


class State(Enum):
    MOVING = 'moving'
    TURNING = 'turning'


class RobotController(Node):
    """장애물 감지 → 회전 결정 → 명령 발행"""

    def __init__(self):
        super().__init__('robot_controller')
        self.state = State.MOVING
        self.prev_state = State.MOVING
        self.is_obstacle = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.command_pub = self.create_publisher(String, '/robot_command', 10)
        self.obstacle_sub = self.create_subscription(Bool, '/is_obstacle', self.obstacle_callback, 10)

        self.get_logger().info('시작 → 전진')

    # 제어 루프 func (0.1초마다 상태 확인 → 명령 발행)
    def control_loop(self, msg=None):
        if msg is not None:
            self.is_obstacle = msg.data
        if self.prev_state != self.state:
            print(f'상태 변경 감지 : {self.prev_state} → {self.state}, 장애물 감지: {self.is_obstacle}')

        if self.state == State.MOVING:
            if self.is_obstacle:
                self.state = State.TURNING
                self.state_pub.publish(String(data='turning'))
                self.move_command(String(data='turning'))
                self.get_logger().info('장애물 감지 → 회전 시작')
            else:
                self.move_command(String(data='forward'))
                self.state_pub.publish(String(data='moving'))
        elif self.state == State.TURNING:
            # self.get_logger().info('회전 중...')
            if not self.is_obstacle:
                self.state = State.MOVING
                self.state_pub.publish(String(data='moving'))
                self.move_command(String(data='forward'))
                self.get_logger().info('장애물 제거 → 전진 시작')
            else:
                self.move_command(String(data='turning'))
                self.state_pub.publish(String(data='turning'))

    
    # 장애물 감지 콜백 func
    def obstacle_callback(self, msg: Bool):
        # if msg.data != self.is_obstacle:
        #     self.get_logger().info(f'장애물 : {msg.data}')
            # pass  # 장애물 감지 여부가 변경될 때마다 로그 출력
        self.is_obstacle = msg.data

    # 이동/회전 명령 발행 func
    def move_command(self, msg: String):
        # self.get_logger().info(f'행동 명령: {msg.data}')
        self.command_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

