# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import math
# import time

# SIDE_DURATION = 1.5   # 직진 시간 (초)
# TURN_DURATION = 1.0   # 회전 시간 (초)
# LINEAR_SPEED  = 2.0   # 직진 속도 (m/s)
# ANGULAR_SPEED = math.pi / 2  # 90도/s → 1초에 정확히 90도 회전

# class Square(Node):
#     def __init__(self):
#         super().__init__('square')
#         self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.state = 'forward'   # 'forward' or 'turn'
#         self.elapsed = 0.0
#         self.sides_done = 0
#         self.state_start = time.time()

#     def timer_callback(self):
#         self.elapsed = time.time() - self.state_start
#         msg = Twist()

#         if self.state == 'forward':
#             msg.linear.x = LINEAR_SPEED
#             if self.elapsed >= SIDE_DURATION:
#                 self.state = 'turn'
#                 self.elapsed = 0.0
#                 # self.state_start = time.time()
#                 self.get_logger().info(f'직진 완료 → 회전 시작 (변 {self.sides_done + 1})')

#         elif self.state == 'turn':
#             msg.angular.z = ANGULAR_SPEED
#             if self.elapsed >=TURN_DURATION:
#                 self.state = 'forward'
#                 # self.state_start = time.time()
#                 self.sides_done += 1
#                 self.get_logger().info(f'회전 완료 → 직진 시작 (완료한 변: {self.sides_done})')

#         self.publisher.publish(msg)

#     def stop(self):
#         self.publisher.publish(Twist())  # 속도 0 발행
#         self.timer.cancel()
#         self.get_logger().info('사각형 완료!')


# def main(args=None):
#     rclpy.init(args=args)
#     node = Square()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        
        # 1. /turtle1/cmd_vel 토픽으로 Twist 메시지 발행 설정
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 2. 1초마다 콜백 함수를 실행하는 타이머 생성
        self.timer_period = 1.0  # 1초
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 상태 제어를 위한 변수들
        self.count = 0
        self.is_moving = True  # True: 전진 단계, False: 회전 단계

    def timer_callback(self):
        msg = Twist()

        if self.count < 8:  # 전진 4회 + 회전 4회 = 총 8회 동작
            if self.is_moving:
                # [전진 단계] 1초 동안 선속도 2.0으로 이동
                msg.linear.x = 2.0
                msg.angular.z = 0.0
                self.get_logger().info(f'Step {self.count // 2 + 1}: Moving Forward')
                self.is_moving = False  # 다음 단계는 회전으로 변경
            else:
                # [회전 단계] 1초 동안 각속도 π/2(90도)로 회전
                msg.linear.x = 0.0
                msg.angular.z = 1.5708  # 약 π/2
                self.get_logger().info(f'Step {self.count // 2 + 1}: Rotating 90 degrees')
                self.is_moving = True   # 다음 단계는 전진으로 변경
            
            self.publisher_.publish(msg)
            self.count += 1
        else:
            # 모든 미션 완료 후 정지
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Mission Complete: Square Finished!')
            # 타이머 중단 또는 노드 종료 로직
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()