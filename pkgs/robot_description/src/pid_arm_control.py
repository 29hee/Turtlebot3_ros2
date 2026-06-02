#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult


class PIDArmController(Node):
    def __init__(self):
        super().__init__('pid_arm_controller')

        # --- 파라미터 선언 ---
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('setpoint', 0.0)   # 목표: 수평 0.0 rad
        self.declare_parameter('dt', 0.01)        # 제어 주기 100Hz
        self.declare_parameter('max_effort', 50.0)
        self.declare_parameter('gravity_gain', 0.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.setpoint = self.get_parameter('setpoint').value
        self.dt = self.get_parameter('dt').value
        self.max_effort = self.get_parameter('max_effort').value
        self.gravity_gain = self.get_parameter('gravity_gain').value

        # --- PID 내부 상태 ---
        self.integral = 0.0
        self.prev_error = 0.0
        self.current_position = 0.0

        # --- 구독 / 발행 ---
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)

        # --- 제어 타이머 ---
        self.timer = self.create_timer(self.dt, self.control_loop)

        # --- 런타임 파라미터 콜백 ---
        self.add_on_set_parameters_callback(self.param_callback)

        self.get_logger().info(
            f'PID Arm Controller started | Kp={self.kp} Ki={self.ki} Kd={self.kd} | '
            f'Setpoint={self.setpoint} rad | gravity_gain={self.gravity_gain} '
        )

    def param_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
            elif param.name == 'ki':
                self.ki = param.value
            elif param.name == 'kd':
                self.kd = param.value
            elif param.name == 'setpoint':
                self.setpoint = param.value
            elif param.name == 'gravity_gain':
                self.gravity_gain = param.value
        self.get_logger().info(
            f'Parameter updated | Kp={self.kp} Ki={self.ki} Kd={self.kd} | Setpoint={self.setpoint} | GravityGain={self.gravity_gain}',
            throttle_duration_sec=1.0)
        return SetParametersResult(successful=True)

    def joint_callback(self, msg: JointState):
        if 'arm_joint' in msg.name:
            idx = msg.name.index('arm_joint')
            self.current_position = msg.position[idx]

    def control_loop(self):
        error = self.setpoint - self.current_position

        # P 항
        p_term = self.kp * error

        # I 항
        self.integral += error * self.dt
        i_term = self.ki * self.integral

        # D 항
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        self.prev_error = error

        # 중력 보상 계산 (setpoint 기준)
        ref_angle = self.setpoint
        gravity_comp = - self.gravity_gain * math.cos(ref_angle)

        # 출력 합산
        effort = p_term + i_term + d_term + gravity_comp

        # Anti-windup (Clamping + Back-calculation)
        if effort > self.max_effort:
            self.integral -= error * self.dt
            effort = self.max_effort
        elif effort < -self.max_effort:
            self.integral -= error * self.dt
            effort = -self.max_effort

        # 발행
        msg = Float64MultiArray()
        msg.data = [effort]
        self.effort_pub.publish(msg)

        # 로그 (튜닝 관찰용, 0.5초 간격)
        self.get_logger().info(
            f'pos={self.current_position:+.4f} | err={error:+.4f} | '
            f'eff={effort:+.3f} | P={p_term:+.2f} I={i_term:+.2f} D={d_term:+.2f} G={gravity_comp:+.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PIDArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 0 토크 발행 (안전)
        zero = Float64MultiArray()
        zero.data = [0.0]
        node.effort_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
