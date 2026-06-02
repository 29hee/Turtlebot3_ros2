#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

OBSTACLE_DIST_M = 0.2
FRONT_HALF_DEG = 15


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.obstacle_pub = self.create_publisher(Bool, '/is_obstacle', 10)

    def scan_callback(self, msg: LaserScan):
        try:
            ranges = msg.ranges
            total = len(ranges)
            
            # 전방 각도 범위를 라디안으로 정확히 계산
            angle_per_index = msg.angle_increment  # 이미 라디안 단위
            front_range_rad = math.radians(FRONT_HALF_DEG)
            front_range_indices = int(front_range_rad / abs(angle_per_index))
            
            front_indices = list(range(0, front_range_indices + 1)) \
                            + list(range(total - front_range_indices, total))
            
            # 전방의 모든 거리값 (inf 포함)
            front_ranges_raw = [ranges[i] for i in front_indices]
            
            # .inf는 "너무 가까움" 또는 "벽" → 장애물로 간주
            # 유효한 값 중 최소값 찾기
            valid_ranges = [r for r in front_ranges_raw 
                           if not math.isnan(r) and not math.isinf(r) and r > 0.0]
            
            # 감지 조건: 1) .inf 있음 OR 2) 유효값이 임계값 미만
            has_inf = any(math.isinf(r) for r in front_ranges_raw)
            min_valid_dist = min(valid_ranges) if valid_ranges else float('inf')
            detected = has_inf or (min_valid_dist < OBSTACLE_DIST_M)
            
            # 디버그 로그 (INFO로 변경해서 바로 보임)
            # self.get_logger().info(f"[SCAN] front_indices={len(front_indices)}, "
            #                       f"inf_count={sum(1 for r in front_ranges_raw if math.isinf(r))}, "
            #                       f"valid_count={len(valid_ranges)}, "
            #                       f"min_dist={min_valid_dist:.3f}m, "
            #                       f"detected={detected}")
            
            # 장애물 감지 여부를 Bool 메시지로 발행
            self.obstacle_pub.publish(Bool(data=detected))
            
        except Exception as e:
            self.get_logger().error(f"[ERROR] scan_callback failed: {e}", throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
#     main()
