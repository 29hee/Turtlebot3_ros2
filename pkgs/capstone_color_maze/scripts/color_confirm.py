#!/usr/bin/env python3
"""
color_confirm.py
런타임 '벽 확인(confirmation)' 술어 노드.  [Phase 3 - 런타임]

역할:
  - /camera/image_raw 를 구독해 target_color HSV 마스크를 만들고
  - 마스크가 '프레임 전체'의 몇 %를 덮는지(coverage) 계산해
  - coverage 를 /target_coverage (std_msgs/Float32, 0~1) 로,
    coverage >= 0.60 여부를 /target_confirmed (std_msgs/Bool) 로 발행한다.

이 노드는 색을 '판정'하지 않는다(그건 color_detector). 오직 target_color 한 색의
프레임 점유율만 보고 "이 벽이 충분히 가까이/정면에 있나?"를 술어로 답한다.
maze_tour.py(내비게이션)가 이 토픽을 보고 벽 도착을 확정한다.

사양 근거:
  - "runtime per-wall color confirmation requires the target-color HSV mask to cover
     at least 60% of the camera frame"  → 전체 프레임 기준(ROI 아님).

실행:
  source /opt/ros/humble/setup.bash
  python3 color_confirm.py --ros-args -p target_color:=RED
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
import cv2
import numpy as np
from cv_bridge import CvBridge

from maze_common import COLOR_RANGES, CONFIRM_THRESHOLD, normalize_color, is_confirmed


class ColorConfirm(Node):
    def __init__(self):
        super().__init__('color_confirm')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_color', 'RED')
        self.declare_parameter('threshold', CONFIRM_THRESHOLD)   # 프레임 점유율 임계
        self.declare_parameter('open_kernel', 3)                 # 노이즈 제거 커널 크기

        image_topic = self.get_parameter('image_topic').value
        self.threshold = float(self.get_parameter('threshold').value)
        k = int(self.get_parameter('open_kernel').value)
        self.kernel = np.ones((k, k), np.uint8) if k > 0 else None

        self.target = normalize_color(self.get_parameter('target_color').value)
        if self.target is None:
            raise ValueError(
                f"target_color 가 RED/GREEN/BLUE 중 하나가 아님: "
                f"{self.get_parameter('target_color').value!r}")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.pub_cov = self.create_publisher(Float32, '/target_coverage', 10)
        self.pub_ok = self.create_publisher(Bool, '/target_confirmed', 10)

        self.get_logger().info(
            f"color_confirm 시작 — target={self.target}, 임계={self.threshold:.0%}, "
            f"구독={image_topic}")

    def target_mask(self, hsv):
        """target 색 HSV 범위들을 OR 로 합쳐 이진 마스크 반환."""
        mask = None
        for lo, hi in COLOR_RANGES[self.target]:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return mask

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   # 전체 프레임(ROI 아님)
        mask = self.target_mask(hsv)
        total = max(1, mask.shape[0] * mask.shape[1])
        coverage = int(cv2.countNonZero(mask)) / total

        self.pub_cov.publish(Float32(data=float(coverage)))
        self.pub_ok.publish(Bool(data=bool(is_confirmed(coverage, self.threshold))))


def main(args=None):
    rclpy.init(args=args)
    node = ColorConfirm()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
