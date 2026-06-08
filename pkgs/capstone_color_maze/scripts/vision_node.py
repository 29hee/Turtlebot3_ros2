#!/usr/bin/env python3
"""
vision_node.py — 카메라 영상을 '딱 한 번' 풀어 색 신호를 계산하는 단일 디코더.

[왜] 예전엔 color_mapper·maze_explorer·digit_recognizer 가 '각자' 매 프레임 HSV/inRange 를
돌려 PC CPU 를 3중으로 먹었다(카메라 throttling 의 PC측 원인). → 색 계산은 여기서 한 번만
하고, 결과를 가벼운 토픽으로 뿌린다. 나머지 노드는 영상 대신 이 토픽을 구독한다.

발행:
  /detected_color   (std_msgs/String)              우세색 RED|GREEN|BLUE|NONE (기존 호환)
  /color_signal     (std_msgs/Float32MultiArray)   [color_id, cx_norm, coverage]
        color_id : maze_common.COLOR_IDS 인덱스 (0=NONE,1=RED,2=GREEN,3=BLUE)
        cx_norm  : 우세색 blob 중심 x, 화면중앙 기준 -1(왼쪽)~+1(오른쪽)  (비주얼 서보용)
        coverage : 중앙 ROI 내 우세색 점유율 0~1                          (근접/게이팅용)

구독: /camera/image_raw (image_upright 가 똑바로 세운 표준 토픽)
실행: python3 vision_node.py            (디버그 창: -p show:=true)
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

from maze_common import COLOR_RANGES, color_to_id

DRAW_BGR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'BLUE': (255, 0, 0), 'NONE': (200, 200, 200)}


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('roi_ratio', 0.7)     # 중앙 ROI 한 변 비율
        self.declare_parameter('min_ratio', 0.03)    # 이 점유율 미만이면 NONE(작은 색도 잡게 낮춤)
        self.declare_parameter('show', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.show = bool(self.get_parameter('show').value)

        self.bridge = CvBridge()
        self._last = None
        self.create_subscription(Image, self.image_topic, self.cb, qos_profile_sensor_data)
        self.pub_color = self.create_publisher(String, '/detected_color', 10)
        self.pub_sig = self.create_publisher(Float32MultiArray, '/color_signal', 10)
        self.get_logger().info(f"vision_node 시작 — 단일 디코더, 구독 {self.image_topic}")

    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return
        h, w = frame.shape[:2]
        rw, rh = int(w * self.roi_ratio), int(h * self.roi_ratio)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        roi = frame[y0:y0 + rh, x0:x0 + rw]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        area = max(1, rw * rh)
        kernel = np.ones((3, 3), np.uint8)

        best, best_cov, best_mask = 'NONE', 0.0, None
        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            cov = cv2.countNonZero(mask) / area
            if cov > best_cov:
                best, best_cov, best_mask = color, cov, mask

        color = best if best_cov >= self.min_ratio else 'NONE'
        cx_norm = 0.0
        if color != 'NONE' and best_mask is not None:
            mmt = cv2.moments(best_mask)
            if mmt['m00'] > 0:
                cx = mmt['m10'] / mmt['m00']
                cx_norm = (cx - rw / 2.0) / (rw / 2.0)

        self.pub_color.publish(String(data=color))
        self.pub_sig.publish(Float32MultiArray(
            data=[float(color_to_id(color)), float(cx_norm), float(best_cov)]))

        if color != self._last:
            self.get_logger().info(f"색: {color} (cov={best_cov:.2f}, cx={cx_norm:+.2f})")
            self._last = color

        if self.show:
            box = DRAW_BGR[color]
            cv2.rectangle(frame, (x0, y0), (x0 + rw, y0 + rh), box, 2)
            cv2.putText(frame, f"{color} cov={best_cov:.2f} cx={cx_norm:+.2f}",
                        (x0, max(20, y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box, 2)
            cv2.imshow('vision_node', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
