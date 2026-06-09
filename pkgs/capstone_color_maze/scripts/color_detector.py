#!/usr/bin/env python3
"""
color_detector.py
TurtleBot3(burger_cam) 의 /camera/image_raw 를 받아 미로 벽의 R/G/B 색을 인식한다.

기능(색 전용):
  - 카메라 영상 구독 → cv_bridge 로 OpenCV 변환
  - HSV inRange 로 Red / Green / Blue 마스크 → 중앙 ROI 에서 우세 색 판정 → /detected_color(String)
  - show:=true 면 색·마스크를 화면에 표시(실조명 HSV 보정용 디버그 도구)

숫자 인식은 별도 노드 digit_recognizer.py(EasyOCR) 가 전담한다(/detected_digit 발행).
  → 여기서는 색만. 색 범위는 maze_common.COLOR_RANGES 단일 출처.

실행(패키지화 전, 스크립트 직접):
    source /opt/ros/humble/setup.bash
    python3 color_detector.py
    # 화면을 끄고 헤드리스로 돌리려면:  python3 color_detector.py --ros-args -p show:=false
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge

from maze_common import COLOR_RANGES   # HSV 색 범위 단일 출처


# 화면에 그릴 색(BGR)
DRAW_BGR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'BLUE': (255, 0, 0), 'NONE': (200, 200, 200)}


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('show', True)          # cv2.imshow 디버그 창
        self.declare_parameter('roi_ratio', 0.7)      # 중앙 ROI 한 변 비율(0~1). 크게=색을 더 넓게 본다
        self.declare_parameter('min_ratio', 0.05)     # ROI 내 색 픽셀이 이 비율 넘어야 '검출'
        # 카메라 상하반전 보정은 보통 image_upright.py 가 소스에서 처리(표준 토픽이 똑바로).
        # 이 노드를 image_upright 없이 거꾸로 영상에 직접 물릴 때만 -p rotate_180:=true.
        self.declare_parameter('rotate_180', False)

        image_topic = self.get_parameter('image_topic').value
        self.show = bool(self.get_parameter('show').value)
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)

        self.bridge = CvBridge()
        self.last_color = None

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/detected_color', 10)

        self.get_logger().info(
            f"color_detector(색 전용) 시작 — 구독: {image_topic}, show={self.show}")

    # ──────────────────────────────────────────────────────────────
    def make_mask(self, hsv, color):
        """주어진 색의 HSV 범위들을 OR 로 합쳐 이진 마스크 반환."""
        mask = None
        for lo, hi in COLOR_RANGES[color]:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        if self.rotate_180:                       # 거꾸로 장착된 카메라 바로 세우기(보정 미사용 시)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]

        # 중앙 ROI (로봇 정면) 좌표
        rw, rh = int(w * self.roi_ratio), int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        x2, y2 = x1 + rw, y1 + rh
        roi = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_area = max(1, rw * rh)

        # 색별 ROI 내 픽셀 비율 계산 → 최댓값 선택
        ratios = {}
        for color in COLOR_RANGES:
            mask = self.make_mask(hsv, color)
            ratios[color] = int(cv2.countNonZero(mask)) / roi_area

        best = max(ratios, key=ratios.get)
        detected = best if ratios[best] >= self.min_ratio else 'NONE'

        # 색이 바뀔 때만 로그(스팸 방지). 모든 색 비율을 함께 출력.
        if detected != self.last_color:
            ratio_str = ' '.join(f"{c[0]}={ratios[c]:.2f}" for c in COLOR_RANGES)
            self.get_logger().info(f"검출: {detected}  ({ratio_str})")
            self.last_color = detected

        # /detected_color 발행 (NONE 도 발행해 하위 노드가 상태를 알 수 있게)
        self.pub.publish(String(data=detected))

        # ── 디버그 시각화 ──────────────────────────────────────────
        if self.show:
            box = DRAW_BGR[detected]
            cv2.rectangle(frame, (x1, y1), (x2, y2), box, 2)
            label = f"{detected} ({ratios.get(detected, 0):.2f})" if detected != 'NONE' else "NONE"
            cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box, 2)
            cv2.imshow("color_detector", frame)
            cv2.waitKey(1)   # GUI 갱신 필수


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
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
