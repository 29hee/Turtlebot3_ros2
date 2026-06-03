#!/usr/bin/env python3
"""
color_detector.py
TurtleBot3(burger_cam) 의 /camera/image_raw 를 받아 미로 벽의 R/G/B 색을 인식한다.

Phase 2 - 1단계 (초안):
  - 카메라 영상 구독 → cv_bridge 로 OpenCV 변환
  - HSV inRange 로 Red / Green / Blue 마스크 생성
  - 화면 중앙 ROI(로봇 정면) 에서 가장 우세한 색을 판정
  - 결과를 화면에 표시(디버그) + std_msgs/String 으로 /detected_color 발행

다음 단계 예정: 검출 시 TF(map->base_link) + 라이다 정면거리로 색 벽의 map 좌표를 추정해
              color_landmarks.yaml 에 누적 저장.

실행(패키지화 전, 스크립트 직접):
    source /opt/ros/humble/setup.bash
    python3 color_detector.py
    # 화면을 끄고 헤드리스로 돌리려면:  python3 color_detector.py --ros-args -p show:=false
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge


# ── HSV 색 범위 (OpenCV: H 0~179, S 0~255, V 0~255) ──────────────────────────
# Gazebo/Red·Green·Blue 는 채도가 높은 순색이라 아래 범위로 충분. 조명에 따라 미세조정.
COLOR_RANGES = {
    # 빨강은 Hue 가 0 부근에서 끊겨 두 구간으로 나눠 잡는다.
    'RED':   [((0, 100, 70),   (10, 255, 255)),
              ((170, 100, 70), (179, 255, 255))],
    'GREEN': [((40, 80, 50),   (85, 255, 255))],
    'BLUE':  [((100, 120, 50), (130, 255, 255))],
}

# 화면에 그릴 색(BGR)
DRAW_BGR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'BLUE': (255, 0, 0), 'NONE': (200, 200, 200)}


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('show', True)          # cv2.imshow 디버그 창
        self.declare_parameter('roi_ratio', 0.4)      # 중앙 ROI 한 변 비율(0~1)
        self.declare_parameter('min_ratio', 0.05)     # ROI 내 색 픽셀이 이 비율 넘어야 '검출'

        image_topic = self.get_parameter('image_topic').value
        self.show = bool(self.get_parameter('show').value)
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)

        self.bridge = CvBridge()
        self.last_color = None

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(String, '/detected_color', 10)

        self.get_logger().info(f"color_detector 시작 — 구독: {image_topic}, show={self.show}")

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

        # 색이 바뀔 때만 로그(스팸 방지)
        if detected != self.last_color:
            self.get_logger().info(
                f"검출: {detected}  (R={ratios['RED']:.2f} G={ratios['GREEN']:.2f} B={ratios['BLUE']:.2f})")
            self.last_color = detected

        # /detected_color 발행 (NONE 도 발행해 하위 노드가 상태를 알 수 있게)
        self.pub.publish(String(data=detected))

        # ── 디버그 시각화 ──────────────────────────────────────────
        if self.show:
            box = DRAW_BGR[detected]
            cv2.rectangle(frame, (x1, y1), (x2, y2), box, 2)
            cv2.putText(frame, f"{detected} ({ratios.get(detected, 0):.2f})"
                        if detected != 'NONE' else "NONE",
                        (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box, 2)
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
