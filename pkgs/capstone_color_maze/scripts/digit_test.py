#!/usr/bin/env python3
"""
digit_test.py
터틀봇 카메라로 프린트된 숫자(0~9) + 배경색(R/G/B)이 잘 인식되는지 확인하는 테스트 노드.

실행:
  python3 scripts/digit_test.py
  # 시뮬이면: --ros-args -p rotate_180:=false

화면:
  - 중앙 노란 박스 = ROI
  - 색 감지: 박스 색이 감지된 색으로 바뀜 (빨강/초록/파랑/회색)
  - 우상단: ROI 확대본
  - 상단 텍스트: 색 + 숫자 인식 결과
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

# ── 백엔드 로드 (EasyOCR) ────────────────────────────────────────────────────
try:
    import easyocr
    _reader = easyocr.Reader(['en'], gpu=False, verbose=False)
    _BACKEND = 'easyocr'
except ImportError:
    _reader = None
    _BACKEND = 'none'


def recognize(roi_bgr):
    if _BACKEND != 'easyocr':
        return []
    results = _reader.readtext(roi_bgr, allowlist='0123456789', detail=1)
    out = []
    for (_, text, conf) in results:
        text = text.strip()
        if text.isdigit():
            out.append((text, float(conf)))
    return out


class DigitTestNode(Node):
    def __init__(self):
        super().__init__('digit_test')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('roi_ratio', 0.55)   # 중앙 ROI 크기 비율
        self.declare_parameter('rotate_180', True)  # 카메라 거꾸로 장착 시 True

        self.topic = self.get_parameter('image_topic').value
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)

        self.bridge = CvBridge()
        self._last_result = []
        self._last_log_time = 0.0

        self.get_logger().info(f'digit_test 시작  백엔드={_BACKEND}  토픽={self.topic}')
        if _BACKEND == 'none':
            self.get_logger().error('easyocr 미설치 → pip install easyocr')

        self.create_subscription(
            Image, self.topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 실패: {e}')
            return

        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)

        h, w = frame.shape[:2]
        rw = int(w * self.roi_ratio)
        rh = int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        x2, y2 = x1 + rw, y1 + rh
        roi = frame[y1:y2, x1:x2]

        results = recognize(roi)

        # ── 로그: 결과가 바뀔 때만 ────────────────────────────────────
        now = time.time()
        if results != self._last_result or now - self._last_log_time > 3.0:
            if results:
                for txt, conf in results:
                    self.get_logger().info(f'인식: "{txt}"  conf={conf:.2f}')
            else:
                self.get_logger().info('인식: 없음')
            self._last_result = results
            self._last_log_time = now

        # ── 시각화 ────────────────────────────────────────────────────
        disp = frame.copy()
        cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 255), 2)

        if results:
            label = '  '.join(f'"{t}" {c:.0%}' for t, c in results)
            color = (0, 255, 0)
        else:
            label = 'no digit'
            color = (120, 120, 120)

        cv2.putText(disp, f'[{_BACKEND}] {label}',
                    (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        # ROI 확대본을 우상단에 표시
        thumb_h = min(160, h // 3)
        thumb_w = int(thumb_h * rw / rh)
        thumb = cv2.resize(roi, (thumb_w, thumb_h))
        disp[4:4 + thumb_h, w - thumb_w - 4:w - 4] = thumb
        cv2.rectangle(disp, (w - thumb_w - 4, 4), (w - 4, 4 + thumb_h), (0, 255, 255), 1)
        cv2.putText(disp, 'ROI', (w - thumb_w - 4, thumb_h + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        cv2.imshow('digit_test', disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DigitTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
