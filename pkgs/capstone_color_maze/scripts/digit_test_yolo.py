#!/usr/bin/env python3
"""
digit_test_yolo.py
터틀봇 카메라로 프린트된 숫자(0~9) 인식 — YOLO classify 백엔드.

학습 먼저:  python3 scripts/train_yolo_digit.py
실행:       python3 scripts/digit_test_yolo.py
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

_HERE = os.path.dirname(os.path.realpath(__file__))
_MODEL_PATH = os.path.join(os.path.dirname(_HERE), 'models', 'yolo_digit_cls.pt')

try:
    from ultralytics import YOLO as _YOLO
    if os.path.exists(_MODEL_PATH):
        _reader = _YOLO(_MODEL_PATH)
        _BACKEND = 'yolo'
    else:
        _reader = None
        _BACKEND = 'none'
except ImportError:
    _reader = None
    _BACKEND = 'none'


def recognize(roi_bgr):
    if _BACKEND != 'yolo':
        return []
    result = _reader(roi_bgr, verbose=False)[0]
    probs = result.probs
    pred = int(probs.top1)
    conf = float(probs.top1conf)
    if conf < 0.5:
        return []
    return [(str(pred), conf)]


class DigitTestYoloNode(Node):
    def __init__(self):
        super().__init__('digit_test_yolo')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('roi_ratio', 0.55)
        self.declare_parameter('rotate_180', True)

        self.topic = self.get_parameter('image_topic').value
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)

        self.bridge = CvBridge()
        self._last_result = []
        self._last_log_time = 0.0

        self.get_logger().info(f'digit_test_yolo 시작  백엔드={_BACKEND}  토픽={self.topic}')
        if _BACKEND == 'none':
            self.get_logger().error(
                f'YOLO 모델 없음: {_MODEL_PATH}\n'
                f'  → python3 scripts/train_yolo_digit.py 먼저 실행하세요.')

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

        now = time.time()
        if results != self._last_result or now - self._last_log_time > 3.0:
            if results:
                for txt, conf in results:
                    self.get_logger().info(f'인식: "{txt}"  conf={conf:.2f}')
            else:
                self.get_logger().info('인식: 없음')
            self._last_result = results
            self._last_log_time = now

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

        thumb_h = min(160, h // 3)
        thumb_w = int(thumb_h * rw / rh)
        thumb = cv2.resize(roi, (thumb_w, thumb_h))
        disp[4:4 + thumb_h, w - thumb_w - 4:w - 4] = thumb
        cv2.rectangle(disp, (w - thumb_w - 4, 4), (w - 4, 4 + thumb_h), (0, 255, 255), 1)

        cv2.imshow('digit_test_yolo', disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DigitTestYoloNode()
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
