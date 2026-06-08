#!/usr/bin/env python3
"""
digit_recognizer.py
근접에서 색 패널 위 숫자(0~9)를 EasyOCR 로 읽어 /detected_digit (Int32, -1=없음) 발행.
파이프라인의 '유일한' digit 소스 — color_mapper(격자 digit 투표)·maze_tour 가 이걸 구독한다.
(과거 Tesseract(color_detector)·YOLO(digit_test_yolo) 백엔드는 폐기, EasyOCR 로 단일화.)

[핵심: 근접 자기-게이팅]
EasyOCR 는 무거워 매 프레임 돌리면 못 버틴다. 또 멀리서는 숫자가 작아 어차피 못 읽는다.
→ 중앙 ROI '색 점유율(coverage)' 이 gate_ratio 이상일 때 = 패널을 가까이서 정면으로
   볼 때만 OCR 을 돌린다(그 외엔 즉시 -1). 추가로 max_rate[Hz] 로 호출 빈도를 묶는다.
   결과: 이동 중/원거리엔 OCR 비용 0, '캡처 순간(근접 dwell)' 에만 인식 → 표가 거기서 쌓임.

전제: /camera/image_raw 는 image_upright 가 똑바로 세운 표준 토픽(거꾸로면 숫자도 거꾸로).
설치:  pip3 install easyocr
실행:  python3 digit_recognizer.py            # 헤드리스
       python3 digit_recognizer.py --ros-args -p show:=true   # 디버그 창
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32MultiArray
import cv2
from cv_bridge import CvBridge

# EasyOCR 없으면 항상 -1 발행하도록 guard (색 파이프라인은 영향 없음)
try:
    import easyocr
    _OCR_OK = True
except Exception:
    _OCR_OK = False


class DigitRecognizer(Node):
    def __init__(self):
        super().__init__('digit_recognizer')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('roi_ratio', 0.5)     # 중앙 ROI 한 변 비율
        self.declare_parameter('gate_ratio', 0.20)   # ROI 색 점유율 ≥ 이 값일 때만 OCR(근접 판정)
        self.declare_parameter('conf_min', 0.4)      # EasyOCR 확신도(0~1) 이 이상만 채택
        self.declare_parameter('max_rate', 4.0)      # OCR 최대 호출 빈도 [Hz] (EasyOCR 보호)
        self.declare_parameter('show', False)        # 디버그 창

        self.image_topic = self.get_parameter('image_topic').value
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.gate_ratio = float(self.get_parameter('gate_ratio').value)
        self.conf_min = float(self.get_parameter('conf_min').value)
        self.max_rate = float(self.get_parameter('max_rate').value)
        self.show = bool(self.get_parameter('show').value)

        self.bridge = CvBridge()
        self._reader = None
        self._last_ocr = 0.0
        self._last_digit = -1

        if _OCR_OK:
            # Reader 생성은 무거우니 1회만. gpu=False(로봇/노트북 CPU 기준).
            self._reader = easyocr.Reader(['en'], gpu=False, verbose=False)
            self.get_logger().info(
                f"digit_recognizer 시작 — EasyOCR 준비됨 "
                f"(gate={self.gate_ratio}, max_rate={self.max_rate}Hz)")
        else:
            self.get_logger().error("easyocr 미설치 → 숫자 항상 -1. 'pip3 install easyocr' 필요")

        # 근접 게이트는 vision_node 의 coverage 를 그대로 쓴다(여기서 HSV 다시 안 돈다 → 단일 디코딩).
        self._cov = 0.0
        self.create_subscription(Float32MultiArray, '/color_signal', self._sig_cb, 10)
        self.create_subscription(Image, self.image_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Int32, '/detected_digit', 10)

    # ──────────────────────────────────────────────────────────────
    def _sig_cb(self, msg):
        if len(msg.data) >= 3:
            self._cov = float(msg.data[2])   # [color_id, cx_norm, coverage]

    def read_digit(self, roi_bgr):
        """EasyOCR 로 ROI 의 단일 숫자 인식. (숫자, conf). 없으면 (-1, best_conf)."""
        try:
            results = self._reader.readtext(roi_bgr, allowlist='0123456789', detail=1)
        except Exception as e:
            self.get_logger().warn(f"OCR 실패: {e}")
            return -1, 0.0
        best_d, best_c = -1, -1.0
        for (_, text, conf) in results:
            text = text.strip()
            # 작품 번호는 한 자리(0~9) 전제. 멀티문자 토큰은 오인식으로 보고 버림.
            if len(text) == 1 and text.isdigit() and conf > best_c:
                best_d, best_c = int(text), float(conf)
        if best_d >= 0 and best_c >= self.conf_min:
            return best_d, best_c
        return -1, max(best_c, 0.0)

    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        h, w = frame.shape[:2]
        rw, rh = int(w * self.roi_ratio), int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        roi = frame[y1:y1 + rh, x1:x1 + rw]

        digit, conf = -1, 0.0
        gated = self._cov >= self.gate_ratio   # vision_node coverage 기반 근접 게이트
        now = time.time()
        if (self._reader is not None and gated
                and now - self._last_ocr >= 1.0 / max(0.1, self.max_rate)):
            self._last_ocr = now
            digit, conf = self.read_digit(roi)

        self.pub.publish(Int32(data=digit))   # -1=없음
        if digit != self._last_digit:
            self.get_logger().info(f"숫자: {digit if digit >= 0 else '-'} (conf={conf:.2f})")
            self._last_digit = digit

        if self.show:
            col = (0, 255, 0) if digit >= 0 else ((0, 200, 255) if gated else (120, 120, 120))
            cv2.rectangle(frame, (x1, y1), (x1 + rw, y1 + rh), col, 2)
            txt = (f"#{digit} ({conf:.2f})" if digit >= 0
                   else ('gate: OCR 중…' if gated else 'far/none'))
            cv2.putText(frame, txt, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
            cv2.imshow('digit_recognizer', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DigitRecognizer()
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
