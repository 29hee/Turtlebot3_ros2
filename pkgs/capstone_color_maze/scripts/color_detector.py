#!/usr/bin/env python3
"""
color_detector.py
TurtleBot3(burger_cam) 의 /camera/image_raw 를 받아 미로 벽의 R/G/B 색을 인식한다.

기능:
  - 카메라 영상 구독 → cv_bridge 로 OpenCV 변환
  - HSV inRange 로 Red / Green / Blue 마스크 → 중앙 ROI 에서 우세 색 판정 → /detected_color(String)
  - 중앙 ROI 안의 '손글씨 숫자(0~9)' 인식(MNIST CNN) → /detected_digit(Int32, -1=없음)
  - show:=true 면 색·숫자(#n)를 화면에 표시

숫자 모델: models/mnist_cnn.pt (없으면 'python3 scripts/train_digit.py' 로 학습).
숫자 끄기: -p digit:=false

다음 단계 예정: 검출 시 TF(map->base_link) + 라이다 정면거리로 색 벽의 map 좌표를 추정해
              color_landmarks.yaml 에 누적 저장.

실행(패키지화 전, 스크립트 직접):
    source /opt/ros/humble/setup.bash
    python3 color_detector.py
    # 화면을 끄고 헤드리스로 돌리려면:  python3 color_detector.py --ros-args -p show:=false
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import cv2
import numpy as np
from cv_bridge import CvBridge

from maze_common import COLOR_RANGES   # HSV 색 범위 단일 출처

# 숫자 인식(손글씨 0~9)용 — torch 없으면 색 기능만 동작하도록 guard
try:
    import torch
    from digit_model import DigitCNN
    _TORCH_OK = True
except Exception:
    _TORCH_OK = False


def default_model_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'models', 'mnist_cnn.pt')


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
        self.declare_parameter('digit', True)         # 손글씨 숫자 인식 ON/OFF
        self.declare_parameter('model_path', default_model_path())
        self.declare_parameter('digit_conf', 0.6)     # 이 확신도 이상이어야 숫자로 인정
        self.declare_parameter('digit_dark_v', 150)   # 색종이 위 '어두운 잉크' 판정 V(명도) 상한
        # 카메라가 거꾸로 장착됨(이 로봇 기준) → 기본 True 로 영상을 뒤집어 바로 세운다.
        # 안 맞추면 숫자가 거꾸로 들어가 MNIST 가 절대 못 읽는다.
        # (카메라가 정방향인 환경/시뮬이면 -p rotate_180:=false)
        self.declare_parameter('rotate_180', True)

        image_topic = self.get_parameter('image_topic').value
        self.show = bool(self.get_parameter('show').value)
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.digit_on = bool(self.get_parameter('digit').value)
        self.model_path = self.get_parameter('model_path').value
        self.digit_conf = float(self.get_parameter('digit_conf').value)
        self.digit_dark_v = int(self.get_parameter('digit_dark_v').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)

        self.bridge = CvBridge()
        self.last_color = None
        self.last_digit = None
        self._dbg28 = None        # 모델 입력 28×28(디버그 표시용)

        # ── 숫자 인식 모델 로드(있을 때만) ────────────────────────
        self.net = None
        if self.digit_on and _TORCH_OK and os.path.exists(self.model_path):
            self.net = DigitCNN()
            self.net.load_state_dict(torch.load(self.model_path, map_location='cpu'))
            self.net.eval()
            self.get_logger().info(f"숫자 인식 ON — 모델: {self.model_path}")
        elif self.digit_on and not _TORCH_OK:
            self.get_logger().warn("torch 미설치 → 숫자 인식 OFF (색만 동작)")
        elif self.digit_on:
            self.get_logger().warn(
                f"숫자 모델 없음({self.model_path}) → 숫자 인식 OFF. "
                f"먼저 'python3 scripts/train_digit.py' 실행")

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/detected_color', 10)
        self.pub_digit = self.create_publisher(Int32, '/detected_digit', 10)   # -1=없음

        self.get_logger().info(
            f"color_detector 시작 — 구독: {image_topic}, show={self.show}, "
            f"digit={'ON' if self.net else 'OFF'}")

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

    def recognize_digit(self, roi_bgr, color_mask=None):
        """ROI 안의 손글씨 숫자 한 개를 인식. (숫자, 확신도) 반환. 없으면 (-1, conf).
        색종이 위에 쓴 경우(color_mask 주어짐): 먼저 색 영역으로 잘라낸 뒤 그 안에서
        숫자를 분리한다 → '흰 벽 vs 색종이' 대비 때문에 종이를 통째로 오인하는 것 방지."""
        self._dbg28 = None
        if self.net is None:
            return -1, 0.0

        if color_mask is not None and cv2.countNonZero(color_mask) > 0.02 * color_mask.size:
            # 색종이 위 숫자: 종이 bbox(테두리 살짝 안쪽) 안에서 '색이 아니고(=글씨) +
            # 어두운(글레어 아님)' 픽셀만 흰색으로. 빨강종이는 V≈255, 글레어도 V 높아 자동 제외.
            pcnts, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            px, py, pw, ph = cv2.boundingRect(max(pcnts, key=cv2.contourArea))
            m = int(0.08 * max(pw, ph))
            y0, y1, x0, x1 = py + m, py + ph - m, px + m, px + pw - m
            sub = roi_bgr[y0:y1, x0:x1]
            if sub.size == 0:
                return -1, 0.0
            v = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)[:, :, 2]
            notcolor = color_mask[y0:y1, x0:x1] == 0
            th = ((notcolor & (v < self.digit_dark_v)).astype(np.uint8)) * 255
            th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        else:
            # 색 없음(흰 바탕 등): 흑백 Otsu 로 어두운 글씨 분리(MNIST 는 검은 배경에 흰 숫자)
            gray = cv2.GaussianBlur(cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return -1, 0.0
        c = max(cnts, key=cv2.contourArea)
        H, W = th.shape
        if cv2.contourArea(c) < 0.01 * H * W:    # 너무 작으면 노이즈로 간주
            return -1, 0.0
        x, y, w, h = cv2.boundingRect(c)
        crop = th[y:y + h, x:x + w]
        # 정사각 패딩 → 20×20 → 28×28 중앙 배치 (MNIST 전처리 관례)
        side = max(w, h)
        sq = np.zeros((side, side), np.uint8)
        sq[(side - h) // 2:(side - h) // 2 + h, (side - w) // 2:(side - w) // 2 + w] = crop
        sq = cv2.resize(sq, (20, 20), interpolation=cv2.INTER_AREA)
        img28 = np.zeros((28, 28), np.uint8)
        img28[4:24, 4:24] = sq
        self._dbg28 = img28                        # 디버그: 모델 입력 그대로 화면에 표시
        t = torch.from_numpy(img28).float().div(255.0)
        t = (t - 0.1307) / 0.3081
        with torch.no_grad():
            prob = torch.softmax(self.net(t.view(1, 1, 28, 28)), dim=1)
            conf, pred = prob.max(1)
        conf, pred = float(conf), int(pred)
        return (pred, conf) if conf >= self.digit_conf else (-1, conf)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        if self.rotate_180:                       # 거꾸로 장착된 카메라 바로 세우기
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

        # ── 손글씨 숫자 인식 (색종이 위 숫자면 그 색 영역 안에서 분리) ──
        color_mask = self.make_mask(hsv, detected) if detected != 'NONE' else None
        digit, dconf = self.recognize_digit(roi, color_mask)
        self.pub_digit.publish(Int32(data=digit))   # -1 = 없음
        if digit != self.last_digit:
            self.get_logger().info(
                f"숫자: {digit if digit >= 0 else '-'} (conf={dconf:.2f})")
            self.last_digit = digit

        # ── 디버그 시각화 ──────────────────────────────────────────
        if self.show:
            box = DRAW_BGR[detected]
            cv2.rectangle(frame, (x1, y1), (x2, y2), box, 2)
            label = f"{detected} ({ratios.get(detected, 0):.2f})" if detected != 'NONE' else "NONE"
            if digit >= 0:
                label += f"  #{digit} ({dconf:.2f})"
            cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box, 2)
            # 모델에 들어가는 28×28 을 우상단에 크게(84×84) 표시 — 전처리 점검용
            if self._dbg28 is not None:
                fh, fw = frame.shape[:2]
                vis = cv2.cvtColor(
                    cv2.resize(self._dbg28, (84, 84), interpolation=cv2.INTER_NEAREST),
                    cv2.COLOR_GRAY2BGR)
                frame[2:86, fw - 86:fw - 2] = vis
                cv2.putText(frame, "model in", (fw - 86, 98),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
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
