#!/usr/bin/env python3
"""
color_detector.py
TurtleBot3(burger_cam) 의 /camera/image_raw 를 받아 미로 벽의 R/G/B 색을 인식한다.

기능:
  - 카메라 영상 구독 → cv_bridge 로 OpenCV 변환
  - HSV inRange 로 Red / Green / Blue 마스크 → 중앙 ROI 에서 우세 색 판정 → /detected_color(String)
  - 중앙 ROI 안의 '숫자(0~9)' 인식(Tesseract OCR) → /detected_digit(Int32, -1=없음)
  - show:=true 면 색·숫자(#n)를 화면에 표시

숫자 인식: Tesseract OCR(pytesseract) 사용. 별도 학습/모델 가중치 불필요.
설치:  sudo apt install tesseract-ocr  &&  pip3 install pytesseract
숫자 끄기: -p digit:=false

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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import cv2
import numpy as np
from cv_bridge import CvBridge

from maze_common import COLOR_RANGES   # HSV 색 범위 단일 출처

# 숫자 인식(0~9)용 OCR — pytesseract 없으면 색 기능만 동작하도록 guard
try:
    import pytesseract
    _OCR_OK = True
except Exception:
    _OCR_OK = False

# 단일 숫자 한 글자만 읽도록: psm 10(낱문자) + 0~9 화이트리스트
_OCR_CONFIG = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'


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
        self.declare_parameter('digit', True)         # 숫자 인식 ON/OFF
        self.declare_parameter('digit_conf', 0.4)     # OCR 확신도(0~1) 컷. psm10(낱문자)은 정답이어도 conf 가 낮게 나와 0.4 로 둠
        # 숫자는 항상 '검은 글자' 전제(흰 벽/색 패널 어디서나 잉크가 배경보다 어둡다).
        self.declare_parameter('digit_max_fill', 0.6)  # 글씨 후보가 영역의 이 비율보다 크면 배경 오분할로 보고 기각
        # 카메라 상하반전 보정은 '소스에서' image_upright.py 가 표준 토픽(/camera/image_raw)을
        # 똑바로 세워 발행하므로 여기서는 기본 False(이중 회전 방지). RViz·rqt 도 같은 토픽이라
        # 함께 똑바로 보인다. image_upright 를 안 쓰고 이 노드가 직접 뒤집어야 하면 -p rotate_180:=true.
        self.declare_parameter('rotate_180', False)

        image_topic = self.get_parameter('image_topic').value
        self.show = bool(self.get_parameter('show').value)
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.digit_on = bool(self.get_parameter('digit').value)
        self.digit_conf = float(self.get_parameter('digit_conf').value)
        self.digit_max_fill = float(self.get_parameter('digit_max_fill').value)
        self.rotate_180 = bool(self.get_parameter('rotate_180').value)

        self.bridge = CvBridge()
        self.last_color = None
        self.last_digit = None
        self._dbg_ocr = None      # OCR 입력 이미지(디버그 표시용)

        # ── 숫자 인식(OCR) 사용 가능 여부 점검 ───────────────────
        self.ocr_on = False
        if self.digit_on and _OCR_OK:
            try:
                ver = pytesseract.get_tesseract_version()
                self.ocr_on = True
                self.get_logger().info(f"숫자 인식 ON — Tesseract OCR v{ver}")
            except Exception as e:
                self.get_logger().warn(
                    f"tesseract 실행 불가 → 숫자 인식 OFF (색만 동작): {e}. "
                    f"'sudo apt install tesseract-ocr' 필요")
        elif self.digit_on and not _OCR_OK:
            self.get_logger().warn(
                "pytesseract 미설치 → 숫자 인식 OFF (색만 동작). "
                "'pip3 install pytesseract' 필요")

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/detected_color', 10)
        self.pub_digit = self.create_publisher(Int32, '/detected_digit', 10)   # -1=없음

        self.get_logger().info(
            f"color_detector 시작 — 구독: {image_topic}, show={self.show}, "
            f"digit={'ON' if self.ocr_on else 'OFF'}")

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
        """ROI 안의 '검은 숫자' 한 개를 Tesseract OCR 로 인식. (숫자, 확신도) 반환. 없으면 (-1, conf).
        숫자는 항상 검은 글자라는 전제(흰 벽엔 흰 글씨를 못 쓰므로) → '어두운 픽셀 = 글씨' 로 통일.
        색 패널(R/G/B) 위면: 먼저 패널로 잘라낸 뒤 그 안에서 어두운 잉크만 분리
        (흰 벽 vs 색 패널 대비로 패널을 통째 글씨로 오인하는 것 방지)."""
        self._dbg_ocr = None
        if not self.ocr_on:
            return -1, 0.0

        if color_mask is not None and cv2.countNonZero(color_mask) > 0.02 * color_mask.size:
            # 색 패널(R/G/B) 위 검은 숫자: 패널 bbox(테두리 살짝 안쪽)로 자른 뒤 그 안에서
            # Otsu 역임계로 '어두운 잉크'만 분리. 패널 색은 잉크보다 밝아 자동으로 갈린다.
            pcnts, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            px, py, pw, ph = cv2.boundingRect(max(pcnts, key=cv2.contourArea))
            m = int(0.08 * max(pw, ph))
            y0, y1, x0, x1 = py + m, py + ph - m, px + m, px + pw - m
            sub = roi_bgr[y0:y1, x0:x1]
            if sub.size == 0:
                return -1, 0.0
        else:
            # 흰 벽(기본 배경) 위 검은 숫자. 빈 흰 벽이면 어두운 덩이가 없어 아래 면적필터에서
            # 걸러져 '숫자 없음' → 흰 벽 자체를 숫자로 오인하지 않는다.
            sub = roi_bgr
        gray = cv2.GaussianBlur(cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return -1, 0.0
        c = max(cnts, key=cv2.contourArea)
        H, W = th.shape
        area = cv2.contourArea(c)
        # 너무 작으면 노이즈, 너무 크면(영역 대부분) 배경 오분할 → 둘 다 '숫자 아님'
        if area < 0.01 * H * W or area > self.digit_max_fill * H * W:
            return -1, 0.0
        x, y, w, h = cv2.boundingRect(c)
        crop = th[y:y + h, x:x + w]                # 흰 글씨(255) on 검은 배경(0)
        # 정사각 패딩 → 64×64 로 키우고 둘레 여백 추가. Tesseract 는 글자 주변 여백이 필요.
        side = max(w, h)
        sq = np.zeros((side, side), np.uint8)
        sq[(side - h) // 2:(side - h) // 2 + h, (side - w) // 2:(side - w) // 2 + w] = crop
        sq = cv2.resize(sq, (64, 64), interpolation=cv2.INTER_AREA)
        margin = 24
        canvas = np.zeros((64 + 2 * margin, 64 + 2 * margin), np.uint8)
        canvas[margin:margin + 64, margin:margin + 64] = sq
        ocr_img = 255 - canvas                     # Tesseract 규격: 검은 글씨 on 흰 배경
        self._dbg_ocr = ocr_img                    # 디버그: OCR 입력 그대로 화면에 표시

        try:
            data = pytesseract.image_to_data(
                ocr_img, config=_OCR_CONFIG, output_type=pytesseract.Output.DICT)
        except Exception as e:
            self.get_logger().warn(f"OCR 실패: {e}")
            return -1, 0.0

        # 0~9 화이트리스트라 숫자만 나온다. conf 최대인 한 글자 토큰을 채택.
        best_pred, best_conf = -1, -1.0
        for txt, cf in zip(data['text'], data['conf']):
            txt = (txt or '').strip()
            try:
                cf = float(cf)
            except (TypeError, ValueError):
                cf = -1.0
            if len(txt) == 1 and txt.isdigit() and cf > best_conf:
                best_pred, best_conf = int(txt), cf
        conf = max(best_conf, 0.0) / 100.0         # tesseract conf 는 0~100
        return (best_pred, conf) if best_pred >= 0 and conf >= self.digit_conf else (-1, conf)

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
            # OCR 에 들어가는 이미지를 우상단에 크게(84×84) 표시 — 전처리 점검용
            if self._dbg_ocr is not None:
                fh, fw = frame.shape[:2]
                vis = cv2.cvtColor(
                    cv2.resize(self._dbg_ocr, (84, 84), interpolation=cv2.INTER_NEAREST),
                    cv2.COLOR_GRAY2BGR)
                frame[2:86, fw - 86:fw - 2] = vis
                cv2.putText(frame, "ocr in", (fw - 86, 98),
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
