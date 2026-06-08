#!/usr/bin/env python3
"""
image_upright.py
거꾸로 장착된 버거 카메라를 '소스에서 한 번' 회전해 똑바로 세워 재발행한다.
이 노드 하나만 거치면 RViz·OCR(color_detector)·rqt·기타 모든 구독자가 정상 방향
영상을 본다(각 노드가 따로 뒤집을 필요 없음 → 이중 회전/누락 footgun 제거).

[대역폭: 무선엔 '압축 1스트림'만]
기본(compressed_in=True)으로 v4l2 의 압축 토픽(in_topic+'/compressed', JPEG)을 받아
PC 에서 한 번만 디코딩→회전→표준 토픽(raw+compressed)을 'PC 로컬'로 재발행한다.
→ WiFi 에는 압축 1개만 흐르고, PC측 구독자(color_mapper/digit_recognizer/explorer…)가
   몇 개든 무선 부하는 그대로다. (raw 를 무선으로 받던 기존 대비 ~10배 절감.)
   ※ Pi 에 compressed_image_transport 필요: sudo apt install ros-humble-compressed-image-transport
   raw 로 받고 싶으면 -p compressed_in:=false.

토픽 배선:
    입력(원본, 거꾸로):  in_topic(+/compressed)  기본 /camera/image_raw_rot   (v4l2, 로봇)
    출력(똑바로):        out_topic               기본 /camera/image_raw        ← 표준 토픽(raw)
    출력(압축):          out_topic + '/compressed'                            ← 경량 뷰어용(정방향)

이 노드는 순수 릴레이라 PC 에서 돌린다(로봇 Pi 에 cv_bridge·repo 불필요, Pi CPU 절약).
    (로봇) ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw_rot
    (PC)   python3 scripts/image_upright.py

회전 파라미터 flip:  180(상하+좌우, 기본) | v(상하만) | h(좌우만)
실행:  python3 image_upright.py    (좌우만 뒤집힘:  -p flip:=h)
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge


class ImageUpright(Node):
    def __init__(self):
        super().__init__('image_upright')

        self.declare_parameter('in_topic', '/camera/image_raw_rot')   # 원본(거꾸로)
        self.declare_parameter('out_topic', '/camera/image_raw')      # 보정본(표준 토픽)
        self.declare_parameter('flip', '180')                         # 180 | v | h
        # True: 무선 절약 위해 in_topic+'/compressed'(JPEG) 구독. False: raw Image 구독.
        self.declare_parameter('compressed_in', True)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.out_topic = out_topic
        self.flip = str(self.get_parameter('flip').value).lower()
        self.compressed_in = bool(self.get_parameter('compressed_in').value)
        if self.flip not in ('180', 'v', 'h'):
            self.get_logger().warn(
                f"알 수 없는 flip='{self.flip}' → 회전 없이 원본 그대로 통과. 180|v|h 중 하나로 줄 것.")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        # 똑바로 선 '압축' 영상도 함께 발행 → WiFi 에서 RViz/rqt 가 정방향 영상을 가볍게 확인.
        self.pub_c = self.create_publisher(
            CompressedImage, out_topic + '/compressed', qos_profile_sensor_data)

        if self.compressed_in:
            self.sub = self.create_subscription(
                CompressedImage, in_topic + '/compressed', self.cb_compressed,
                qos_profile_sensor_data)
            src = in_topic + '/compressed'
        else:
            self.sub = self.create_subscription(
                Image, in_topic, self.cb_raw, qos_profile_sensor_data)
            src = in_topic
        self.get_logger().info(
            f"image_upright 시작 — {src} → {out_topic}(+/compressed) "
            f"(flip={self.flip}, compressed_in={self.compressed_in})")

        # ★ '순차 보장' 가드: out_topic 은 image_upright 만 발행해야 한다. v4l2 가 _rot 으로
        #   remap 안 돼 같은 토픽을 직접 쏘면 발행자 2개 → 구독자가 거꾸로/똑바로 프레임을
        #   '랜덤'으로 받게 된다(보정 깨짐). 주기적으로 발행자 수를 확인해 그 즉시 경고.
        self._dup_warned = False
        self.create_timer(3.0, self._check_single_publisher)

    def _check_single_publisher(self):
        n = self.count_publishers(self.out_topic)
        if n > 1 and not self._dup_warned:
            self._dup_warned = True
            self.get_logger().error(
                f"⚠ {self.out_topic} 발행자 {n}개! image_upright 외에 다른 노드(v4l2?)가 직접 발행 중 "
                f"→ 거꾸로/똑바로 프레임이 랜덤으로 섞인다. v4l2 를 '/camera/image_raw_rot' 로 "
                f"remap 했는지 확인: -r /image_raw:=/camera/image_raw_rot")
        elif n <= 1:
            self._dup_warned = False

    # ── 입력 두 경로 → 공통 처리 ────────────────────────────────────
    def cb_compressed(self, msg):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"압축 디코딩 실패: {e}")
            return
        if img is None:
            return
        self.process(img, msg.header)

    def cb_raw(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return
        self.process(img, msg.header)

    def process(self, img, header):
        if self.flip == '180':
            img = cv2.rotate(img, cv2.ROTATE_180)
        elif self.flip == 'v':
            img = cv2.flip(img, 0)
        elif self.flip == 'h':
            img = cv2.flip(img, 1)
        # 그 외 값이면 회전 없이 통과(설정 실수 시 영상은 끊기지 않게 — 시작 시 경고만)

        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = header        # 타임스탬프/frame_id 보존 → TF·RViz 시간 동기 유지
        self.pub.publish(out)

        # 압축본(JPEG) 동시 발행 — /camera/image_raw/compressed (대역폭 절약 뷰어용)
        cmsg = CompressedImage()
        cmsg.header = header
        cmsg.format = 'jpeg'
        cmsg.data = cv2.imencode('.jpg', img)[1].tobytes()
        self.pub_c.publish(cmsg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageUpright()
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
