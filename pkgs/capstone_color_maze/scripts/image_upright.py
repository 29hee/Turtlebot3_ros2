#!/usr/bin/env python3
"""
image_upright.py
거꾸로 장착된 버거 카메라를 '소스에서 한 번' 회전해 똑바로 세워 재발행한다.
이 노드 하나만 거치면 RViz·OCR(color_detector)·rqt·기타 모든 구독자가 정상 방향
영상을 본다(각 노드가 따로 뒤집을 필요 없음 → 이중 회전/누락 footgun 제거).

토픽 배선(표준 토픽을 '똑바로 선' 영상으로 채우는 게 핵심):
    입력(원본, 거꾸로):  in_topic   기본 /camera/image_raw_rot   (v4l2 가 로봇에서 발행)
    출력(똑바로):        out_topic  기본 /camera/image_raw        ← 기존 파이프라인 표준 토픽
    출력(압축):          out_topic + '/compressed'                ← WiFi 경량 뷰어용(정방향)

이 노드는 순수 토픽 릴레이라 PC 에서 돌린다(로봇 Pi 에 OpenCV/cv_bridge·repo 불필요, Pi CPU 절약).
카메라 기동은 다음처럼: 로봇은 원본을 _rot 으로 빼고, PC 의 이 노드가 표준 토픽을 채움.
    (로봇) ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/camera/image_raw_rot
    (PC)   python3 scripts/image_upright.py

회전 방식 파라미터 flip:
    180  : 180° 회전(상하+좌우, 기본) — 카메라가 천장을 보도록 뒤집힌 일반적 경우
    v    : 상하만(수직 미러)
    h    : 좌우만(수평 미러)

실행:
    source /opt/ros/humble/setup.bash
    python3 image_upright.py
    # 예: 좌우만 뒤집힌 장착이면  python3 image_upright.py --ros-args -p flip:=h
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


class ImageUpright(Node):
    def __init__(self):
        super().__init__('image_upright')

        self.declare_parameter('in_topic', '/camera/image_raw_rot')   # 원본(거꾸로)
        self.declare_parameter('out_topic', '/camera/image_raw')      # 보정본(표준 토픽)
        self.declare_parameter('flip', '180')                         # 180 | v | h

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.flip = str(self.get_parameter('flip').value).lower()
        if self.flip not in ('180', 'v', 'h'):
            # 잘못된 값이면 회전 없이 통과 → 거꾸로인 채 조용히 흘러갈 수 있어 한 번 경고.
            self.get_logger().warn(
                f"알 수 없는 flip='{self.flip}' → 회전 없이 원본 그대로 통과. 180|v|h 중 하나로 줄 것.")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        # 똑바로 선 '압축' 영상도 함께 발행 → WiFi 에서 RViz/rqt 가 정방향 영상을 가볍게 확인.
        self.pub_c = self.create_publisher(
            CompressedImage, out_topic + '/compressed', qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Image, in_topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"image_upright 시작 — {in_topic} → {out_topic} (flip={self.flip})")

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        if self.flip == '180':
            img = cv2.rotate(img, cv2.ROTATE_180)
        elif self.flip == 'v':
            img = cv2.flip(img, 0)
        elif self.flip == 'h':
            img = cv2.flip(img, 1)
        # 그 외 값이면 회전 없이 통과(설정 실수 시 영상은 끊기지 않게)

        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = msg.header        # 타임스탬프/frame_id 보존 → TF·RViz 시간 동기 유지
        self.pub.publish(out)

        # 압축본(JPEG) 동시 발행 — /camera/image_raw/compressed (대역폭 절약 뷰어용)
        cmsg = CompressedImage()
        cmsg.header = msg.header
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
