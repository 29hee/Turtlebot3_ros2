#!/usr/bin/env python3
"""
color_mapper.py
TurtleBot3(burger_cam) 로 미로를 돌며, 정면에서 본 색 벽(R/G/B)의 map 좌표를 추정해
색상 시맨틱맵(color_landmarks.yaml)을 누적 구축한다.  [Phase 2 - 2단계]

[v2] 중복 방지: 자유점 클러스터링 대신 '격자 투표(grid voting)' 사용.
  - 투영점을 grid_res 격자로 스냅 → 같은 칸이면 같은 벽으로 간주
  - 칸마다 색별 득표 누적 → 최종은 칸별 '최다 득표' 색, 단 총득표 >= min_votes 인 칸만
  - 한 벽을 여러 번/여러 각도로 봐도 같은 칸으로 합쳐지고, 1~2회 노이즈는 탈락한다.

입력:
  /camera/image_raw   (sensor_msgs/Image)     정면 색 판정
  /scan               (sensor_msgs/LaserScan)  정면 거리
  TF  map -> base_link                         로봇 위치/자세

출력:
  maps/color_landmarks.yaml   {RED:[{x,y,votes}], GREEN:[...], BLUE:[...]}
  /color_landmarks            (visualization_msgs/MarkerArray)  RViz 시각화

전제: TF 'map' 프레임 필요 → mapping.launch.py(slam_toolbox) 또는 AMCL 가동 중이어야 함.

실행(패키지화 전):
    source /opt/ros/humble/setup.bash
    source /home/user/Workspace/turtlebot3_ws/install/setup.bash
    python3 color_mapper.py
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import cv2
import yaml
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from maze_common import COLOR_RANGES   # HSV 색 범위 단일 출처


def default_landmarks_path():
    """이 스크립트 기준 ../maps/color_landmarks.yaml (하드코딩 경로 제거)."""
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


# ── 마커 색 (HSV 색 범위 COLOR_RANGES 는 maze_common 에서 import) ─────────────
MARKER_RGB = {'RED': (1.0, 0.0, 0.0), 'GREEN': (0.0, 1.0, 0.0), 'BLUE': (0.0, 0.3, 1.0)}


def quat_rotate(q, v):
    """쿼터니언 q=(x,y,z,w) 로 벡터 v 회전 (의존성 없이 직접 계산)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return rx, ry, rz


class ColorMapper(Node):
    def __init__(self):
        super().__init__('color_mapper')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('roi_ratio', 0.4)       # 중앙 ROI 비율
        # 검출 인정 ROI 색 비율. 1.0m 패널은 2.5m 거리에서 ROI의 ~9%로 떨어지므로, 방
        # 중앙(0,0)에서 2.5m 떨어진 벽까지 잡으려면 0.10 은 너무 빡빡 → 0.07 로 완화.
        # (grid-voting + min_votes + maze_common 클러스터 필터가 노이즈를 거르므로 안전.)
        self.declare_parameter('min_ratio', 0.07)
        self.declare_parameter('min_range', 0.12)      # 유효 정면거리 하한 [m]
        # 유효 정면거리 상한 [m]. 방이 5×5라 중앙에서 벽까지 2.5m → max_range 가 2.0 이면
        # 중앙/내부 스핀으로 어느 벽도 못 잡는다. 2.6 으로 올려 한 번의 스핀에서 마주보는
        # 벽을 거리와 무관하게 기록(라이다는 3.5m까지 유효). min_ratio 완화와 짝을 이룬다.
        self.declare_parameter('max_range', 2.6)
        self.declare_parameter('grid_res', 0.30)       # 격자 한 변 [m] (스냅 단위)
        self.declare_parameter('min_votes', 5)         # 이 득표 이상인 칸만 최종 채택
        self.declare_parameter('save_path', default_landmarks_path())
        self.declare_parameter('save_period', 3.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.roi_ratio = float(self.get_parameter('roi_ratio').value)
        self.min_ratio = float(self.get_parameter('min_ratio').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.grid_res = float(self.get_parameter('grid_res').value)
        self.min_votes = int(self.get_parameter('min_votes').value)
        self.save_path = self.get_parameter('save_path').value
        self.save_period = float(self.get_parameter('save_period').value)

        # ── 상태 ──────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.scan = None
        # 격자 투표: {(gx,gy): {'RED':n,'GREEN':n,'BLUE':n}}  gx,gy 는 칸 인덱스(int)
        self.votes = {}
        # 격자별 digit 투표: {(gx,gy): {digit: count}}
        self.digit_votes = {}
        self._latest_digit = -1   # /detected_digit 최신값

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # I/O
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, '/detected_digit', self._digit_cb, 10)
        self.pub_color = self.create_publisher(String, '/detected_color', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/color_landmarks', 10)
        self.create_timer(self.save_period, self.save_cb)

        self.get_logger().info(
            f"color_mapper(v2 grid-voting) 시작 — grid_res={self.grid_res}m, "
            f"min_votes={self.min_votes}, 저장:{self.save_path}")

    # ──────────────────────────────────────────────────────────────
    def _digit_cb(self, msg):
        self._latest_digit = int(msg.data)

    def scan_cb(self, msg):
        self.scan = msg

    def front_range(self):
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return None
        n = len(s.ranges)
        i0 = int(round((0.0 - s.angle_min) / s.angle_increment)) % n
        win = max(1, int(math.radians(5) / s.angle_increment))
        vals = []
        for k in range(-win, win + 1):
            r = s.ranges[(i0 + k) % n]
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                vals.append(r)
        return float(np.median(vals)) if vals else None

    def dominant_color(self, frame):
        h, w = frame.shape[:2]
        rw, rh = int(w * self.roi_ratio), int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        hsv = cv2.cvtColor(frame[y1:y1 + rh, x1:x1 + rw], cv2.COLOR_BGR2HSV)
        area = max(1, rw * rh)
        kernel = np.ones((3, 3), np.uint8)
        ratios = {}
        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            ratios[color] = int(cv2.countNonZero(mask)) / area
        best = max(ratios, key=ratios.get)
        return (best, ratios[best]) if ratios[best] >= self.min_ratio else ('NONE', ratios[best])

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 변환 실패: {e}")
            return

        color, _ = self.dominant_color(frame)
        self.pub_color.publish(String(data=color))
        if color == 'NONE':
            return

        d = self.front_range()
        if d is None or not (self.min_range <= d <= self.max_range):
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return

        t = tf.transform.translation
        q = (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w)
        rx, ry, _ = quat_rotate(q, (d, 0.0, 0.0))
        self.vote(color, t.x + rx, t.y + ry)

    # ── 격자 투표 ─────────────────────────────────────────────────
    def cell_of(self, x, y):
        return (int(math.floor(x / self.grid_res)), int(math.floor(y / self.grid_res)))

    def cell_center(self, gx, gy):
        return ((gx + 0.5) * self.grid_res, (gy + 0.5) * self.grid_res)

    def vote(self, color, x, y):
        key = self.cell_of(x, y)
        cell = self.votes.setdefault(key, {c: 0 for c in COLOR_RANGES})
        cell[color] += 1
        if self._latest_digit >= 0:
            dcell = self.digit_votes.setdefault(key, {})
            dcell[self._latest_digit] = dcell.get(self._latest_digit, 0) + 1
        self.publish_markers()

    def finalized(self):
        """채택된 칸만: [(color, cx, cy, votes, digit_or_None), ...]"""
        out = []
        for (gx, gy), cnt in self.votes.items():
            total = sum(cnt.values())
            if total < self.min_votes:
                continue
            color = max(cnt, key=cnt.get)
            cx, cy = self.cell_center(gx, gy)
            dcell = self.digit_votes.get((gx, gy), {})
            digit = max(dcell, key=dcell.get) if dcell else None
            out.append((color, cx, cy, cnt[color], digit))
        return out

    # ── 출력 ──────────────────────────────────────────────────────
    def publish_markers(self):
        arr = MarkerArray()
        # 이전 마커 전체 삭제 후 다시 그림(채택 칸만 보이도록)
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        for color, cx, cy, _votes, _digit in self.finalized():
            r, g, b = MARKER_RGB[color]
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = color
            m.id = mid
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.grid_res * 0.9
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
            arr.markers.append(m)
            mid += 1
        self.pub_marker.publish(arr)

    def save_cb(self):
        fin = self.finalized()
        if not fin:
            return
        data = {c: [] for c in COLOR_RANGES}
        for color, cx, cy, v, digit in fin:
            entry = {'x': round(cx, 3), 'y': round(cy, 3), 'votes': v}
            if digit is not None:
                entry['digit'] = digit
            data[color].append(entry)
        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, 'w') as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        except Exception as e:
            self.get_logger().warn(f"저장 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_cb()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
