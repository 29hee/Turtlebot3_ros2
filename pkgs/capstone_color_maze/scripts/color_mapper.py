#!/usr/bin/env python3
"""
color_mapper.py
미로를 돌며 '정면 근접에서 본' 색 벽(R/G/B)의 map 좌표를 추정해 색상 시맨틱맵
(color_landmarks.yaml)을 누적 구축한다. 숫자(digit)도 격자별로 함께 투표·저장한다.

[v3] 색 계산을 직접 하지 않는다 — vision_node 가 푼 /detected_color 와 digit_recognizer 의
  /detected_digit 만 구독한다(단일 디코딩: PC CPU 절약). 라이다 정면거리 + TF 로 투영해
  격자 투표한다.

[근접 전용] max_range 를 짧게(기본 0.8m) 둔다 → 멀리서 흐릿하게 본 색을 엉뚱한 칸에
  투영하던 과거 실패를 차단. maze_explorer 가 패널 ~0.3m 까지 접근해 dwell 하는 동안의
  '확실한 근접 관측'만 표로 쌓인다.

격자 투표(grid voting): 투영점을 grid_res 칸으로 스냅 → 칸마다 색·digit 득표 누적 →
  최종은 칸별 최다 득표(총득표 >= min_votes 인 칸만). 한 벽을 여러 번 봐도 한 칸으로 합쳐진다.

입력:
  /detected_color (std_msgs/String)       vision_node 우세색
  /detected_digit (std_msgs/Int32)        digit_recognizer 숫자(-1=없음)
  /scan           (sensor_msgs/LaserScan) 정면 거리
  TF  map -> base_link                     로봇 위치/자세
출력:
  maps/color_landmarks.yaml   {RED:[{x,y,votes,digit}], ...}  (색+숫자 둘 다 인식된 칸만)
  /color_landmarks            (visualization_msgs/MarkerArray)  RViz 시각화

전제: TF 'map' 프레임(slam_toolbox 또는 AMCL). vision_node 가 /detected_color 를 발행 중이어야 함.
실행: python3 color_mapper.py --ros-args -p use_sim_time:=true
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import yaml

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from maze_common import VALID_COLORS, normalize_color


def default_landmarks_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


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

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_range', 0.12)      # 유효 정면거리 하한 [m]
        # ★ 근접 전용 상한. maze_explorer 가 ~0.3m 까지 접근하므로 0.8 이면 충분.
        #   크게 두면 멀리서 본 색이 엉뚱한 칸에 투영돼 맵이 더러워진다(과거 실패).
        self.declare_parameter('max_range', 0.8)
        self.declare_parameter('grid_res', 0.30)       # 격자 한 변 [m]
        self.declare_parameter('min_votes', 5)         # 이 득표 이상인 칸만 채택
        self.declare_parameter('save_path', default_landmarks_path())
        self.declare_parameter('save_period', 3.0)
        # 2-pass 매핑: Phase1 은 '색 좌표만' 저장(require_digit=false), Phase2 가 정면 방문해
        #   숫자를 채운다. require_digit=true(기본)면 색+숫자 둘 다 있는 칸만 저장(단일패스 호환).
        self.declare_parameter('require_digit', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.grid_res = float(self.get_parameter('grid_res').value)
        self.min_votes = int(self.get_parameter('min_votes').value)
        self.save_path = self.get_parameter('save_path').value
        self.save_period = float(self.get_parameter('save_period').value)
        self.require_digit = bool(self.get_parameter('require_digit').value)
        self._dropped_nodigit = 0   # 숫자 미상으로 보류된 칸 수(저장 시 경고용)

        self.scan = None
        self._latest_digit = -1
        self.votes = {}        # {(gx,gy): {'RED':n,...}}
        self.digit_votes = {}  # {(gx,gy): {digit:count}}
        # 시점방향(법선): 벽에서 '로봇이 본 쪽'을 누적 → Phase2 가 그 면에서 정면 접근.
        #   (중앙 박스는 면이 바깥을 향하므로 중심쪽 접근이 틀림 → 본 방향을 따라야 함.)
        self.normals = {}      # {(gx,gy): [sum_nx, sum_ny]}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/detected_color', self.color_cb, 10)
        self.create_subscription(Int32, '/detected_digit', self.digit_cb, 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/color_landmarks', 10)
        self.create_timer(self.save_period, self.save_cb)

        self.get_logger().info(
            f"color_mapper(v3 topic-driven) 시작 — 근접 max_range={self.max_range}m, "
            f"grid_res={self.grid_res}m, min_votes={self.min_votes}, "
            f"색+숫자 둘 다 필수, 저장:{self.save_path}")

    # ──────────────────────────────────────────────────────────────
    def scan_cb(self, msg):
        self.scan = msg

    def digit_cb(self, msg):
        self._latest_digit = int(msg.data)

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

    def color_cb(self, msg):
        """vision_node 우세색 수신 → 근접·TF 유효 시 격자 투표."""
        color = normalize_color(msg.data)
        if color is None:                 # 'NONE' 등
            return
        d = self.front_range()
        if d is None or not (self.min_range <= d <= self.max_range):
            return                        # 근접 게이트: 멀면 기록 안 함
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
        # 시점방향(벽→로봇) 단위벡터 = -(전방벡터)/d. Phase2 가 이 쪽에서 정면 접근.
        nx, ny = (-rx / d, -ry / d) if d > 1e-6 else (0.0, 0.0)
        self.vote(color, t.x + rx, t.y + ry, nx, ny)

    # ── 격자 투표 ─────────────────────────────────────────────────
    def cell_of(self, x, y):
        return (int(math.floor(x / self.grid_res)), int(math.floor(y / self.grid_res)))

    def cell_center(self, gx, gy):
        return ((gx + 0.5) * self.grid_res, (gy + 0.5) * self.grid_res)

    def vote(self, color, x, y, nx=0.0, ny=0.0):
        key = self.cell_of(x, y)
        cell = self.votes.setdefault(key, {c: 0 for c in VALID_COLORS})
        cell[color] += 1
        nrm = self.normals.setdefault(key, [0.0, 0.0])
        nrm[0] += nx
        nrm[1] += ny
        if self._latest_digit >= 0:
            dcell = self.digit_votes.setdefault(key, {})
            dcell[self._latest_digit] = dcell.get(self._latest_digit, 0) + 1
        self.publish_markers()

    def cell_normal(self, gx, gy):
        """칸의 평균 시점방향(단위벡터). 없으면 (0,0)."""
        nrm = self.normals.get((gx, gy))
        if not nrm:
            return 0.0, 0.0
        mag = math.hypot(nrm[0], nrm[1])
        return (nrm[0] / mag, nrm[1] / mag) if mag > 1e-6 else (0.0, 0.0)

    def finalized(self):
        """채택된 칸만: [(color, cx, cy, votes, digit, nx, ny), ...].
        require_digit=true(단일패스): 색+숫자 둘 다인 칸만. false(2-pass Phase1): 색만이라도.
        nx,ny = 평균 시점방향(Phase2 정면접근용)."""
        out = []
        self._dropped_nodigit = 0
        for (gx, gy), cnt in self.votes.items():
            total = sum(cnt.values())
            if total < self.min_votes:
                continue
            color = max(cnt, key=cnt.get)
            cx, cy = self.cell_center(gx, gy)
            dcell = self.digit_votes.get((gx, gy), {})
            digit = max(dcell, key=dcell.get) if dcell else None
            if digit is None:
                self._dropped_nodigit += 1     # 색만 잡힘
                if self.require_digit:
                    continue                   # 단일패스: 색+숫자 둘 다라야 저장
                # 2-pass Phase1: 색만이라도 좌표 저장(Phase2 가 정면 방문해 숫자 채움)
            nx, ny = self.cell_normal(gx, gy)
            out.append((color, cx, cy, cnt[color], digit, nx, ny))
        return out

    # ── 출력 ──────────────────────────────────────────────────────
    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        for color, cx, cy, _, _, _, _ in self.finalized():
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
        if self.require_digit and self._dropped_nodigit:
            self.get_logger().warn(
                f"숫자 미상으로 {self._dropped_nodigit}칸 보류 — "
                f"색+숫자 둘 다 인식돼야 저장됨. 해당 패널 재접근해 숫자를 읽힐 것")
        if not fin:
            return
        data = {c: [] for c in VALID_COLORS}
        for color, cx, cy, v, digit, nx, ny in fin:
            entry = {'x': round(cx, 3), 'y': round(cy, 3), 'votes': v}
            if digit is not None:
                entry['digit'] = digit
            if nx or ny:                       # 시점방향(Phase2 정면접근용)
                entry['nx'] = round(nx, 3)
                entry['ny'] = round(ny, 3)
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
