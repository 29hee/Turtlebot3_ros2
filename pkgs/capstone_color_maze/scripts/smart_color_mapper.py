#!/usr/bin/env python3
"""
smart_color_mapper.py
벽 따라 주행하다 색 후보 감지 → 정면으로 돌아 확인 → 기록 → 돌아와 계속 주행.

2단계 임계:
  후보(candidate_ratio=0.03): 전체 프레임에서 '혹시?' 감지
  확인(confirm_ratio=0.10):   정면 ROI에서 '맞다!' 기록

저장: maps/color_landmarks.yaml (color_mapper 와 동일 포맷, digit 포함)

실행:
  python3 scripts/smart_color_mapper.py
  python3 scripts/smart_color_mapper.py --ros-args -p show:=true -p rotate_180:=true
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
import tf2_ros

from maze_common import COLOR_RANGES

_ST_FOLLOW = 'FOLLOW'
_ST_TURN_FACE = 'TURNING_TO_FACE'
_ST_CONFIRM = 'CONFIRMING'
_ST_TURN_BACK = 'TURNING_BACK'

MARKER_RGB = {'RED': (1.0, 0.0, 0.0), 'GREEN': (0.0, 1.0, 0.0), 'BLUE': (0.0, 0.3, 1.0)}


def _yaw_from_tf(tf):
    q = tf.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def _quat_rotate(q, v):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class SmartColorMapper(Node):
    def __init__(self):
        super().__init__('smart_color_mapper')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # 카메라 수평 화각 [rad]. burger_cam ≈ 182° = 3.18. 시뮬/실물 다를 수 있으니 파라미터로.
        self.declare_parameter('hfov', 1.745)            # 기본 100°
        self.declare_parameter('candidate_ratio', 0.03)  # 전체 프레임 낮은 임계
        self.declare_parameter('confirm_ratio', 0.10)    # 중앙 ROI 높은 임계
        self.declare_parameter('roi_ratio', 0.4)         # 확인용 중앙 ROI 크기 비율
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 2.6)
        self.declare_parameter('grid_res', 0.30)
        self.declare_parameter('min_votes', 5)
        self.declare_parameter('spin_speed', 0.3)        # 느리게! SLAM 스캔매칭 안 깨지게
        self.declare_parameter('confirm_frames', 3)      # 연속 OK 프레임 수
        self.declare_parameter('confirm_timeout', 5.0)   # 확인 최대 대기 [s]
        self.declare_parameter('cooldown_secs', 6.0)     # 기록 후 재검출 억제 [s]
        self.declare_parameter('rotate_180', False)
        self.declare_parameter('show', False)
        self.declare_parameter('save_period', 3.0)
        self.declare_parameter('save_path', self._default_save_path())

        def p(name):
            return self.get_parameter(name).value

        self.image_topic    = p('image_topic')
        self.map_frame      = p('map_frame')
        self.base_frame     = p('base_frame')
        self.hfov           = float(p('hfov'))
        self.candidate_ratio = float(p('candidate_ratio'))
        self.confirm_ratio  = float(p('confirm_ratio'))
        self.roi_ratio      = float(p('roi_ratio'))
        self.min_range      = float(p('min_range'))
        self.max_range      = float(p('max_range'))
        self.grid_res       = float(p('grid_res'))
        self.min_votes      = int(p('min_votes'))
        self.spin_speed     = float(p('spin_speed'))
        self.confirm_frames = int(p('confirm_frames'))
        self.confirm_timeout = float(p('confirm_timeout'))
        self.cooldown_secs  = float(p('cooldown_secs'))
        self.rotate_180     = bool(p('rotate_180'))
        self.show           = bool(p('show'))
        self.save_path      = p('save_path')

        # ── 벽타기 파라미터 (wall_follower 와 동일) ──────────────
        self.target_right = 0.45
        self.front_stop   = 0.45
        self.lost_right   = 0.9
        self.v_fwd        = 0.15
        self.kp           = 1.6

        # ── 상태 ──────────────────────────────────────────────────
        self.state          = _ST_FOLLOW
        self.candidate_color = None
        self.blob_angle     = 0.0   # 감지 시점 블롭 각도 [rad], + = 오른쪽
        self.back_yaw       = None  # 복귀 목표 yaw
        self.face_yaw_target = None # 정면 회전 목표 yaw
        self.confirm_count  = 0
        self.confirm_start  = 0.0
        self._last_record_t = 0.0

        # ── 센서 데이터 ───────────────────────────────────────────
        self.scan          = None
        self.bridge        = CvBridge()
        self._frame        = None
        self._latest_digit = -1

        # ── 격자 투표 ──────────────────────────────────────────────
        self.votes       = {}   # {(gx,gy): {color: count}}
        self.digit_votes = {}   # {(gx,gy): {digit: count}}

        # ── TF ────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── ROS I/O ───────────────────────────────────────────────
        self.pub_vel    = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/color_landmarks', 10)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, '/detected_digit', self._digit_cb, 10)

        self.create_timer(0.1, self._loop)
        self.create_timer(float(p('save_period')), self._save_cb)

        self.get_logger().info(
            f'smart_color_mapper 시작 — '
            f'candidate={self.candidate_ratio:.2f} confirm={self.confirm_ratio:.2f} '
            f'spin={self.spin_speed:.2f}rad/s hfov={math.degrees(self.hfov):.0f}° '
            f'저장:{self.save_path}')

    # ──────────────────────────────────────────────────────────────
    # 유틸
    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _default_save_path():
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')

    # ── 센서 콜백 ─────────────────────────────────────────────────
    def _scan_cb(self, msg):
        self.scan = msg

    def _digit_cb(self, msg):
        self._latest_digit = int(msg.data)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 실패: {e}')
            return
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        self._frame = frame

    # ── LiDAR 헬퍼 ───────────────────────────────────────────────
    def _sector_min(self, deg_lo, deg_hi):
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return float('inf')
        n = len(s.ranges)
        vals = []
        d = deg_lo
        while d <= deg_hi:
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and not math.isinf(r) and not math.isnan(r) and s.range_min < r < s.range_max:
                vals.append(r)
            d += 1
        return min(vals) if vals else float('inf')

    def _front_range(self):
        s = self.scan
        if s is None or len(s.ranges) == 0:
            return None
        n = len(s.ranges)
        i0 = int(round(-s.angle_min / s.angle_increment)) % n
        win = max(1, int(math.radians(5) / s.angle_increment))
        vals = [s.ranges[(i0 + k) % n] for k in range(-win, win + 1)
                if math.isfinite(s.ranges[(i0 + k) % n])
                and s.range_min <= s.ranges[(i0 + k) % n] <= s.range_max]
        return float(np.median(vals)) if vals else None

    # ── 색 검출 ───────────────────────────────────────────────────
    def _detect_candidate(self, frame):
        """전체 프레임. 가장 비율 높은 색, 그 비율, 블롭 중심 각도 반환."""
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        area = max(1, h * w)
        kernel = np.ones((3, 3), np.uint8)
        best_color, best_ratio, best_angle = 'NONE', 0.0, 0.0
        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            ratio = cv2.countNonZero(mask) / area
            if ratio > best_ratio:
                best_ratio = ratio
                best_color = color
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx_px = M['m10'] / M['m00']
                    # + = 블롭이 오른쪽 → 시계방향(yaw 감소)으로 회전해야 정면
                    best_angle = (cx_px - w / 2) / w * self.hfov
        if best_ratio < self.candidate_ratio:
            return 'NONE', best_ratio, 0.0
        return best_color, best_ratio, best_angle

    def _confirm_ratio_roi(self, frame):
        """중앙 ROI에서 candidate_color 의 HSV 마스크 비율."""
        if self.candidate_color is None:
            return 0.0
        h, w = frame.shape[:2]
        rw, rh = int(w * self.roi_ratio), int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        roi = frame[y1:y1 + rh, x1:x1 + rw]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        area = max(1, rw * rh)
        kernel = np.ones((3, 3), np.uint8)
        mask = None
        for lo, hi in COLOR_RANGES.get(self.candidate_color, []):
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if mask is None:
            return 0.0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.countNonZero(mask) / area

    # ── TF ────────────────────────────────────────────────────────
    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None

    def _get_yaw(self):
        tf = self._get_tf()
        return _yaw_from_tf(tf) if tf else None

    # ── 격자 투표 ─────────────────────────────────────────────────
    def _cell_of(self, x, y):
        return (int(math.floor(x / self.grid_res)), int(math.floor(y / self.grid_res)))

    def _cell_center(self, gx, gy):
        return ((gx + 0.5) * self.grid_res, (gy + 0.5) * self.grid_res)

    def _vote(self, color, x, y):
        key = self._cell_of(x, y)
        cell = self.votes.setdefault(key, {c: 0 for c in COLOR_RANGES})
        cell[color] += 1
        if self._latest_digit >= 0:
            dc = self.digit_votes.setdefault(key, {})
            dc[self._latest_digit] = dc.get(self._latest_digit, 0) + 1
        self._publish_markers()

    def _record_wall(self, n=1):
        """현재 정면 LiDAR + TF 로 벽 위치 투표 n회. 성공하면 True."""
        d = self._front_range()
        if d is None or not (self.min_range <= d <= self.max_range):
            return False
        tf = self._get_tf()
        if tf is None:
            return False
        t = tf.transform.translation
        rx, ry, _ = _quat_rotate(tf.transform.rotation, (d, 0.0, 0.0))
        wx, wy = t.x + rx, t.y + ry
        for _ in range(n):
            self._vote(self.candidate_color, wx, wy)
        self.get_logger().info(
            f'[기록] {self.candidate_color} @ ({wx:.2f},{wy:.2f}) '
            f'd={d:.2f}m votes+={n} digit={self._latest_digit}')
        return True

    # ── 마커 발행 ─────────────────────────────────────────────────
    def _publish_markers(self):
        arr = MarkerArray()
        clr = Marker()
        clr.header.frame_id = self.map_frame
        clr.action = Marker.DELETEALL
        arr.markers.append(clr)
        mid = 0
        for (gx, gy), cnt in self.votes.items():
            if sum(cnt.values()) < self.min_votes:
                continue
            color = max(cnt, key=cnt.get)
            cx, cy = self._cell_center(gx, gy)
            r, g, b = MARKER_RGB[color]
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = color; m.id = mid; m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.grid_res * 0.9; m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
            arr.markers.append(m)
            mid += 1
        self.pub_marker.publish(arr)

    # ── YAML 저장 ─────────────────────────────────────────────────
    def _save_cb(self):
        out = {c: [] for c in COLOR_RANGES}
        for (gx, gy), cnt in self.votes.items():
            if sum(cnt.values()) < self.min_votes:
                continue
            color = max(cnt, key=cnt.get)
            cx, cy = self._cell_center(gx, gy)
            dc = self.digit_votes.get((gx, gy), {})
            digit = max(dc, key=dc.get) if dc else None
            entry = {'x': round(cx, 3), 'y': round(cy, 3), 'votes': cnt[color]}
            if digit is not None:
                entry['digit'] = digit
            out[color].append(entry)
        if not any(out.values()):
            return
        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            with open(self.save_path, 'w') as f:
                yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
        except Exception as e:
            self.get_logger().warn(f'저장 실패: {e}')

    # ── 벽타기 명령 ───────────────────────────────────────────────
    def _wall_follow_cmd(self):
        front = self._sector_min(-20, 20)
        right = self._sector_min(-100, -80)
        cmd = Twist()
        if front < self.front_stop:
            cmd.angular.z = 0.7
        elif right > self.lost_right:
            cmd.linear.x = 0.12
            cmd.angular.z = -0.6
        else:
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.8, min(0.8, -self.kp * err))
        return cmd

    # ── 메인 제어 루프 (10 Hz) ────────────────────────────────────
    def _loop(self):
        frame = self._frame

        # ── FOLLOW ─────────────────────────────────────────────────
        if self.state == _ST_FOLLOW:
            if self.scan is None:
                return
            self.pub_vel.publish(self._wall_follow_cmd())

            if time.time() - self._last_record_t < self.cooldown_secs:
                return   # 쿨다운: 막 기록한 벽을 다시 잡지 않도록
            if frame is None:
                return

            color, ratio, blob_angle = self._detect_candidate(frame)
            if color == 'NONE':
                return

            yaw = self._get_yaw()
            if yaw is None:
                return

            self.candidate_color = color
            self.blob_angle = blob_angle
            self.back_yaw = yaw
            # 블롭이 오른쪽(+blob_angle) → yaw를 blob_angle만큼 감소(시계방향)시켜야 정면
            self.face_yaw_target = _wrap(yaw - blob_angle)
            self.pub_vel.publish(Twist())   # 정지
            self.state = _ST_TURN_FACE
            self.get_logger().info(
                f'[후보] {color} ratio={ratio:.3f} '
                f'blob={math.degrees(blob_angle):+.1f}° '
                f'yaw {math.degrees(yaw):.1f}° → {math.degrees(self.face_yaw_target):.1f}°')

        # ── TURNING_TO_FACE ────────────────────────────────────────
        elif self.state == _ST_TURN_FACE:
            yaw = self._get_yaw()
            if yaw is None:
                return
            err = _wrap(self.face_yaw_target - yaw)
            if abs(err) < 0.05:
                self.pub_vel.publish(Twist())
                self.confirm_count = 0
                self.confirm_start = time.time()
                self.state = _ST_CONFIRM
                self.get_logger().info(f'[정면] → 확인 중 ({self.candidate_color})')
            else:
                cmd = Twist()
                # proportional: 빠르게 돌리되 spin_speed 상한
                cmd.angular.z = max(-self.spin_speed, min(self.spin_speed, 1.5 * err))
                self.pub_vel.publish(cmd)

        # ── CONFIRMING ─────────────────────────────────────────────
        elif self.state == _ST_CONFIRM:
            if frame is None:
                return
            ratio = self._confirm_ratio_roi(frame)
            timed_out = (time.time() - self.confirm_start) > self.confirm_timeout

            if ratio >= self.confirm_ratio:
                self.confirm_count += 1
                if self.confirm_count >= self.confirm_frames:
                    ok = self._record_wall(n=self.min_votes)
                    self._last_record_t = time.time()
                    if not ok:
                        self.get_logger().warn(
                            f'[확인] {self.candidate_color} TF/LiDAR 없음 — 기록 실패')
                    self.state = _ST_TURN_BACK
            else:
                self.confirm_count = max(0, self.confirm_count - 1)
                if timed_out:
                    self.get_logger().info(
                        f'[포기] {self.candidate_color} 확인 시간 초과(ratio={ratio:.3f}) → 복귀')
                    self.state = _ST_TURN_BACK

        # ── TURNING_BACK ───────────────────────────────────────────
        elif self.state == _ST_TURN_BACK:
            yaw = self._get_yaw()
            if yaw is None:
                return
            err = _wrap(self.back_yaw - yaw)
            if abs(err) < 0.05:
                self.pub_vel.publish(Twist())
                self.state = _ST_FOLLOW
                self.candidate_color = None
                self.get_logger().info('[복귀] 벽 따라가기 재개')
            else:
                cmd = Twist()
                cmd.angular.z = max(-self.spin_speed, min(self.spin_speed, 1.5 * err))
                self.pub_vel.publish(cmd)

        # ── 디버그 창 ──────────────────────────────────────────────
        if self.show and frame is not None:
            self._show_debug(frame)

    def _show_debug(self, frame):
        disp = frame.copy()
        h, w = frame.shape[:2]
        rw = int(w * self.roi_ratio)
        rh = int(h * self.roi_ratio)
        x1, y1 = (w - rw) // 2, (h - rh) // 2
        cv2.rectangle(disp, (x1, y1), (x1 + rw, y1 + rh), (255, 255, 0), 2)
        label = f'[{self.state}] {self.candidate_color or "-"} digit={self._latest_digit}'
        cv2.putText(disp, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.imshow('smart_color_mapper', disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SmartColorMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._save_cb()
        node.pub_vel.publish(Twist())
        if node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
