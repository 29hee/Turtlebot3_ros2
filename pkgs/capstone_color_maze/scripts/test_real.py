#!/usr/bin/env python3
"""
test_real.py — 사무실 패널 약식 테스트 (Nav2 불필요)

두 단계를 별도로 실행:

  [1단계: SLAM 구축]
    python3 test_real.py --ros-args -p mode:=slam
    → 벽타기 주행 (color 무시). Ctrl+C 로 종료.
    → slam_toolbox 맵이 안정화될 때까지 돌린다.

  [2단계: 색 감지 + 추종]
    python3 test_real.py --ros-args -p mode:=color
    → 제자리 360° 스핀 → 색+digit 기록 → YAML 저장 → 결과 출력
    → 터미널에서 "RED 1" / "GREEN 2" 입력 → 해당 패널 앞으로 이동

전제:
  slam_toolbox 실행 중 (TF map→base_link 필요)
  /camera/image_raw, /scan 퍼블리시 중
  (선택) digit_test.py 실행 시 /detected_digit 연동
"""
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32, String
import tf2_ros

from maze_common import COLOR_RANGES, parse_target, cluster_cells, filter_clusters

# ── 상태 ──────────────────────────────────────────────────────────────────────
_ST_SEEK    = 'SEEK_WALL'   # 직진해서 벽 찾기
_ST_SLAM    = 'SLAM'        # 벽타기 (SLAM 구축용)
_ST_SPIN    = 'SPIN'        # 360° 스핀 (색 감지)
_ST_IDLE    = 'IDLE'        # 완료 대기 / 입력 대기
_ST_TURN    = 'TURN'        # 목표 방향으로 회전
_ST_DRIVE   = 'DRIVE'       # 목표로 직진
_ST_ARRIVED = 'ARRIVED'     # 도착

_DRAW_BGR = {'RED': (0,0,255), 'GREEN': (0,200,0), 'BLUE': (255,80,0)}


def _quat_rotate(q, v):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    tx = 2*(y*vz - z*vy); ty = 2*(z*vx - x*vz); tz = 2*(x*vy - y*vx)
    return (vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx))


def _yaw_from_tf(tf):
    q = tf.transform.rotation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))


def _wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


class TestReal(Node):
    def __init__(self):
        super().__init__('test_real')

        self.declare_parameter('mode',        'slam')   # 'slam' | 'color'
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('spin_speed',  0.20)     # rad/s
        self.declare_parameter('min_range',   0.15)
        self.declare_parameter('max_range',   3.00)
        self.declare_parameter('min_ratio',   0.04)   # Gazebo 렌더링용 낮춤
        self.declare_parameter('roi_ratio',   0.50)
        self.declare_parameter('grid_res',    0.30)
        self.declare_parameter('standoff',    0.60)     # 패널 앞 정지 거리 [m]
        self.declare_parameter('show',        False)
        self.declare_parameter('save_path',   self._default_save_path())

        def p(n): return self.get_parameter(n).value
        self.mode        = str(p('mode')).lower()
        self.image_topic = p('image_topic')
        self.spin_speed  = float(p('spin_speed'))
        self.min_range   = float(p('min_range'))
        self.max_range   = float(p('max_range'))
        self.min_ratio   = float(p('min_ratio'))
        self.roi_ratio   = float(p('roi_ratio'))
        self.grid_res    = float(p('grid_res'))
        self.standoff    = float(p('standoff'))
        self.show        = bool(p('show'))
        self.save_path   = p('save_path')

        # 벽타기 파라미터 (slam 모드)
        self._front_stop   = 0.45
        self._target_right = 0.45
        self._lost_right   = 0.90
        self._v_fwd        = 0.12
        self._kp           = 1.6

        # 공유 상태
        self.scan           = None
        self._frame         = None
        self._latest_digit  = -1
        self.votes          = {}
        self.digit_votes    = {}
        self._spin_acc      = 0.0
        self._spin_prev_yaw = None
        self._spin_start_t  = None
        self._spin_timeout  = (2*math.pi / max(self.spin_speed, 0.05)) * 1.3
        self.target_wx      = None
        self.target_wy      = None

        self.state = _ST_SEEK if self.mode == 'slam' else _ST_SPIN

        self.bridge      = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_vel   = self.create_publisher(Twist,  'cmd_vel',         10)
        self.pub_color = self.create_publisher(String, '/detected_color',  10)
        self.create_subscription(LaserScan, '/scan',           self._scan_cb,  qos_profile_sensor_data)
        self.create_subscription(Image,     self.image_topic,  self._img_cb,   qos_profile_sensor_data)
        self.create_subscription(Int32,     '/detected_digit', self._digit_cb, 10)
        self.create_timer(0.1, self._loop)

        if self.mode == 'slam':
            print('\n[test_real] ── SLAM 모드 ────────────────────────────')
            print('벽 탐색 중... 벽 발견 후 벽타기 시작. Ctrl+C 로 종료.\n')
        else:
            print('\n[test_real] ── COLOR 모드 ───────────────────────────')
            print(f'360° 스핀 시작 (speed={self.spin_speed} rad/s)')
            print('패널을 카메라 시야 안에 배치하세요.\n')

    @staticmethod
    def _default_save_path():
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')

    # ── 콜백 ─────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg): self.scan = msg
    def _digit_cb(self, msg): self._latest_digit = int(msg.data)
    def _img_cb(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    # ── 헬퍼 ─────────────────────────────────────────────────────────────────
    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None

    def _sector_min(self, deg_lo, deg_hi):
        s = self.scan
        if s is None or not s.ranges:
            return float('inf')
        n = len(s.ranges)
        vals = []
        for d in range(int(deg_lo), int(deg_hi)+1):
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                vals.append(r)
        return min(vals) if vals else float('inf')

    def _front_range(self):
        s = self.scan
        if s is None or not s.ranges:
            return None
        n = len(s.ranges)
        i0 = int(round(-s.angle_min / s.angle_increment)) % n
        win = max(1, int(math.radians(5) / s.angle_increment))
        vals = [s.ranges[(i0+k)%n] for k in range(-win, win+1)
                if math.isfinite(s.ranges[(i0+k)%n])
                and s.range_min <= s.ranges[(i0+k)%n] <= s.range_max]
        return float(np.median(vals)) if vals else None

    def _wall_follow_cmd(self):
        front = self._sector_min(-20, 20)
        right = self._sector_min(-100, -80)
        cmd = Twist()
        if front < self._front_stop:
            cmd.angular.z = 0.7
        elif right > self._lost_right:
            cmd.linear.x = 0.10
            cmd.angular.z = -0.5
        else:
            err = right - self._target_right
            cmd.linear.x = self._v_fwd
            cmd.angular.z = max(-0.8, min(0.8, -self._kp * err))
        return cmd

    def _detect_color(self, frame):
        h, w = frame.shape[:2]
        rw = int(w*self.roi_ratio); rh = int(h*self.roi_ratio)
        x1, y1 = (w-rw)//2, (h-rh)//2
        hsv = cv2.cvtColor(frame[y1:y1+rh, x1:x1+rw], cv2.COLOR_BGR2HSV)
        area = max(1, rw*rh)
        kernel = np.ones((3,3), np.uint8)
        ratios = {}
        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            ratios[color] = cv2.countNonZero(mask) / area
        best = max(ratios, key=ratios.get)
        return (best, ratios[best]) if ratios[best] >= self.min_ratio else ('NONE', 0.0)

    def _vote(self, color, x, y):
        key = (int(math.floor(x/self.grid_res)), int(math.floor(y/self.grid_res)))
        cell = self.votes.setdefault(key, {c: 0 for c in COLOR_RANGES})
        cell[color] += 1
        if self._latest_digit >= 0:
            dc = self.digit_votes.setdefault(key, {})
            dc[self._latest_digit] = dc.get(self._latest_digit, 0) + 1

    def _save_and_print(self):
        # 1) 원시 셀 수집
        raw = {c: [] for c in COLOR_RANGES}
        for (gx, gy), cnt in self.votes.items():
            color = max(cnt, key=cnt.get)
            if cnt[color] == 0:
                continue
            cx = (gx+0.5)*self.grid_res; cy = (gy+0.5)*self.grid_res
            dc = self.digit_votes.get((gx, gy), {})
            digit = max(dc, key=dc.get) if dc else None
            entry = {'x': round(cx,3), 'y': round(cy,3), 'votes': cnt[color]}
            if digit is not None:
                entry['digit'] = digit
            raw[color].append(entry)

        # 2) 클러스터링 + 노이즈 필터 (maze_common)
        out = {c: [] for c in COLOR_RANGES}
        for color, cells in raw.items():
            clustered = cluster_cells(cells)
            filtered  = filter_clusters(clustered, frac=0.1, floor=3)
            out[color] = filtered

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        with open(self.save_path, 'w') as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

        print('\n── 색 맵 결과 (클러스터링 후 저장) ─────────')
        any_found = False
        for color, walls in out.items():
            for w in walls:
                d = f" digit={w['digit']}" if 'digit' in w else ''
                print(f'  {color}: ({w["x"]:.2f}, {w["y"]:.2f}) votes={w["votes"]}{d}')
                any_found = True
        if not any_found:
            print('  !! 감지된 색 없음 — 패널 위치/HSV 범위 확인 필요')
        print('─────────────────────────────────────────────')
        return out

    # ── 입력 스레드 ───────────────────────────────────────────────────────────
    def _input_loop(self, data):
        print('\n목표 입력 (예: RED  GREEN 2  BLUE 1)  종료: quit\n')
        while rclpy.ok():
            try:
                line = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if line.lower() in ('quit', 'q', 'exit'):
                self.pub_vel.publish(Twist())
                rclpy.shutdown()
                return
            color, digit = parse_target(line)
            if color is None:
                print('  형식: COLOR [숫자]  예) RED  GREEN 2')
                continue
            walls = data.get(color, [])
            if not walls:
                print(f'  {color} 패널이 맵에 없습니다')
                continue
            if digit is not None:
                match = [w for w in walls if w.get('digit') == digit]
                if not match:
                    print(f'  {color} digit={digit} 없음. 기록된: {[w.get("digit") for w in walls]}')
                    continue
                target = match[0]
            else:
                target = walls[0]
            self.target_wx = target['x']
            self.target_wy = target['y']
            self.state = _ST_TURN
            print(f'  → {color} ({self.target_wx:.2f}, {self.target_wy:.2f}) 이동 시작')
            while self.state != _ST_ARRIVED and rclpy.ok():
                time.sleep(0.2)
            self.state = _ST_IDLE

    # ── 메인 루프 (10 Hz) ─────────────────────────────────────────────────────
    def _loop(self):
        tf = self._get_tf()

        # ─ 벽 찾기: 직진 → 벽 발견 시 SLAM 전환 ─────────────────────────────
        if self.state == _ST_SEEK:
            front = self._front_range()
            if front is not None and front < 0.8:
                self.pub_vel.publish(Twist())
                self.state = _ST_SLAM
                print('[SEEK→SLAM] 벽 발견, 벽타기 시작')
            else:
                cmd = Twist()
                cmd.linear.x = 0.15
                self.pub_vel.publish(cmd)
            return

        # ─ SLAM 모드: 벽타기 ─────────────────────────────────────────────────
        if self.state == _ST_SLAM:
            if self.scan is not None:
                self.pub_vel.publish(self._wall_follow_cmd())
            return

        # ─ COLOR 모드: 360° 스핀 ─────────────────────────────────────────────
        if self.state == _ST_SPIN:
            if self._spin_start_t is None:
                self._spin_start_t = time.time()

            cmd = Twist()
            cmd.angular.z = self.spin_speed
            self.pub_vel.publish(cmd)

            if tf is not None:
                yaw = _yaw_from_tf(tf)
                if self._spin_prev_yaw is not None:
                    self._spin_acc += abs(_wrap(yaw - self._spin_prev_yaw))
                self._spin_prev_yaw = yaw

            frame = self._frame
            if frame is not None and tf is not None:
                color, _ = self._detect_color(frame)
                self.pub_color.publish(String(data=color))
                if color != 'NONE':
                    d = self._front_range()
                    if d and self.min_range <= d <= self.max_range:
                        t = tf.transform.translation
                        rx, ry, _ = _quat_rotate(tf.transform.rotation, (d, 0.0, 0.0))
                        self._vote(color, t.x+rx, t.y+ry)
                if self.show:
                    self._show_debug(frame, color if color != 'NONE' else None)

            elapsed = time.time() - self._spin_start_t
            if self._spin_acc >= 2*math.pi or elapsed >= self._spin_timeout:
                self.pub_vel.publish(Twist())
                data = self._save_and_print()
                self.state = _ST_IDLE
                threading.Thread(target=self._input_loop, args=(data,), daemon=True).start()
            return

        # ─ 목표 방향으로 회전 ─────────────────────────────────────────────────
        if self.state == _ST_TURN:
            if tf is None:
                return
            yaw     = _yaw_from_tf(tf)
            rx, ry  = tf.transform.translation.x, tf.transform.translation.y
            desired = math.atan2(self.target_wy - ry, self.target_wx - rx)
            err     = _wrap(desired - yaw)
            if abs(err) < 0.05:
                self.pub_vel.publish(Twist())
                self.state = _ST_DRIVE
            else:
                cmd = Twist()
                cmd.angular.z = max(-0.4, min(0.4, 2.0 * err))
                self.pub_vel.publish(cmd)
            return

        # ─ 직진 ──────────────────────────────────────────────────────────────
        if self.state == _ST_DRIVE:
            if tf is None:
                return
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            yaw    = _yaw_from_tf(tf)
            dist   = math.hypot(self.target_wx-rx, self.target_wy-ry)
            front  = self._front_range()
            if dist <= self.standoff or (front and front < 0.35):
                self.pub_vel.publish(Twist())
                self.state = _ST_ARRIVED
                msg = f'잔여={dist:.2f}m' + (f'  전방={front:.2f}m' if front else '')
                print(f'\n[도착] {msg}')
            else:
                desired = math.atan2(self.target_wy-ry, self.target_wx-rx)
                err = _wrap(desired - yaw)
                cmd = Twist()
                cmd.linear.x  = min(0.15, max(0.05, 0.2*(dist-self.standoff)))
                cmd.angular.z = max(-0.5, min(0.5, 2.0*err))
                self.pub_vel.publish(cmd)
            return

    # ── 디버그 창 ─────────────────────────────────────────────────────────────
    def _show_debug(self, frame, color):
        disp = frame.copy()
        h, w = frame.shape[:2]
        rw = int(w*self.roi_ratio); rh = int(h*self.roi_ratio)
        x1, y1 = (w-rw)//2, (h-rh)//2
        bgr = _DRAW_BGR.get(color, (0,255,255))
        cv2.rectangle(disp, (x1,y1), (x1+rw, y1+rh), bgr, 2)
        label = f'[{self.state}] {color or "NONE"}  {math.degrees(self._spin_acc):.0f}°/360°'
        cv2.putText(disp, label, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, bgr, 2)
        cv2.imshow('test_real', disp)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TestReal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.pub_vel.publish(Twist())
        if node.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
