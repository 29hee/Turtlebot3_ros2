#!/usr/bin/env python3
"""
preflight.py — 매핑/런타임 시작 전 '사전 점검'. 그동안 런타임에서 하나씩 터지던
'조용한 실패'(numpy 충돌, 카메라 안 뜸/너무 느림, 로봇 bringup 미실행, TF 없음, 클럭 skew)를
시작 전에 한 번에 잡는다. ❌ 가 있으면 그것부터 고치고 매핑을 시작할 것.

실행:
    source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
    python3 scripts/preflight.py
"""
import importlib
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def check_deps():
    rows = []
    try:
        import numpy
        v = numpy.__version__
        ok = int(v.split('.')[0]) < 2
        rows.append((ok, f"numpy {v}" + ("" if ok else "  → numpy 2.x! cv_bridge 깨짐: pip3 install \"numpy<2\"")))
    except Exception as e:
        rows.append((False, f"numpy import 실패: {e}"))
    for mod, hint in [('cv2', 'python3-opencv'),
                      ('cv_bridge', 'ros-humble-cv-bridge'),
                      ('easyocr', 'pip3 install easyocr')]:
        try:
            importlib.import_module(mod)
            rows.append((True, f"{mod} import OK"))
        except Exception as e:
            rows.append((False, f"{mod} import 실패 → {hint}  ({type(e).__name__})"))
    return rows


class Preflight(Node):
    def __init__(self):
        super().__init__('preflight')
        self.img = 0
        self.scan = 0
        self.last_scan_stamp = None
        self.create_subscription(Image, '/camera/image_raw',
                                 lambda m: self._img(), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        import tf2_ros
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def _img(self):
        self.img += 1

    def _on_scan(self, msg):
        self.scan += 1
        self.last_scan_stamp = msg.header.stamp


def line(ok, msg):
    print(("  ✅ " if ok else "  ❌ ") + msg)


def warn(msg):
    print("  ⚠  " + msg)


def main():
    print("\n=== [1] 파이썬 의존성 ===")
    deps_ok = True
    for ok, msg in check_deps():
        line(ok, msg)
        deps_ok = deps_ok and ok

    rclpy.init()
    n = Preflight()
    print("\n=== [2] 토픽 수신(5초 측정) ===")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    dur = max(0.1, time.time() - t0)
    img_hz, scan_hz = n.img / dur, n.scan / dur

    if img_hz <= 0:
        line(False, "/camera/image_raw 0Hz — v4l2_camera 미실행? (카메라 노드 확인)")
    elif img_hz < 5:
        warn(f"/camera/image_raw {img_hz:.1f}Hz — 너무 느림. 이동 중 색을 놓침 → "
             f"v4l2 -p image_size:=\"[640,480]\" -p time_per_frame:=\"[1,15]\" 또는 압축전송")
    else:
        line(True, f"/camera/image_raw {img_hz:.1f}Hz")

    if scan_hz <= 0:
        line(False, "/scan 0Hz — 로봇 bringup(turtlebot3_node) 미실행 or ROS_DOMAIN_ID 불일치")
    else:
        line(True, f"/scan {scan_hz:.1f}Hz")

    print("\n=== [3] TF (map/odom -> base_link) ===")
    tf_ok = False
    for parent in ('map', 'odom'):
        try:
            n.tfb.lookup_transform(parent, 'base_link', Time())
            line(True, f"TF {parent}->base_link OK")
            tf_ok = True
            break
        except Exception:
            warn(f"TF {parent}->base_link 없음")
    if not tf_ok:
        line(False, "TF 없음 — SLAM/AMCL 또는 로봇 bringup(odom) 확인")

    print("\n=== [4] 클럭 동기 (Pi↔PC) ===")
    if n.last_scan_stamp is not None:
        skew = abs((n.get_clock().now() - Time.from_msg(n.last_scan_stamp)).nanoseconds) / 1e9
        if skew > 0.5:
            line(False, f"/scan 타임스탬프가 현재시각과 {skew:.1f}s 차이 — 로봇 Pi와 PC 클럭 어긋남! "
                        f"(slam 'timestamp earlier than cache' 원인) → 양쪽 chrony/NTP 동기 필요")
        else:
            line(True, f"클럭 skew {skew*1000:.0f}ms (OK)")
    else:
        warn("/scan 미수신이라 클럭 점검 불가")

    print("\n=== [5] 로봇이 cmd_vel 듣고 있나 ===")
    cnt = n.count_subscribers('/cmd_vel')
    if cnt > 0:
        line(True, f"/cmd_vel 구독자 {cnt} (로봇 수신 중)")
    else:
        line(False, "/cmd_vel 구독자 0 — 로봇이 명령을 안 들음(bringup 확인). 탐사기가 움직여도 로봇은 가만")

    print("\n점검 끝. ❌/⚠ 항목부터 해결하고 매핑을 시작하세요.\n")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
