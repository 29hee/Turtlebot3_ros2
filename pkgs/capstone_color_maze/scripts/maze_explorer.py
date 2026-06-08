#!/usr/bin/env python3
"""
maze_explorer.py — 색-반응형 매핑 탐사 주행 (scan_explorer 대체).

[왜 새로 쓰나]
구 scan_explorer 는 ①시간 기반 + ②하드코딩 시뮬 웨이포인트 + ③막히면 제자리 회전 이라
실공간에서 '같은 자리 빙빙' 이 났다. 또 색을 멀리서 스치듯 봐 색·숫자 매핑이 부실했다.
→ 본 노드는 '벽을 타며 색을 찾고, 색을 보면 그 패널 정면 ~0.3m 까지 비주얼 서보로
  접근해 멈춰서(dwell) 그 근접 위치에서만 색·숫자를 쌓게' 한다. 기록 자체는 옆에서 도는
  color_mapper(색·digit 격자투표) + digit_recognizer(EasyOCR) 가 담당하고, 이 노드는
  '좋은 관측 위치로 로봇을 데려가는' 역할만 한다(역할 분리).

[같은 자리 빙빙 금지 — 필수 안전장치]
  A. 방문 격자 메모리 + 미방문 지향(하드코딩 웨이포인트 폐기).
  B. 진행 워치독: 윈도 동안 이동거리 < stuck_dist 면 '갇힘' → 강제 탈출 기동.
  C. loop/orbit 감지: 시작 셀로 충분히 이동 후 복귀 시 그 루프 종료 → 다음 미방문 구역.
  D. TF 끊김 시 무한 제자리 회전 금지(짧게만, 이후 느린 전진으로 칸 바꿔 재수렴).
  E. 종료는 시간이 아니라 '미방문 소진' (시간은 안전 상한일 뿐).

[2국면]  PERIMETER(둘레 벽타기) → loop closure → INWARD(중앙 진입) → ISLAND(섬 벽타기).
  어느 국면이든 색을 보면 APPROACH→CAPTURE→RESUME 가 끼어든다.

전제: TF 'map' 프레임(slam_toolbox), 회전은 느리게(맵 안 깨지게).
실행:  python3 maze_explorer.py --ros-args -p use_sim_time:=true
"""
import math
import os
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import tf2_ros

from maze_common import id_to_color, resolve_target_walls, VALID_COLORS
# id_to_color: color_id → 'RED' 등 (vision_node 와 동일 표)
# resolve_target_walls/VALID_COLORS: 품질 게이트에서 색+숫자 '벽' 수 집계용


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class MazeExplorer(Node):
    def __init__(self, duration):
        super().__init__('maze_explorer')
        # use_sim_time 은 --ros-args -p 로 넘기면 rclpy 가 자동 선언하므로 여기서 선언하지 않는다
        # (중복 선언하면 ParameterAlreadyDeclaredException 으로 노드가 죽는다).

        # ── 주행/벽타기 파라미터 ─────────────────────────────────────
        self.declare_parameter('v_fwd', 0.08)        # 전진 속도 [m/s] (느리게 — 색 놓치지 않게)
        self.declare_parameter('target_right', 0.45)  # 오른쪽 벽 유지 거리 [m]
        self.declare_parameter('front_stop', 0.45)    # 전방 정지 임계 [m]
        self.declare_parameter('spin_speed', 0.25)    # 회전 각속도 [rad/s] (느리게! SLAM 보호)
        # ── 색 접근(비주얼 서보) 파라미터 ───────────────────────────
        # 작은 색 조각(≈5%)만 보여도 그쪽으로 정렬·접근하도록 낮게 둔다.
        self.declare_parameter('seen_ratio', 0.03)    # ROI 색 점유율 ≥ 이 값이면 '발견' → 접근
        self.declare_parameter('standoff', 0.30)      # 패널 앞 정지 거리(라이다 정면) [m]
        self.declare_parameter('capture_secs', 2.5)   # 근접 dwell 시간 [s] (이 동안 기록 누적)
        self.declare_parameter('dedup_dist', 0.6)     # 이 거리 내 이미 캡처한 패널이면 재접근 스킵 [m]
        # ── 안티-스턱 파라미터 ───────────────────────────────────────
        self.declare_parameter('visit_res', 0.4)      # 방문 격자 한 변 [m]
        self.declare_parameter('stuck_win', 8.0)      # 진행 점검 윈도 [s]
        self.declare_parameter('stuck_dist', 0.2)     # 윈도 동안 이 거리 미만 이동이면 '갇힘' [m]
        self.declare_parameter('escape_secs', 3.0)    # 탈출 기동 지속 [s]
        self.declare_parameter('loop_close_dist', 0.5)  # 시작 셀 복귀 판정 거리 [m]
        self.declare_parameter('tf_lost_spin', 1.5)   # TF 끊김 시 제자리 회전 최대 [s]
        # 색 접근이 이 시간 안에 근접(standoff) 도달 못 하면 포기하고 벽타기 복귀.
        #   (어안렌즈로 색 중심 정렬이 안 돼 제자리 회전만 하는 무한루프 방지.)
        self.declare_parameter('approach_timeout', 12.0)
        # ── 센서/odom 생존 워치독 ────────────────────────────────────
        #   라이다(/scan)나 위치(TF map→base_link)가 끊기면(예: turtlebot3_node 배터리로
        #   사망, 라이다 멈춤) '탐사 중'으로 착각하며 헛돌지 않게 즉시 정지+경고한다.
        self.declare_parameter('sensor_timeout', 3.0)    # /scan 이 이 시간 끊기면 정지 [s]
        self.declare_parameter('pose_lost_limit', 12.0)  # TF 위치가 이 시간 끊기면 정지+경고 [s]
        # ── 매핑 종료 품질 게이트 ────────────────────────────────────
        #   자연 종료(둘레/섬 한 바퀴) 시 color_landmarks.yaml 의 '색+숫자 벽' 수가
        #   min_quality_walls 미만이면 종료하지 않고 재탐사(놓친 숫자 재수집). 0=비활성.
        #   시간 상한(total)은 그대로 안전망이라 무한루프 없음.
        self.declare_parameter('min_quality_walls', 1)   # 종료 전 필요한 색+숫자 벽 최소수(0=끔)
        self.declare_parameter('max_resweeps', 2)        # 품질 미달 시 재탐사 최대 횟수

        self.total = duration
        self.v_fwd = float(self.get_parameter('v_fwd').value)
        self.target_right = float(self.get_parameter('target_right').value)
        self.front_stop = float(self.get_parameter('front_stop').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)
        self.seen_ratio = float(self.get_parameter('seen_ratio').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.capture_secs = float(self.get_parameter('capture_secs').value)
        self.dedup_dist = float(self.get_parameter('dedup_dist').value)
        self.visit_res = float(self.get_parameter('visit_res').value)
        self.stuck_win = float(self.get_parameter('stuck_win').value)
        self.stuck_dist = float(self.get_parameter('stuck_dist').value)
        self.escape_secs = float(self.get_parameter('escape_secs').value)
        self.loop_close_dist = float(self.get_parameter('loop_close_dist').value)
        self.tf_lost_spin = float(self.get_parameter('tf_lost_spin').value)
        self.approach_timeout = float(self.get_parameter('approach_timeout').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.pose_lost_limit = float(self.get_parameter('pose_lost_limit').value)
        self.min_quality_walls = int(self.get_parameter('min_quality_walls').value)
        self.max_resweeps = int(self.get_parameter('max_resweeps').value)
        self._resweeps = 0
        # color_mapper 와 동일 경로의 color_landmarks.yaml (품질 게이트에서 읽음)
        _here = os.path.dirname(os.path.realpath(__file__))
        self._landmarks_path = os.path.join(os.path.dirname(_here), 'maps', 'color_landmarks.yaml')

        # ── IO ───────────────────────────────────────────────────────
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        # 영상은 직접 안 푼다 — vision_node 가 푼 색 신호만 구독(단일 디코딩).
        self.create_subscription(Float32MultiArray, '/color_signal', self.on_signal, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 진척 상황 방송(quality_monitor/로그용): "PERIMETER", "CAPTURE RED@(x,y)" 등
        self.pub_phase = self.create_publisher(String, '/explorer_phase', 10)

        # ── 상태 ─────────────────────────────────────────────────────
        self.scan = None
        self.color = 'NONE'          # 최근 우세색
        self.color_cx = 0.0          # 색 blob 중심 x (정규화 -1~+1, +는 화면 오른쪽)
        self.color_cov = 0.0         # ROI 색 점유율
        self.phase = 'PERIMETER'     # PERIMETER | INWARD | ISLAND | (+ APPROACH/CAPTURE/ESCAPE 인터럽트)
        self.interrupt = None        # 'APPROACH' | 'CAPTURE' | 'ESCAPE' | None
        self.start = self.now()
        self.phase_start = self.start
        self.captured = []           # [(x,y), ...] 이미 캡처한 패널 map 좌표(중복 방지)
        self.skipped = []            # [(x,y), ...] 접근 시간초과로 건너뛴 위치(재트리거 방지)
        self.inward_target = None    # 내부(섬) 진입 목표 = 둘레 경로 무게중심(≈방 중앙)
        self.visited = set()         # 방문 격자 셀 인덱스
        self.start_cell = None       # 둘레 loop closure 기준 시작 셀
        self.left_start = False      # 시작 셀을 충분히 벗어났는가(복귀 판정 게이트)
        # 진행 워치독용 표본
        self.wd_pose = None
        self.wd_time = self.start
        # /color_signal 생존 감시 — 안 들어오면 vision_node 가 죽은 것(색 못 봄 → approach 불가)
        self._last_signal_time = None
        self._last_sig_warn = 0.0
        self._last_phase_pub = 0.0
        # 센서/odom 워치독 상태
        self._last_scan_t = None       # 마지막 /scan 수신 시각
        self._pose_lost_since = None   # TF 위치 끊김 시작 시각
        self._last_sensor_warn = 0.0
        self._last_pose_warn = 0.0
        self.get_logger().info(
            f"maze_explorer 시작 — 색-반응 매핑, total(상한)={self.total:.0f}s, "
            f"standoff={self.standoff}m, stuck<{self.stuck_dist}m/{self.stuck_win:.0f}s")
        self.timer = self.create_timer(0.05, self.on_timer)

    # ── 시간/센서 ────────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now()

    def elapsed(self, since):
        return (self.now() - since).nanoseconds / 1e9

    def on_scan(self, msg):
        self.scan = msg
        self._last_scan_t = self.now()   # 센서 워치독용 생존 표시

    def on_signal(self, msg):
        """vision_node 의 [color_id, cx_norm, coverage] → 비주얼 서보/발견 신호.
        seen_ratio 미만 점유율은 NONE 으로 취급(탐사기측 발견 임계)."""
        d = msg.data
        self._last_signal_time = self.now()   # 신호 생존 표시
        if len(d) < 3:
            return
        cov = float(d[2])
        self.color = id_to_color(d[0]) if cov >= self.seen_ratio else 'NONE'
        self.color_cx = float(d[1])
        self.color_cov = cov

    def sector_min(self, deg_lo, deg_hi):
        s = self.scan
        if s is None:
            return float('inf')
        n = len(s.ranges)
        vals = []
        d = deg_lo
        while d <= deg_hi:
            idx = int(round((math.radians(d) - s.angle_min) / s.angle_increment)) % n
            r = s.ranges[idx]
            if r and math.isfinite(r) and s.range_min < r < s.range_max:
                vals.append(r)
            d += 1
        return min(vals) if vals else float('inf')

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def cell(self, x, y):
        return (int(math.floor(x / self.visit_res)), int(math.floor(y / self.visit_res)))

    def visited_centroid(self):
        """방문한 격자 셀들의 중심 = 둘레를 돈 경로의 무게중심 ≈ 방 중앙.
        SLAM 원점(0,0)은 '로봇 시작 위치'이지 방 중앙이 아니므로, 내부(섬) 진입 목표로
        이걸 쓴다(시작점으로 되돌아가 엉뚱한 벽을 섬으로 오인하던 문제 해결)."""
        if not self.visited:
            return (0.0, 0.0)
        xs = [(gx + 0.5) * self.visit_res for (gx, gy) in self.visited]
        ys = [(gy + 0.5) * self.visit_res for (gx, gy) in self.visited]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    # ── 주행 프리미티브 ──────────────────────────────────────────────
    def wall_follow_cmd(self):
        """오른손 벽타기 한 스텝."""
        front = self.sector_min(-20, 20)
        right = self.sector_min(-100, -80)
        cmd = Twist()
        if front < self.front_stop:
            cmd.angular.z = 0.4            # 전방 막힘 → 좌회전(천천히)
        elif right > 0.9:
            cmd.linear.x = 0.07            # 오른벽 잃음 → 오른쪽으로 붙기
            cmd.angular.z = -0.3
        else:
            err = right - self.target_right
            cmd.linear.x = self.v_fwd
            cmd.angular.z = max(-0.35, min(0.35, -1.0 * err))
        return cmd

    def approach_cmd(self):
        """비주얼 서보: 색 blob 을 화면 중앙에 두고 라이다 정면거리 standoff 까지 전진.
        (도착여부, Twist). 색을 잃으면 도착 아님으로 반환(상위에서 복귀 처리)."""
        cmd = Twist()
        if self.color == 'NONE':
            return False, cmd                 # 색 놓침 → 상위에서 WALL_FOLLOW 복귀
        front = self.sector_min(-15, 15)
        # 중심 정렬: cx>0(오른쪽)이면 우회전(-z). 게인 작게(천천히 그쪽을 본다).
        cmd.angular.z = max(-0.35, min(0.35, -0.6 * self.color_cx))
        if front <= self.standoff:
            return True, Twist()              # 도착(정지)
        # 정렬도에 비례해 전진: 중앙(cx≈0)=full, 비스듬할수록 느리게, |cx|≥0.5면 거의 정지.
        # 어안렌즈로 cx 가 안 잡혀도 '제자리 스톨' 없이 늘 그쪽을 보며 천천히 다가간다.
        align = max(0.0, 1.0 - abs(self.color_cx) / 0.5)
        cmd.linear.x = self.v_fwd * align
        return False, cmd

    def publish_phase(self, text):
        self.pub_phase.publish(String(data=text))

    def switch(self, phase=None, interrupt='__keep__'):
        if phase is not None:
            self.phase = phase
        if interrupt != '__keep__':
            self.interrupt = interrupt
        self.phase_start = self.now()

    # ── 안티-스턱: 진행 워치독 ──────────────────────────────────────
    def watchdog_stuck(self, pose):
        """윈도 동안 이동거리가 stuck_dist 미만이면 True(갇힘). 표본 갱신 포함."""
        if pose is None:
            return False
        if self.wd_pose is None:
            self.wd_pose, self.wd_time = pose, self.now()
            return False
        if self.elapsed(self.wd_time) >= self.stuck_win:
            moved = math.hypot(pose[0] - self.wd_pose[0], pose[1] - self.wd_pose[1])
            self.wd_pose, self.wd_time = pose, self.now()
            if moved < self.stuck_dist:
                return True
        return False

    # ── 메인 루프 ────────────────────────────────────────────────────
    def on_timer(self):
        # 0) 안전 상한(시간) — 종료는 본래 미방문 소진이지만, 폭주 방지용 상한.
        if self.elapsed(self.start) > self.total:
            self.stop_and_quit(f'시간 상한({self.total:.0f}s) 도달 → 정지')
            return
        if self.scan is None:
            return

        # ★ 센서 워치독: /scan 이 sensor_timeout 이상 끊기면 정지(stale scan 으로 헛돌기 방지).
        if (self._last_scan_t is not None
                and self.elapsed(self._last_scan_t) > self.sensor_timeout):
            self.pub.publish(Twist())
            now_s = self.elapsed(self.start)
            if now_s - self._last_sensor_warn > 5.0:
                self._last_sensor_warn = now_s
                self.get_logger().error(
                    f'/scan {self.sensor_timeout:.0f}s+ 끊김 — 라이다/로봇 bringup 확인. 정지.')
            return

        # ★ 색 신호(/color_signal) 생존 감시 — 이게 없으면 색을 못 봐 'approach' 자체가 불가.
        #   (조용히 벽만 도는 대신 큰 소리로 알려 vision_node 누락/사망을 즉시 드러낸다.)
        now_s = self.elapsed(self.start)
        # 카메라가 느리면(1~2Hz) 정상에도 몇 초 공백이 난다 → 8s 임계 + 시작 유예로 오발 방지.
        stale = (self._last_signal_time is None
                 or self.elapsed(self._last_signal_time) > 8.0)
        if now_s > 12.0 and stale and now_s - self._last_sig_warn > 15.0:
            self._last_sig_warn = now_s
            self.get_logger().warn(
                "/color_signal 8s+ 끊김 — vision_node 생존/카메라 Hz 확인 "
                "(카메라가 매우 느리면 정상일 수 있음).")

        # 현재 상태를 1초마다 방송(quality_monitor/사용자가 실시간으로 뭐 하는지 보게)
        if now_s - self._last_phase_pub > 1.0:
            self._last_phase_pub = now_s
            tag = self.interrupt or self.phase
            self.publish_phase(f"{tag} (색:{self.color}, 캡처:{len(self.captured)})")

        pose = self.get_pose()

        # 1) TF 위치 끊김 처리.
        if pose is None:
            if self._pose_lost_since is None:
                self._pose_lost_since = self.now()
            lost_for = self.elapsed(self._pose_lost_since)
            if lost_for > self.pose_lost_limit:
                # 장기 끊김 = 일시적 SLAM 흔들림이 아니라 odom/TF 사망(예: turtlebot3_node
                #   배터리로 죽음). '장님 주행' 금지 — 정지하고 큰 소리로 알린다.
                self.pub.publish(Twist())
                now_s = self.elapsed(self.start)
                if now_s - self._last_pose_warn > 5.0:
                    self._last_pose_warn = now_s
                    self.get_logger().error(
                        f'위치(TF map→base_link) {lost_for:.0f}s 끊김 — odom/SLAM 사망 의심 '
                        f'(turtlebot3_node·배터리 확인). 정지.')
                return
            # 단기 끊김 — '무한 제자리 회전 금지'(D). 짧게만 돌고, 그 뒤엔 느린 전진으로 칸 변경.
            c = Twist()
            if lost_for < self.tf_lost_spin:
                c.angular.z = self.spin_speed
            else:
                c.linear.x = 0.08      # 칸을 바꿔 SLAM 재수렴 유도(제자리 회전은 정보 0)
            self.pub.publish(c)
            return
        self._pose_lost_since = None   # 위치 정상 → 끊김 타이머 리셋

        # 방문 격자 기록
        self.visited.add(self.cell(pose[0], pose[1]))
        if self.start_cell is None:
            self.start_cell = self.cell(pose[0], pose[1])

        # 2) 갇힘 감지(B) → ESCAPE 인터럽트
        #    CAPTURE(정지 dwell 정상) / APPROACH(색 중심 정렬 위해 제자리 회전 가능)는 제외 —
        #    APPROACH 는 자체 approach_timeout 으로 무한루프를 막는다(아래).
        if self.interrupt not in ('CAPTURE', 'APPROACH') and self.watchdog_stuck(pose):
            self.get_logger().warn('진행 워치독: 갇힘 감지 → 탈출 기동')
            self.publish_phase('ESCAPE')
            self.switch(interrupt='ESCAPE')

        # ── 인터럽트 처리(색 접근/캡처/탈출이 일반 국면보다 우선) ──
        if self.interrupt == 'ESCAPE':
            # 전방이 열린 쪽으로 틀어 직진해 루프/끼임을 깬다.
            c = Twist()
            if self.sector_min(-25, 25) < self.front_stop:
                left = self.sector_min(20, 70); right = self.sector_min(-70, -20)
                c.angular.z = 0.45 if left > right else -0.45
            else:
                c.linear.x = self.v_fwd
            self.pub.publish(c)
            if self.elapsed(self.phase_start) >= self.escape_secs:
                self.switch(interrupt=None)   # 일반 국면 복귀
            return

        if self.interrupt == 'APPROACH':
            arrived, cmd = self.approach_cmd()
            # 시간초과: 어안렌즈로 중심 정렬이 안 돼 standoff 도달 실패 → 포기하고 벽타기 복귀.
            #   이 자리를 skipped 로 기억해 바로 재트리거되지 않게 한다.
            if not arrived and self.elapsed(self.phase_start) >= self.approach_timeout:
                self.skipped.append((pose[0], pose[1]))
                self.get_logger().warn(
                    f'접근 시간초과({self.approach_timeout:.0f}s) — 건너뜀 @({pose[0]:.2f},{pose[1]:.2f})')
                self.switch(interrupt=None)
                return
            if self.color == 'NONE':
                self.switch(interrupt=None)   # 색 놓침 → 일반 국면
                return
            if arrived:
                self.publish_phase(f'CAPTURE {self.color}')
                self.get_logger().info(f'패널 근접 도달({self.color}) → {self.capture_secs:.1f}s 기록')
                self.switch(interrupt='CAPTURE')
                return
            self.pub.publish(cmd)
            return

        if self.interrupt == 'CAPTURE':
            self.pub.publish(Twist())          # 정지 dwell — mapper/recognizer 가 기록
            if self.elapsed(self.phase_start) >= self.capture_secs:
                self.captured.append((pose[0], pose[1]))
                self.get_logger().info(f'캡처 완료 @({pose[0]:.2f},{pose[1]:.2f}) — 총 {len(self.captured)}개')
                self.switch(interrupt=None)    # 일반 국면 복귀(다음 벽 탐색)
            return

        # ── 색 발견 → 접근 트리거(중복/방금 캡처 제외) ──
        if self.color != 'NONE' and not self._recently_captured(pose):
            self.publish_phase(f'APPROACH {self.color}')
            self.switch(interrupt='APPROACH')
            return

        # ── 일반 국면: 벽타기 + loop closure → 중앙 진입 → 섬 ──
        self.run_phase(pose)

    def _recently_captured(self, pose):
        """현재 위치가 이미 캡처했거나 접근 포기(skipped)한 지점 근처면 True(재트리거 방지)."""
        for (cx, cy) in self.captured + self.skipped:
            if math.hypot(pose[0] - cx, pose[1] - cy) < self.dedup_dist:
                return True
        return False

    def run_phase(self, pose):
        x, y, _ = pose
        cur = self.cell(x, y)

        if self.phase == 'PERIMETER':
            # 시작 셀을 충분히 벗어났다가 다시 돌아오면 둘레 한 바퀴 완료로 본다(C).
            d_start = math.hypot(x - (self.start_cell[0] + 0.5) * self.visit_res,
                                 y - (self.start_cell[1] + 0.5) * self.visit_res)
            if not self.left_start and d_start > 1.0:
                self.left_start = True
            if self.left_start and d_start < self.loop_close_dist:
                self.inward_target = self.visited_centroid()   # (0,0)=시작점 아님 → 둘레 중심으로
                self.get_logger().info(
                    f'=== 둘레 한 바퀴(loop closure) → 내부'
                    f'({self.inward_target[0]:+.1f},{self.inward_target[1]:+.1f}) 진입 ===')
                self.publish_phase('INWARD')
                self.switch('INWARD')
                return
            self.pub.publish(self.wall_follow_cmd())

        elif self.phase == 'INWARD':
            # 맵 중앙(0,0)으로 직진하다 섬 벽을 만나면 섬 벽타기로 전환.
            front = self.sector_min(-20, 20)
            if front < self.front_stop:
                self.get_logger().info('=== 중앙에서 섬 벽 접촉 → 섬 벽타기 ===')
                self.publish_phase('ISLAND')
                self.island_start_cell = cur
                self.left_start = False
                self.start_cell = cur          # 섬 loop closure 기준 갱신
                self.switch('ISLAND')
                return
            tx, ty = self.inward_target if self.inward_target else (0.0, 0.0)
            c = Twist()
            herr = wrap(math.atan2(ty - y, tx - x) - pose[2])
            c.angular.z = max(-0.4, min(0.4, 0.9 * herr))
            c.linear.x = self.v_fwd if abs(herr) < 0.5 else 0.05
            # 내부 목표(둘레 무게중심) 근처인데 섬이 없으면 섬 없음 → 종료.
            if math.hypot(x - tx, y - ty) < 0.4:
                self.complete_or_continue('내부 중심 도달했으나 섬 없음 → 탐사 종료')
                return
            self.pub.publish(c)

        elif self.phase == 'ISLAND':
            d_start = math.hypot(x - (self.start_cell[0] + 0.5) * self.visit_res,
                                 y - (self.start_cell[1] + 0.5) * self.visit_res)
            if not self.left_start and d_start > 1.0:
                self.left_start = True
            if self.left_start and d_start < self.loop_close_dist:
                self.complete_or_continue('=== 섬 한 바퀴 완료 → 탐사 종료 ===')
                return
            self.pub.publish(self.wall_follow_cmd())

    # ── 품질 게이트 ──────────────────────────────────────────────────
    def _count_quality_walls(self):
        """color_landmarks.yaml 에서 '색+숫자' 가 확정된 '벽'(클러스터) 수를 센다.
        파일 없음/깨짐은 0(노드는 안 죽음)."""
        try:
            with open(self._landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return 0
        total = 0
        for c in VALID_COLORS:
            for w in resolve_target_walls(data, c):
                if w.get('digit') is not None:
                    total += 1
        return total

    def complete_or_continue(self, reason):
        """자연 종료 시점의 품질 게이트. 기준 충족이면 종료, 미달이면 재탐사(상한까지),
        상한 도달이면 경고와 함께 그대로 종료. (시간 상한은 별도 안전망.)"""
        if self.min_quality_walls <= 0:
            self.stop_and_quit(reason)            # 게이트 비활성 → 기존 동작
            return
        n = self._count_quality_walls()
        if n >= self.min_quality_walls:
            self.stop_and_quit(reason + f' (품질 OK: 색+숫자 벽 {n}개)')
            return
        if self._resweeps < self.max_resweeps:
            self._resweeps += 1
            self.get_logger().warn(
                f'품질 미달(색+숫자 벽 {n}/{self.min_quality_walls}) → 재탐사 '
                f'{self._resweeps}/{self.max_resweeps}: 놓친 숫자 재수집을 위해 다시 돈다')
            self.publish_phase(f'QUALITY_LOW {n}/{self.min_quality_walls}')
            # 캡처/스킵 기록을 비워 이미 본 패널도 재접근해 digit 을 다시 읽게 한다.
            self.captured.clear()
            self.skipped.clear()
            self.phase = 'PERIMETER'
            self.interrupt = None
            self.left_start = False
            self.start_cell = None
            self.phase_start = self.now()
            return
        self.stop_and_quit(
            reason + f' (품질 미달 {n}/{self.min_quality_walls}, 재탐사 상한 도달 — 그대로 종료)')

    def stop_and_quit(self, msg):
        self.pub.publish(Twist())
        self.get_logger().info(msg + f' (캡처 {len(self.captured)}개, 방문셀 {len(self.visited)})')
        self.publish_phase('DONE')
        # 타이머 콜백 안에서 shutdown 하면 executor 가 교착 → SystemExit 로 빠져나가 main 정리.
        self.timer.cancel()
        raise SystemExit


def _arg(name, default, cast=float):
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            return cast(sys.argv[i + 1])
    return default


def main():
    duration = _arg('--duration', 600.0)
    rclpy.init()
    node = MazeExplorer(duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
