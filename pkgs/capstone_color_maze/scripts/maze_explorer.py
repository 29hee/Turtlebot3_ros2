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
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import tf2_ros

from maze_common import id_to_color   # color_id → 'RED' 등 (vision_node 와 동일 표)


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
        # ★ 먼저 색을 화면 중앙으로 '유심히' 정렬한 뒤에만 전진. 덜 맞으면 제자리 회전만(천천히).
        cmd.linear.x = self.v_fwd if abs(self.color_cx) < 0.25 else 0.0
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

        # ★ 색 신호(/color_signal) 생존 감시 — 이게 없으면 색을 못 봐 'approach' 자체가 불가.
        #   (조용히 벽만 도는 대신 큰 소리로 알려 vision_node 누락/사망을 즉시 드러낸다.)
        now_s = self.elapsed(self.start)
        stale = (self._last_signal_time is None
                 or self.elapsed(self._last_signal_time) > 3.0)
        if stale and now_s - self._last_sig_warn > 5.0:
            self._last_sig_warn = now_s
            self.get_logger().error(
                "⚠ /color_signal 안 들어옴 → 색을 못 봐 패널 접근 불가(OCR도 불가). "
                "vision_node 가 떠 있는지 확인: ros2 node list | grep vision_node "
                "(없으면 mapping.launch 로 띄우거나 vision_node.py 실행; numpy<2 필요)")

        # 현재 상태를 1초마다 방송(quality_monitor/사용자가 실시간으로 뭐 하는지 보게)
        if now_s - self._last_phase_pub > 1.0:
            self._last_phase_pub = now_s
            tag = self.interrupt or self.phase
            self.publish_phase(f"{tag} (색:{self.color}, 캡처:{len(self.captured)})")

        pose = self.get_pose()

        # 1) TF 끊김 — '무한 제자리 회전 금지'(D). 짧게만 돌고, 그 뒤엔 느린 전진으로 칸 변경.
        if pose is None:
            c = Twist()
            if self.elapsed(self.phase_start) < self.tf_lost_spin:
                c.angular.z = self.spin_speed
            else:
                c.linear.x = 0.08      # 칸을 바꿔 SLAM 재수렴 유도(제자리 회전은 정보 0)
            self.pub.publish(c)
            return

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
                self.get_logger().info('=== 둘레 한 바퀴(loop closure) → 중앙 진입 ===')
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
            c = Twist()
            herr = wrap(math.atan2(0.0 - y, 0.0 - x) - pose[2])
            c.angular.z = max(-0.4, min(0.4, 0.9 * herr))
            c.linear.x = self.v_fwd if abs(herr) < 0.5 else 0.05
            # 중앙 근처(원점 0.4m)인데 섬이 없으면 섬 없음 → 종료.
            if math.hypot(x, y) < 0.4:
                self.stop_and_quit('중앙 도달했으나 섬 없음 → 탐사 종료')
                return
            self.pub.publish(c)

        elif self.phase == 'ISLAND':
            d_start = math.hypot(x - (self.start_cell[0] + 0.5) * self.visit_res,
                                 y - (self.start_cell[1] + 0.5) * self.visit_res)
            if not self.left_start and d_start > 1.0:
                self.left_start = True
            if self.left_start and d_start < self.loop_close_dist:
                self.stop_and_quit('=== 섬 한 바퀴 완료 → 탐사 종료 ===')
                return
            self.pub.publish(self.wall_follow_cmd())

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
