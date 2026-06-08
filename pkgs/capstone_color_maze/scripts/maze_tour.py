#!/usr/bin/env python3
"""
maze_tour.py
색상 시맨틱맵(color_landmarks.yaml)을 이용한 '색벽 순회 + 마지막 벽 정지' 런타임.
[Phase 4 — 수정 사양]

  ※ 이 노드는 mission_executor.py(구 사양: 가장 가까운 한 벽만 들렀다가 '출구'로 복귀)를
    대체한다. 현재 사양에는 출구가 없다 — target_color 의 '모든' 벽을 순회하며 각 벽을
    카메라로 확인하고, '마지막으로 확인한 벽'에서 정지한 뒤 /maze_done 을 발행한다.

동작(이벤트 구동 서비스):
  - 시작 시 실로봇이면 relocalize(자기위치 추정) 1회 후 대기한다.
  - /target_color(std_msgs/String: RED|GREEN|BLUE)를 받으면 그 색 순회를 시작:
    1) color_landmarks.yaml 에서 그 색의 모든 벽을 계산(없으면 no-match, 미이동).   [AC7]
    2) 현재 위치 기준 nearest-neighbor 방문 순서.
    3) 각 벽 '접근 포즈'로 Nav2 주행 → color_confirm 의 /target_confirmed
       (점유율 >= CONFIRM_THRESHOLD, 현재 30%)로 확인.
    4) 모든 벽 확인 후 '마지막 확인 벽'에 정지 + /maze_done(True).               [AC6]
       (하나라도 확인 실패면 부분결과 처리하고 /maze_done 미발행.)
  - 순회가 끝나면 다시 /target_color 대기(oneshot=true 면 1회 후 종료).

전제 스택(미리 실행 — bringup.launch.py 가 함께 띄움):
  - 저장된 맵 + map_server + AMCL, nav2(navigate_to_pose), color_confirm.py

실행:
  # 연속 서비스(색 무관 대기): bringup.launch.py 가 이 노드를 띄운다. 색은 토픽으로:
  ros2 topic pub --once /target_color std_msgs/String "{data: RED}"
  # 단발 데모: python3 maze_tour.py --ros-args -p target_color:=RED -p oneshot:=true
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose

from maze_common import (
    normalize_color, parse_target, approach_pose, order_walls, resolve_target_walls,
    CONFIRM_THRESHOLD,
)

# 로그용 한국어 색 이름 ("빨강 3번에 도착했습니다")
KOR = {'RED': '빨강', 'GREEN': '초록', 'BLUE': '파랑'}


def default_landmarks_path():
    """이 스크립트 기준 ../maps/color_landmarks.yaml (하드코딩 경로 제거)."""
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class MazeTour(Node):
    def __init__(self):
        super().__init__('maze_tour')

        # ── 파라미터 ──────────────────────────────────────────────
        # 초기 색(비우면 /target_color 가 올 때까지 대기). 색 무관 bringup 은 ''.
        self.declare_parameter('target_color', '')
        # true: 한 색 순회 후 노드 종료(단발 데모/runtime.launch 용).
        # false: 순회 후 다시 /target_color 대기(연속 서비스/bringup 용).
        self.declare_parameter('oneshot', False)
        self.declare_parameter('landmarks_path', default_landmarks_path())
        self.declare_parameter('standoff', 0.45)        # 벽 앞 정지 거리 [m]
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('confirm_window', 4.0)   # 도착 후 확인 관측 시간 [s]
        self.declare_parameter('confirm_min_true', 3)   # 이 횟수 이상 True 면 확인
        # 시작 시 자기위치 재추정(relocalization). SLAM 매핑 땐 불필요(위치 고정)하고,
        # 실로봇 런타임에선 켠 위치가 맵 어디인지 모르므로 true 로 켜서, 전역 파티클을
        # 흩뿌린 뒤 제자리 회전으로 AMCL 을 수렴시키고 나서 순회를 시작한다.
        # (시뮬은 set_initial_pose 가 맞으므로 기본 false 로 두어 기존 동작 보존.)
        self.declare_parameter('relocalize', False)
        self.declare_parameter('relocalize_speed', 0.5)      # 회전 각속도 [rad/s]
        # 고정 바퀴수가 아니라 'AMCL 공분산이 임계 이하로 수렴할 때까지' 회전한다.
        # relocalize_max_turns 는 수렴 못 할 때를 대비한 안전 상한(무한회전 방지).
        # 임계는 실측 기반: 전역분산 후 ~20s(±1.6바퀴) 회전하면 pos_std~0.17m,
        # yaw_std~0.28rad 에서 평탄화한다(회전 중엔 모션모델이 yaw 불확실성을 계속
        # 주입해 그 아래로는 안 내려감). 그 plateau 바로 위로 임계를 잡아 '수렴 도달'을
        # 판정한다. (더 빡빡하게 잡으면 영원히 트립 못 하고 상한까지 헛돈다 — 실측 확인.)
        self.declare_parameter('relocalize_max_turns', 3.0)  # 최대 회전 바퀴 수(상한)
        self.declare_parameter('relocalize_pos_std', 0.25)   # 위치 표준편차 임계 [m]
        self.declare_parameter('relocalize_yaw_std', 0.35)   # yaw 표준편차 임계 [rad]

        init_color, init_digit = parse_target(self.get_parameter('target_color').value)
        self.target = init_color
        self.target_digit = init_digit      # None 이면 색만 확인, 정수면 색+숫자 확인
        self.oneshot = bool(self.get_parameter('oneshot').value)
        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.confirm_window = float(self.get_parameter('confirm_window').value)
        self.confirm_min_true = int(self.get_parameter('confirm_min_true').value)
        self.relocalize = bool(self.get_parameter('relocalize').value)
        self.relocalize_speed = float(self.get_parameter('relocalize_speed').value)
        self.relocalize_max_turns = float(self.get_parameter('relocalize_max_turns').value)
        self.relocalize_pos_std = float(self.get_parameter('relocalize_pos_std').value)
        self.relocalize_yaw_std = float(self.get_parameter('relocalize_yaw_std').value)

        # ── 상태/IO ───────────────────────────────────────────────
        self._confirmed_now = False        # /target_confirmed 최신값
        self._detected_digit = -1          # /detected_digit 최신값 (-1=없음)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Bool, '/target_confirmed', self._on_confirmed, 10)
        self.create_subscription(Int32, '/detected_digit', self._on_digit, 10)
        # relocalization 용: 제자리 회전 명령 + AMCL 전역 재초기화 서비스 + 공분산 모니터
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.global_loc = self.create_client(Empty, '/reinitialize_global_localization')
        self._amcl_cov = None   # 최신 /amcl_pose 공분산(36) — 수렴 판정용
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

        # /maze_done 은 늦게 접속한 구독자도 받도록 latched(transient_local)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_done = self.create_publisher(Bool, '/maze_done', latched)
        # 런타임 색 지정: /target_color(RED/GREEN/BLUE) 를 받으면 그 색 순회를 예약한다.
        self.pending_color = None
        self.pending_digit = None
        self.create_subscription(String, '/target_color', self._on_target_color, 10)

    # ── 콜백/유틸 ─────────────────────────────────────────────────
    def _on_confirmed(self, msg):
        self._confirmed_now = bool(msg.data)

    def _on_digit(self, msg):
        self._detected_digit = int(msg.data)

    def _on_amcl(self, msg):
        self._amcl_cov = msg.pose.covariance   # 길이 36 (행우선 6x6)

    def _on_target_color(self, msg):
        """런타임 타겟 지정. 'RED' 또는 'RED_1' 형식 모두 수신.
        다음 순회로 예약(현재 순회 중이면 끝난 뒤 처리)."""
        c, d = parse_target(msg.data)
        if c is None:
            self.get_logger().warn(f'/target_color 무시(유효하지 않은 형식): {msg.data!r}')
            return
        self.pending_color = c
        self.pending_digit = d
        label = f'{c}_{d}' if d is not None else c
        self.get_logger().info(f'/target_color 수신: {label} → 순회 예약')

    def _amcl_converged(self):
        """AMCL 위치/yaw 표준편차가 둘 다 임계 이하면 True (수렴)."""
        c = self._amcl_cov
        if c is None:
            return False
        pos_std = max(math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)))  # xx, yy
        yaw_std = math.sqrt(max(c[35], 0.0))                                  # yaw-yaw
        return pos_std <= self.relocalize_pos_std and yaw_std <= self.relocalize_yaw_std

    def load_target_walls(self):
        """원시 셀을 클러스터링/필터해 '진짜 벽'(각 벽에 안정 id 부여)으로 반환."""
        with open(self.landmarks_path) as f:
            data = yaml.safe_load(f) or {}
        return resolve_target_walls(data, self.target)

    def get_robot_xy(self, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                return tf.transform.translation.x, tf.transform.translation.y
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(yaw)
        return p

    def nav_to(self, x, y, yaw, label):
        """navigate_to_pose 동기 호출. 도착 성공 여부 반환."""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음 (Nav2 미실행?)')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        self.get_logger().info(f'[{label}] 주행 → ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}°)')

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'[{label}] 목표 거부됨')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(f'[{label}] {"도착" if ok else "실패(status=%d)" % status}')
        return ok

    def await_confirmation(self):
        """도착 후 confirm_window 초간 색(+숫자) 확인.
        - target_digit 없음: 색 확인만 (기존 동작)
        - target_digit 있음: 색 확인 AND 숫자 일치 둘 다 만족해야 True"""
        self._confirmed_now = False
        true_count = 0
        end = time.time() + self.confirm_window
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            color_ok = self._confirmed_now
            digit_ok = (self.target_digit is None or
                        self._detected_digit == self.target_digit)
            if color_ok and digit_ok:
                true_count += 1
                if true_count >= self.confirm_min_true:
                    return True
        return true_count >= self.confirm_min_true

    def relocalize_in_place(self):
        """실로봇 시작용 자기위치 재추정. AMCL 전역 파티클을 흩뿌린 뒤, 제자리에서
        'AMCL 공분산이 임계 이하로 수렴할 때까지'(상한 relocalize_max_turns) 회전한다.
        고정 바퀴수와 달리 수렴을 보장해, 이후 confirm 이 요구하는 정밀도를 맞춘다.
        (SLAM 매핑은 위치 고정이라 불필요 → 런타임/AMCL 전용.)"""
        # 1) 전역 재초기화(서비스 있으면): 파티클을 맵 전체에 흩뿌려 잘못된 초기포즈를 버린다.
        if self.global_loc.wait_for_service(timeout_sec=3.0):
            self.global_loc.call_async(Empty.Request())
            self.get_logger().info('AMCL 전역 재초기화 요청(파티클 분산)')
            self._amcl_cov = None   # 분산 직후 옛 공분산으로 조기수렴 오판 방지
        else:
            self.get_logger().warn('전역 재초기화 서비스 없음 — 회전만으로 수렴 시도')
        # 2) 공분산이 임계 이하로 떨어질 때까지 회전(최대 relocalize_max_turns 바퀴)
        max_dur = self.relocalize_max_turns * 2.0 * math.pi / max(0.05, self.relocalize_speed)
        self.get_logger().info(
            f'자기위치 추정 회전 — 수렴까지(상한 {self.relocalize_max_turns:.1f}바퀴/'
            f'~{max_dur:.0f}s, 임계 pos<{self.relocalize_pos_std}m yaw<{self.relocalize_yaw_std}rad)')
        twist = Twist()
        twist.angular.z = self.relocalize_speed
        end = time.time() + max_dur
        converged = False
        last_log = 0.0
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            c = self._amcl_cov
            now = time.time()
            if c is not None and now - last_log > 2.0:   # 수렴 추이 관찰용
                last_log = now
                ps = max(math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)))
                ys = math.sqrt(max(c[35], 0.0))
                self.get_logger().info(f'  수렴 중… pos_std={ps:.3f}m yaw_std={ys:.3f}rad')
            if self._amcl_converged():
                converged = True
                break
        self.cmd_pub.publish(Twist())   # 정지
        c = self._amcl_cov
        if converged and c is not None:
            ps = max(math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)))
            ys = math.sqrt(max(c[35], 0.0))
            self.get_logger().info(
                f'AMCL 수렴 완료(pos_std={ps:.3f}m, yaw_std={ys:.3f}rad) → 순회 시작')
        else:
            self.get_logger().warn(
                '최대 회전까지 공분산 임계 미달 — 그대로 진행(confirm 실패 가능). '
                'relocalize_max_turns 를 늘리거나 맵/스캔 품질 점검 권장.')

    # ── 서비스 루프 ────────────────────────────────────────────────
    def serve(self):
        """이벤트 구동 진입점. (1) 실로봇이면 relocalize 1회, (2) 초기 색이 있으면 첫
        미션으로 큐잉, (3) 이후 /target_color 를 받을 때마다 그 색을 순회한다.
        oneshot=true 면 첫 순회 후 종료(단발 데모)."""
        if self.relocalize:           # 자기위치는 색과 무관 → 시작 시 한 번만
            self.relocalize_in_place()
        if self.target is not None:   # 파라미터로 초기 색을 줬으면 첫 미션 예약
            self.pending_color = self.target
            self.pending_digit = self.target_digit
        self.get_logger().info(
            'maze_tour 대기 — /target_color 로 색 지정 (RED/GREEN/BLUE)')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.pending_color is None:
                continue
            self.target = self.pending_color
            self.target_digit = self.pending_digit
            self.pending_color = None
            self.pending_digit = None
            self.run_tour()
            if self.oneshot:
                self.get_logger().info('oneshot=true → 순회 1회 후 종료')
                return
            self.get_logger().info('=== 다음 색 대기 (/target_color) ===')

    # ── 한 색 순회 ─────────────────────────────────────────────────
    def run_tour(self):
        self.get_logger().info(f'=== 색벽 순회 시작: target = {self.target} ===')
        self.pub_done.publish(Bool(data=False))   # 새 미션 시작 → done 리셋

        walls = self.load_target_walls()
        if not walls:
            # AC7: no-match → 메시지 출력 + 무이동 + /maze_done 미발행
            self.get_logger().warn(
                f'[no-match] 색맵에 {self.target} 벽이 없음 — 움직이지 않음. '
                f'({self.landmarks_path})')
            print(f'NO MATCH: no {self.target} wall in annotation; staying put.')
            return False

        rxy = self.get_robot_xy()
        if rxy is None:
            self.get_logger().error('로봇 위치(TF map->base_link) 못 받음. AMCL 가동 확인.')
            return False

        kor = KOR.get(self.target, self.target)
        order = order_walls(walls, rxy)   # 방문 순서(최근접). id 는 벽 고유 신원(별개).
        ids = ', '.join(f'{kor} {w["id"]}번' for w in order)
        self.get_logger().info(
            f'{kor} 벽 {len(order)}개 순회 예정 (방문순서: {ids}, '
            f'시작 {rxy[0]:.2f},{rxy[1]:.2f})')

        confirmed = []          # [(id, x, y, ax, ay, yaw), ...] 확인된 벽
        failed = []             # [(id, 사유), ...] 접근/확인 실패한 벽(존재하면 미션 실패)
        for w in order:
            wid = w['id']
            ax, ay, yaw = approach_pose(w['x'], w['y'], self.standoff)
            label = f'{kor} {wid}번 ({w["x"]:.2f},{w["y"]:.2f})'
            if not self.nav_to(ax, ay, yaw, label):
                self.get_logger().error(f'{kor} {wid}번 접근 실패')
                failed.append((wid, '접근'))
                continue
            self.get_logger().info(f'{kor} {wid}번에 도착했습니다')
            if self.await_confirmation():
                self.get_logger().info(f'{kor} {wid}번 확인({CONFIRM_THRESHOLD:.0%} 이상)')
                confirmed.append((wid, w['x'], w['y'], ax, ay, yaw))
            else:
                self.get_logger().error(f'{kor} {wid}번 {CONFIRM_THRESHOLD:.0%} 확인 실패')
                failed.append((wid, '확인'))

        # 엄격 완료: target 색 '모든' 벽이 확인돼야 미션 완료(사양 'after confirming all').
        # 확인/접근 실패 벽이 하나라도 있으면 정상 상태가 아님 → 에스컬레이션, /maze_done 미발행.
        # (그런 벽이 생긴다는 건 보통 매핑/SLAM·랜드마크 품질 문제이므로 매핑을 다시 제대로 할 것.)
        if failed:
            detail = ', '.join(f'{kor} {fid}번({why})' for fid, why in failed)
            self.get_logger().error(
                f'=== 미션 실패: {len(confirmed)}/{len(order)}개만 확인, '
                f'미확인 [{detail}] — /maze_done 미발행. '
                f'매핑/랜드마크를 점검해 모든 {kor} 벽이 잡히도록 재매핑 권장. ===')
            return False

        # 전부 확인됨 → 마지막(=방문 순서상 마지막) 확인 벽에서 정지.
        last_id, last_x, last_y, ax, ay, yaw = confirmed[-1]
        here = self.get_robot_xy(timeout=2.0) or (ax, ay)
        if math.hypot(here[0] - ax, here[1] - ay) > 0.3:
            self.get_logger().info(f'마지막 확인 벽({kor} {last_id}번)으로 복귀 후 정지')
            self.nav_to(ax, ay, yaw, f'{kor} {last_id}번')

        self.pub_done.publish(Bool(data=True))
        self.get_logger().info(
            f'=== 완료: {kor} 벽 {len(confirmed)}개 전부 확인, '
            f'마지막 확인 벽 {kor} {last_id}번 ({last_x:.2f},{last_y:.2f})에서 정지 '
            f'→ /maze_done ===')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MazeTour()
    try:
        node.serve()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
