#!/usr/bin/env python3
"""
maze_tour.py
색상 시맨틱맵(color_landmarks.yaml)을 이용한 '색벽 순회 + 마지막 벽 정지' 런타임.
[Phase 4 — 수정 사양]

  ※ 이 노드는 mission_executor.py(구 사양: 가장 가까운 한 벽만 들렀다가 '출구'로 복귀)를
    대체한다. 현재 사양에는 출구가 없다 — target_color 의 '모든' 벽을 순회하며 각 벽을
    카메라로 확인하고, '마지막으로 확인한 벽'에서 정지한 뒤 /maze_done 을 발행한다.

미션:
  1) color_landmarks.yaml 에서 target_color 의 모든 벽을 계산한다.
     - 없으면: no-match 메시지를 출력하고 '움직이지 않고' 종료(/maze_done 미발행).   [AC7]
  2) 현재 위치에서 nearest-neighbor 로 방문 순서를 정한다.
  3) 각 벽의 '접근 포즈'로 Nav2 주행 → color_confirm 의 /target_confirmed(>=60%)로 확인.
  4) 모든 벽을 처리한 뒤 '마지막으로 확인된 벽'에 머무르고 /maze_done(True) 발행.   [AC6]
     - 한 벽도 확인 못 하면: 부분 결과로 처리하고 /maze_done 을 발행하지 않는다.

전제 스택(미리 실행):
  - 저장된 맵 + map_server + AMCL  (TF map->odom, /odom 제공)
  - nav2 (navigate_to_pose 액션)
  - color_confirm.py  (같은 target_color 로 /target_confirmed 발행)

실행:
  ros2 run ... 또는
  python3 maze_tour.py --ros-args -p target_color:=RED
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
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from maze_common import (
    normalize_color, approach_pose, order_walls, resolve_target_walls,
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
        self.declare_parameter('target_color', 'RED')
        self.declare_parameter('landmarks_path', default_landmarks_path())
        self.declare_parameter('standoff', 0.45)        # 벽 앞 정지 거리 [m]
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('confirm_window', 4.0)   # 도착 후 확인 관측 시간 [s]
        self.declare_parameter('confirm_min_true', 3)   # 이 횟수 이상 True 면 확인

        self.target = normalize_color(self.get_parameter('target_color').value)
        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.confirm_window = float(self.get_parameter('confirm_window').value)
        self.confirm_min_true = int(self.get_parameter('confirm_min_true').value)

        # ── 상태/IO ───────────────────────────────────────────────
        self._confirmed_now = False        # /target_confirmed 최신값
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Bool, '/target_confirmed', self._on_confirmed, 10)

        # /maze_done 은 늦게 접속한 구독자도 받도록 latched(transient_local)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_done = self.create_publisher(Bool, '/maze_done', latched)

    # ── 콜백/유틸 ─────────────────────────────────────────────────
    def _on_confirmed(self, msg):
        self._confirmed_now = bool(msg.data)

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
        """도착 후 confirm_window 초간 /target_confirmed 를 관측해 확인 여부 판정.
        True 표본이 confirm_min_true 이상이면 확인된 것으로 본다(스파이크 방지)."""
        self._confirmed_now = False
        true_count = 0
        end = time.time() + self.confirm_window
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._confirmed_now:
                true_count += 1
                if true_count >= self.confirm_min_true:
                    return True
        return true_count >= self.confirm_min_true

    # ── 미션 ──────────────────────────────────────────────────────
    def run(self):
        if self.target is None:
            self.get_logger().error(
                f"target_color 가 RED/GREEN/BLUE 중 하나가 아님: "
                f"{self.get_parameter('target_color').value!r}")
            return False

        self.get_logger().info(f'=== 색벽 순회 시작: target = {self.target} ===')

        walls = self.load_target_walls()
        if not walls:
            # AC7: no-match → 메시지 출력 + 무이동 + /maze_done 미발행
            self.get_logger().warn(
                f'[no-match] 색맵에 {self.target} 벽이 없음 — 움직이지 않고 종료. '
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
                self.get_logger().info(f'{kor} {wid}번 확인(60% 이상)')
                confirmed.append((wid, w['x'], w['y'], ax, ay, yaw))
            else:
                self.get_logger().error(f'{kor} {wid}번 60% 확인 실패')
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
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
