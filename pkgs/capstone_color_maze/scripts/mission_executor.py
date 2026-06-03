#!/usr/bin/env python3
"""
mission_executor.py
색상 시맨틱맵(color_landmarks.yaml)을 이용한 미션 자율주행.  [Phase 4]

  ⚠️ DEPRECATED (구 사양): 이 노드는 '가장 가까운 한 벽만 들렀다가 출구(-1.5,-1.5)로
     복귀'하는 *수정 이전* 동작이다. 현재 사양에는 출구가 없고, target 색의 '모든' 벽을
     순회하며 각 벽을 60%로 확인한 뒤 '마지막 확인 벽'에서 정지하고 /maze_done 을
     발행한다. 런타임은 maze_tour.py 를 사용하라. 이 파일은 참고용으로만 남겨둔다.

미션: 입력 색(예 "RED") 을 받으면
  1) 색맵에서 로봇과 가장 가까운 그 색 셀(벽)을 고르고
  2) 그 벽 앞 '접근 포즈'(벽에서 standoff 만큼 떨어져 벽을 바라봄)로 Nav2 주행 → 잠깐 정찰("들렀다")
  3) exit 지점(기본 시작점 -1.5,-1.5) 으로 복귀 주행

전제 스택(미리 실행되어 있어야 함):
  - mapping.launch.py  (gazebo + slam_toolbox, /map 과 TF map->odom 제공)
  - nav2_bringup navigation_launch.py use_sim_time:=true  (navigate_to_pose 액션)

실행:
  source /opt/ros/humble/setup.bash
  source /home/user/Workspace/turtlebot3_ws/install/setup.bash
  python3 mission_executor.py RED
  # 색은 인자 또는 파라미터로:  python3 mission_executor.py --ros-args -p target_color:=BLUE
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

import yaml
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('target_color', '')         # 빈값이면 argv 에서 읽음
        self.declare_parameter('landmarks_path',
            '/home/user/workspace/ros2_project/capstone_color_maze/maps/color_landmarks.yaml')
        self.declare_parameter('exit_x', -1.5)
        self.declare_parameter('exit_y', -1.5)
        self.declare_parameter('standoff', 0.45)            # 벽 앞 정지 거리 [m]
        self.declare_parameter('visit_sec', 3.0)            # 벽 앞 정찰 시간 [s]
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.landmarks_path = self.get_parameter('landmarks_path').value
        self.exit_x = float(self.get_parameter('exit_x').value)
        self.exit_y = float(self.get_parameter('exit_y').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.visit_sec = float(self.get_parameter('visit_sec').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 목표 색: 파라미터 > argv
        self.target = (self.get_parameter('target_color').value or '').upper()
        if not self.target:
            for a in sys.argv[1:]:
                if not a.startswith('-') and a.upper() in ('RED', 'GREEN', 'BLUE'):
                    self.target = a.upper()
                    break
        if not self.target:
            self.target = 'RED'

        # TF + Nav2 액션
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # ── 유틸 ──────────────────────────────────────────────────────
    def load_landmarks(self):
        with open(self.landmarks_path) as f:
            data = yaml.safe_load(f) or {}
        return data.get(self.target) or []

    def get_robot_xy(self, timeout=10.0):
        """TF map->base_link 로 현재 위치 (x,y). 준비될 때까지 잠깐 spin."""
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

    def approach_pose(self, wall_x, wall_y):
        """벽 점에서 미로 중심(0,0) 쪽(자유공간)으로 standoff 떨어진 접근 포즈.
        yaw 는 벽을 바라보도록(=중심 반대 방향)."""
        # 중심 방향 단위벡터(자유공간 쪽). 벽이 중심이면 임의 방향.
        vx, vy = -wall_x, -wall_y
        n = math.hypot(vx, vy)
        if n < 1e-3:
            vx, vy, n = 1.0, 0.0, 1.0
        ux, uy = vx / n, vy / n
        ax, ay = wall_x + self.standoff * ux, wall_y + self.standoff * uy
        yaw = math.atan2(wall_y - ay, wall_x - ax)   # 접근점→벽 (벽을 바라봄)
        return ax, ay, yaw

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation = yaw_to_quat(yaw)
        return p

    def nav_to(self, x, y, yaw, label):
        """navigate_to_pose 동기 호출. 성공 여부 반환."""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 액션 서버 없음 (Nav2 미실행?)')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        self.get_logger().info(f'[{label}] 주행 시작 → ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')

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

    # ── 미션 ──────────────────────────────────────────────────────
    def run(self):
        self.get_logger().info(f'=== 미션 시작: 목표 색 = {self.target} ===')

        cells = self.load_landmarks()
        if not cells:
            self.get_logger().error(f'색맵에 {self.target} 셀이 없음: {self.landmarks_path}')
            return False

        rxy = self.get_robot_xy()
        if rxy is None:
            self.get_logger().error('로봇 위치(TF map->base_link) 못 받음. SLAM 가동 확인.')
            return False
        rx, ry = rxy

        # 로봇과 가까운 순으로 후보 정렬 → 접근 성공할 때까지 폴백
        cand = sorted(cells, key=lambda c: math.hypot(c['x'] - rx, c['y'] - ry))
        reached = False
        for i, c in enumerate(cand):
            ax, ay, yaw = self.approach_pose(c['x'], c['y'])
            self.get_logger().info(
                f'후보 {i+1}/{len(cand)}: {self.target} 벽 ({c["x"]:.2f},{c["y"]:.2f}) '
                f'votes={c.get("votes","?")} → 접근점 ({ax:.2f},{ay:.2f})')
            if self.nav_to(ax, ay, yaw, f'{self.target} 벽 접근'):
                reached = True
                break
            self.get_logger().warn('접근 실패 → 다음 후보로')

        if not reached:
            self.get_logger().error(f'{self.target} 벽 접근 모두 실패. 미션 중단.')
            return False

        # 벽 앞 정찰("들렀다")
        self.get_logger().info(f'{self.target} 벽 앞 도착 — {self.visit_sec:.0f}초 정찰')
        t_end = time.time() + self.visit_sec
        while time.time() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

        # exit 복귀 (스폰셀은 map에서 UNKNOWN 이라 도달불가 → 중심 쪽으로 당겨 폴백)
        exit_yaw = math.atan2(-self.exit_y, -self.exit_x)   # 중심을 바라보며 정지
        for ex, ey in self.exit_candidates():
            if self.nav_to(ex, ey, exit_yaw, 'EXIT 복귀'):
                if (ex, ey) != (self.exit_x, self.exit_y):
                    self.get_logger().info(
                        f'(스타트셀 미관측 → 도달가능한 ({ex:.2f},{ey:.2f}) 로 복귀)')
                self.get_logger().info('=== 미션 완료: 탈출 지점 도착 ===')
                return True
            self.get_logger().warn('EXIT 후보 실패 → 중심 쪽 다음 후보로')
        self.get_logger().error('EXIT 복귀 모두 실패.')
        return False

    def exit_candidates(self):
        """요청 exit 부터 중심(0,0) 방향으로 0.1m 씩 당긴 후보들(최대 0.6m)."""
        dx, dy = -self.exit_x, -self.exit_y
        n = math.hypot(dx, dy)
        ux, uy = (dx / n, dy / n) if n > 1e-3 else (0.0, 0.0)
        pts = []
        step = 0.0
        while step <= 0.6 + 1e-6:
            pts.append((self.exit_x + ux * step, self.exit_y + uy * step))
            step += 0.1
        return pts


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
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
