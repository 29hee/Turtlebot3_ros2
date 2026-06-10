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
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

import yaml
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32, String, Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose

from maze_common import (
    normalize_color, parse_target, approach_pose, order_walls, resolve_target_walls,
    color_to_id, CONFIRM_THRESHOLD,
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
        # 벽 앞 Nav2 정지 거리 [m]. 정면접근(visual servo)이 여기서부터 전방가드(0.5m)까지
        # 더 다가가며 점유율을 올리므로, standoff 는 가드보다 충분히 멀어야 전진 여유가 생긴다.
        self.declare_parameter('standoff', 0.70)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('confirm_window', 4.0)   # 도착 후 확인 관측 시간 [s]
        self.declare_parameter('confirm_min_true', 3)   # 이 횟수 이상 True 면 확인
        self.declare_parameter('confirm_retries', 2)         # 확인 실패 시 '뒤로+재확인' 반복 횟수
        self.declare_parameter('confirm_backup_speed', 0.07) # 뒤로 물러나는 속도 [m/s]
        self.declare_parameter('confirm_backup_secs', 1.5)   # 뒤로 물러나는 시간 [s] (~0.1m, 화각 확보)
        self.declare_parameter('dwell_secs', 10.0)           # 타겟 앞 정지 관찰 시간 [s] (인식 무관 머묾)
        self.declare_parameter('map_margin', 0.8)            # 랜드마크 bbox + 이 여유 밖이면 '맵 밖' → 중앙 재시드 [m]
        # ── 정면 정렬·전진(visual servo) ─────────────────────────────
        # 도착 후 Nav2 가 세워준 포즈가 비스듬하면 색 패널을 '옆에서' 봐 점유율이 낮다.
        # /color_signal 의 cx 로 정면을 맞추며, target 색의 '화면 전체(full-frame) 점유율'
        # (/target_coverage)이 approach_coverage(목표) 이상이 될 때까지 천천히 전진해
        # '정면에서 꽉 차게' 만든다. detect 임계는 '색이 보이기 시작'하는 하한 —
        # 그 아래면 전진 대신 탐색 회전으로 색을 화면에 잡는다.
        self.declare_parameter('approach_coverage', 0.50)    # 전체프레임 점유율 목표(도달 시 정지) [0~1]
        self.declare_parameter('approach_detect', 0.05)      # 전체프레임 이 점유율 이상이면 '감지'→전진(이하=탐색 회전)
        self.declare_parameter('approach_speed', 0.08)       # 전진 속도 [m/s]
        self.declare_parameter('approach_kp', 0.6)           # cx→조향 P 게인(정면 정렬)
        self.declare_parameter('approach_search_speed', 0.3) # 색 안 보일 때 탐색 회전 각속도 [rad/s]
        self.declare_parameter('approach_max_secs', 8.0)     # 전진 시간 상한 [s]
        self.declare_parameter('approach_max_advance', 0.45) # 도착점 대비 전진 거리 상한(벽 지나침 방지) [m]
        self.declare_parameter('approach_min_range', 0.50)   # LIDAR 전방 최소거리 — 이하면 정지(충돌 방지) [m]
        self.declare_parameter('approach_front_deg', 20.0)   # 전방 가드 섹터 반각 [deg]
        # ── 벽 정렬(모든 타겟 도착 후, 마지막 벽에 수직 맞춤) ─────────────
        # 전방 ±align_probe_deg 두 빔의 거리를 비교해 같아질 때까지 회전 → 평평한 벽에 수직(정면).
        self.declare_parameter('align_on_finish', True)      # 순회 완료 후 벽 정렬 수행 여부
        self.declare_parameter('align_probe_deg', 25.0)      # 좌우 대칭 비교 빔 각도 [deg]
        self.declare_parameter('align_tol', 0.03)            # 좌우 거리차 허용 [m] (이하면 수직으로 봄)
        self.declare_parameter('align_kp', 1.2)              # 거리차→각속도 P 게인
        self.declare_parameter('align_speed', 0.4)           # 정렬 회전 최대 각속도 [rad/s]
        self.declare_parameter('align_max_secs', 6.0)        # 정렬 시간 상한 [s]
        # ── 측면 재정렬(옆에서 보는 문제 해결) ───────────────────────────
        # 패널을 옆에서 보면(가로 오프셋 cx 큼), 회전만으론 정면이 안 된다. 벽과 평행하게
        # 타겟 쪽으로 '일정거리' 이동해 패널 정면으로 들어간 뒤 다시 벽을 향해 돌아 정면으로 본다.
        # diff-drive 라 평행이동 = 90° 회전 → 전진 → -90° 복귀. cx 가 tol 이내 될 때까지 반복.
        self.declare_parameter('recenter_enabled', True)
        self.declare_parameter('recenter_tol', 0.12)         # |cx| 이 이하면 '정면'으로 보고 종료
        self.declare_parameter('recenter_step', 0.10)        # 한 번에 벽 평행으로 이동할 거리 [m]
        self.declare_parameter('recenter_max_iters', 5)      # 평행이동 반복 상한
        self.declare_parameter('recenter_fwd_speed', 0.08)   # 평행이동 전진 속도 [m/s]
        self.declare_parameter('recenter_sign', 1.0)         # 카메라 좌우 반전 시 -1.0 로 뒤집기
        self.declare_parameter('recenter_turn_tol_deg', 3.0) # 90° 회전 허용 오차 [deg]
        # 시작 시 자기위치 재추정(relocalization). SLAM 매핑 땐 불필요(위치 고정)하고,
        # 실로봇 런타임에선 켠 위치가 맵 어디인지 모르므로 true 로 켜서, 전역 파티클을
        # 흩뿌린 뒤 제자리 회전으로 AMCL 을 수렴시키고 나서 순회를 시작한다.
        # (시뮬은 set_initial_pose 가 맞으므로 기본 false 로 두어 기존 동작 보존.)
        self.declare_parameter('relocalize', False)
        # 회전 각속도 [rad/s]. ★ 느릴수록 매 AMCL 업데이트(update_min_a=0.2rad)당 주입되는
        #   yaw 불확실성이 작아 'yaw 공분산 plateau'가 낮아진다 → 더 엄격한 yaw 임계까지 수렴 가능.
        #   (0.5→0.4 로 낮춰 plateau 를 끌어내림. 대신 같은 바퀴수에 시간이 더 걸림.)
        self.declare_parameter('relocalize_speed', 0.4)
        # 고정 바퀴수가 아니라 'AMCL 공분산이 임계 이하로 수렴할 때까지' 회전한다.
        # relocalize_max_turns 는 수렴 못 할 때를 대비한 안전 상한(무한회전 방지).
        # ※ AMCL 은 update_min_d=0.25m / update_min_a=0.2rad 이상 '움직일 때만' 필터를 갱신한다
        #   (정지 상태에선 공분산이 안 줄어듦). 그래서 수렴은 '회전 중'에만 진행된다.
        # ★ 엄격화: 회전 plateau(@0.4rad/s) 바로 위로 임계를 더 조였다(pos 0.25→0.18, yaw 0.35→0.26).
        #   plateau 에 가까울수록 트립이 빡빡해지므로, max_turns 여유(3→5)와 지속시간(1.5→3.0s)을
        #   함께 키워 '진짜로 좁아졌을 때만' 통과시킨다. 상한까지 미달이면 경고 후 진행(무한회전 방지).
        #   너무 빡빡해 매번 상한까지 헛돌면 임계를 0.20/0.30 정도로 살짝 푸는 것을 권장.
        self.declare_parameter('relocalize_max_turns', 5.0)  # 최대 회전 바퀴 수(상한)
        self.declare_parameter('relocalize_pos_std', 0.18)   # 위치 표준편차 임계 [m] (엄격)
        self.declare_parameter('relocalize_yaw_std', 0.26)   # yaw 표준편차 임계 [rad] (엄격)
        # 임계 아래로 '한 번 튄' 샘플에 속아 조기 종료하지 않도록, 임계 이하가 이 시간만큼
        # '연속 유지'돼야 수렴으로 인정한다(대칭 미로에서 확신하지만 틀린 수렴 줄임).
        self.declare_parameter('relocalize_settle_secs', 3.0)
        self.declare_parameter('goto_center', True)          # 위치추정 시 맵 중앙으로 실제 주행 후 재수렴

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
        self.confirm_retries = int(self.get_parameter('confirm_retries').value)
        self.confirm_backup_speed = float(self.get_parameter('confirm_backup_speed').value)
        self.confirm_backup_secs = float(self.get_parameter('confirm_backup_secs').value)
        self.dwell_secs = float(self.get_parameter('dwell_secs').value)
        self.map_margin = float(self.get_parameter('map_margin').value)
        self.approach_coverage = float(self.get_parameter('approach_coverage').value)
        self.approach_detect = float(self.get_parameter('approach_detect').value)
        self.approach_speed = float(self.get_parameter('approach_speed').value)
        self.approach_kp = float(self.get_parameter('approach_kp').value)
        self.approach_search_speed = float(self.get_parameter('approach_search_speed').value)
        self.approach_max_secs = float(self.get_parameter('approach_max_secs').value)
        self.approach_max_advance = float(self.get_parameter('approach_max_advance').value)
        self.approach_min_range = float(self.get_parameter('approach_min_range').value)
        self.approach_front_deg = float(self.get_parameter('approach_front_deg').value)
        self.align_on_finish = bool(self.get_parameter('align_on_finish').value)
        self.align_probe_deg = float(self.get_parameter('align_probe_deg').value)
        self.align_tol = float(self.get_parameter('align_tol').value)
        self.align_kp = float(self.get_parameter('align_kp').value)
        self.align_speed = float(self.get_parameter('align_speed').value)
        self.align_max_secs = float(self.get_parameter('align_max_secs').value)
        self.recenter_enabled = bool(self.get_parameter('recenter_enabled').value)
        self.recenter_tol = float(self.get_parameter('recenter_tol').value)
        self.recenter_step = float(self.get_parameter('recenter_step').value)
        self.recenter_max_iters = int(self.get_parameter('recenter_max_iters').value)
        self.recenter_fwd_speed = float(self.get_parameter('recenter_fwd_speed').value)
        self.recenter_sign = float(self.get_parameter('recenter_sign').value)
        self.recenter_turn_tol_deg = float(self.get_parameter('recenter_turn_tol_deg').value)
        self.relocalize = bool(self.get_parameter('relocalize').value)
        self.goto_center = bool(self.get_parameter('goto_center').value)
        self.relocalize_speed = float(self.get_parameter('relocalize_speed').value)
        self.relocalize_max_turns = float(self.get_parameter('relocalize_max_turns').value)
        self.relocalize_pos_std = float(self.get_parameter('relocalize_pos_std').value)
        self.relocalize_yaw_std = float(self.get_parameter('relocalize_yaw_std').value)
        self.relocalize_settle_secs = float(self.get_parameter('relocalize_settle_secs').value)

        # ── 상태/IO ───────────────────────────────────────────────
        self._confirmed_now = False        # /target_confirmed 최신값
        self._detected_digit = -1          # /detected_digit 최신값 (-1=없음)
        # 특정 숫자 모드에서 '저장된 landmark digit' 으로 목표를 정한 경우 True.
        #   이때는 라이브 OCR(/detected_digit) 없이 색 확인만으로 도착을 인정한다
        #   (런타임에 숫자가 잠깐 안 읽혀도 매핑 때 확정한 digit 을 신뢰).
        self.target_digit_known = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Bool, '/target_confirmed', self._on_confirmed, 10)
        self.create_subscription(Int32, '/detected_digit', self._on_digit, 10)
        # 정면 정렬·전진용: 점유율 게이트는 화면 전체 기준(/target_coverage, color_confirm),
        # 좌우 정렬(cx)은 vision_node 의 /color_signal[ color_id, cx_norm, coverage ] 사용.
        self._full_cov = 0.0     # target 색 '화면 전체' 점유율(color_confirm)
        self._color_cx = 0.0     # target 색 blob 중심 x(-1 왼쪽 ~ +1 오른쪽)
        self._cx_stamp = 0.0     # _color_cx 가 target 색으로 갱신된 마지막 시각(신선도)
        self._scan_front = float('inf')   # LIDAR 전방 섹터 최소거리 [m]
        self._scan_msg = None    # 최신 LaserScan(벽 정렬에서 특정 각도 빔 조회용)
        self.create_subscription(Float32, '/target_coverage', self._on_coverage, 10)
        self.create_subscription(Float32MultiArray, '/color_signal', self._on_signal, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        # relocalization 용: 제자리 회전 명령 + AMCL 전역 재초기화 서비스 + 공분산 모니터
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._seed_center = (0.0, 0.0)   # 맵 중앙(무적 재시드 기준)
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

    def _on_coverage(self, msg):
        """color_confirm /target_coverage = target 색의 '화면 전체' 점유율(0~1). 전진 게이트용."""
        self._full_cov = float(msg.data)

    def _on_signal(self, msg):
        """vision_node /color_signal = [color_id, cx_norm, coverage]. 좌우 정렬용 cx 만 취한다.
        우세색이 현재 target 색일 때만 cx 갱신(아니면 마지막 본 방향 유지 → 탐색 회전에 활용)."""
        if len(msg.data) < 3:
            return
        if self.target is not None and int(msg.data[0]) == color_to_id(self.target):
            self._color_cx = float(msg.data[1])
            self._cx_stamp = time.time()   # cx 신선도(측면정렬에서 미검출 구분용)

    def _on_scan(self, msg):
        """전방 ±approach_front_deg 섹터의 LIDAR 최소거리(유효 측정만). 충돌 방지 가드용.
        원본 msg 도 보관(벽 정렬에서 특정 각도 빔 거리 조회)."""
        self._scan_msg = msg
        n = len(msg.ranges)
        if n == 0:
            return
        half = math.radians(self.approach_front_deg)
        best = float('inf')
        for i in range(n):
            ang = msg.angle_min + i * msg.angle_increment
            ang = (ang + math.pi) % (2.0 * math.pi) - math.pi   # [-pi, pi] 정규화
            if -half <= ang <= half:
                r = msg.ranges[i]
                if r == r and msg.range_min <= r < msg.range_max and r < best:  # NaN/무한/범위밖 제외
                    best = r
        self._scan_front = best

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
        # AMCL 이 초기화 직후 '공분산 0짜리'(아직 추정 전) /amcl_pose 를 한 번 내보내는데,
        # 그게 0 <= 임계라 '수렴'으로 오판돼 첫 회전을 통째로 건너뛰는 버그가 있었다.
        # → 위치·yaw 가 둘 다 사실상 0 이면 '유효한 추정 아님'으로 보고 수렴으로 치지 않는다.
        if pos_std <= 1e-6 and yaw_std <= 1e-6:
            return False
        return pos_std <= self.relocalize_pos_std and yaw_std <= self.relocalize_yaw_std

    def load_target_walls(self):
        """원시 셀을 클러스터링/필터해 '진짜 벽'(각 벽에 안정 id 부여)으로 반환.
        파일이 없거나 깨져도 노드를 죽이지 않고 빈 리스트(=no-match) 로 안전 처리."""
        try:
            with open(self.landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().error(
                f'색맵 파일 없음: {self.landmarks_path} — 먼저 매핑을 돌려 생성할 것')
            return []
        except Exception as e:
            self.get_logger().error(f'색맵 읽기 실패({e}) — no-match 처리')
            return []
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

    def get_robot_yaw(self, timeout=2.0):
        """map→base_link 의 yaw[rad]. 못 받으면 None. (제자리 회전량 측정용)"""
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                q = tf.transform.rotation
                return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
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
        self._ensure_in_map()   # ★ 무적: 출발 추정이 맵 밖이면 중앙 재시드(planner 'outside map' 차단)
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

    def _nudge_back(self):
        """확인 실패 시 조금 뒤로 물러나 카메라 화각 확보(패널이 너무 가까워 안 잡힐 때).
        Nav2 목표는 이미 끝난 상태라 cmd_vel 직접 발행으로 짧게 후진한다."""
        t = Twist()
        t.linear.x = -abs(self.confirm_backup_speed)
        end = time.time() + self.confirm_backup_secs
        while time.time() < end and rclpy.ok():
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())   # 정지

    def await_confirmation(self):
        """도착 후 confirm_window 초간 색(+숫자) 확인.
        - target_digit 없음: 색 확인만 (기존 동작)
        - target_digit 있고 저장 digit 으로 목표 선정(target_digit_known): 색 확인만
          (숫자는 매핑 때 확정됐으므로 라이브 OCR 불필요)
        - target_digit 있고 저장 digit 없음: 색 확인 AND 라이브 숫자 일치 둘 다 만족"""
        need_live_digit = (self.target_digit is not None
                           and not self.target_digit_known)
        self._confirmed_now = False
        true_count = 0
        end = time.time() + self.confirm_window
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            color_ok = self._confirmed_now
            digit_ok = (not need_live_digit or
                        self._detected_digit == self.target_digit)
            if color_ok and digit_ok:
                true_count += 1
                if true_count >= self.confirm_min_true:
                    return True
        return true_count >= self.confirm_min_true

    def _set_initial_pose(self, x, y, yaw=0.0):
        """AMCL 초기 추정을 (x,y,yaw)에 심는다(/initialpose). 맵 안 좌표라 'outside map' 차단.
        AMCL 이 한두 번 놓칠 수 있어 짧게 여러 번 발행.

        ★ stamp 는 '0'(빈 시각)으로 둔다. PC 시각 now() 로 찍으면 odom TF(로봇/Pi 시각)와
        클럭 skew 가 나서 AMCL 이 'extrapolation into the future' 로 초기포즈 변환에 실패하고
        odom 보정 없이 대충 박힌다(로그의 'Failed to transform initial pose in time'). 0 으로
        두면 AMCL 이 '최신 TF' 로 변환 → skew 무관하게 제대로 적용된다(/scan 의 scan_restamp 와 같은 취지)."""
        for _ in range(5):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = self.map_frame
            msg.header.stamp = rclpy.time.Time().to_msg()   # 0: 최신 TF 사용(클럭 skew 무시)
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation = yaw_to_quat(yaw)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.25    # 위치 분산(0.5m)^2
            cov[35] = 0.25            # yaw 분산
            msg.pose.covariance = cov
            self.init_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._amcl_cov = None
        self.get_logger().info(f'AMCL 초기포즈 심음 @({x:.2f},{y:.2f}) — 절대 맵 밖 아님')

    def _map_bbox(self):
        """랜드마크 전체의 (xmin,ymin,xmax,ymax). 없으면 None."""
        try:
            with open(self.landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return None
        xs, ys = [], []
        for walls in data.values():
            for w in (walls or []):
                if 'x' in w and 'y' in w:
                    xs.append(float(w['x'])); ys.append(float(w['y']))
        if not xs:
            return None
        return (min(xs), min(ys), max(xs), max(ys))

    def _ensure_in_map(self):
        """★ 무적 가드: 로봇 추정이 맵(랜드마크 bbox+여유) 밖으로 발산하면 중앙으로 강제 재시드.
        ('절대 맵 밖일 수 없다' — RViz 로 옮겨도 다시 발산하는 경우 자동 복구.)"""
        bb = self._map_bbox()
        here = self.get_robot_xy(timeout=1.0)
        if bb is None or here is None:
            return
        m = self.map_margin
        if not (bb[0] - m <= here[0] <= bb[2] + m and bb[1] - m <= here[1] <= bb[3] + m):
            self.get_logger().warn(
                f'로봇 추정({here[0]:.1f},{here[1]:.1f})이 맵 밖 — 중앙 재시드(무적 복구)')
            self._set_initial_pose(self._seed_center[0], self._seed_center[1], 0.0)
            self._spin_to_converge()

    def visual_approach(self):
        """도착 후 '정면 정렬 + 전진'(visual servo). Nav2 가 세워준 포즈가 비스듬해
        패널을 옆에서 보면 점유율이 낮으므로, /color_signal 의 cx 로 정면을 맞추며
        target 색의 '화면 전체' 점유율이 approach_coverage(목표) 이상이 될 때까지 전진한다.

        안전 종료:
          - 점유율 목표 도달
          - LIDAR 전방 최소거리 ≤ approach_min_range (벽 코앞 → 충돌 방지)
          - 도착점 대비 전진거리 ≥ approach_max_advance (벽 지나침 방지)
          - 시간 상한 approach_max_secs
        색이 approach_detect 미만이면(거의 안 보임) 전진을 멈추고 마지막 cx 방향으로
        탐색 회전해 색을 화면에 다시 잡는다. 반환: 종료 시 점유율이 목표 이상이면 True."""
        start = self.get_robot_xy(timeout=2.0)
        twist = Twist()
        t_end = time.time() + self.approach_max_secs
        reason = '시간 상한'
        self.get_logger().info(
            f'  정면 정렬·전진 — 전체프레임 점유율 {self.approach_coverage:.0%} 목표 '
            f'(감지 {self.approach_detect:.0%}, 전방가드 {self.approach_min_range:.2f}m)')
        while time.time() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            cov = self._full_cov
            if cov >= self.approach_coverage:
                reason = f'점유율 도달({cov:.0%})'
                break
            if self._scan_front <= self.approach_min_range:
                reason = f'LIDAR 근접({self._scan_front:.2f}m)'
                break
            here = self.get_robot_xy(timeout=0.3)
            if (start and here and
                    math.hypot(here[0] - start[0], here[1] - start[1]) >= self.approach_max_advance):
                reason = '전진거리 상한'
                break
            if cov >= self.approach_detect:
                # 색이 보임 → cx 로 정면 정렬하며 전진(blob 오른쪽이면 우회전)
                twist.linear.x = self.approach_speed
                twist.angular.z = -self.approach_kp * self._color_cx
            else:
                # 색이 거의 안 보임 → 전진 멈추고 마지막 본 방향으로 탐색 회전
                twist.linear.x = 0.0
                twist.angular.z = -math.copysign(self.approach_search_speed, self._color_cx)
            self.cmd_pub.publish(twist)
        self.cmd_pub.publish(Twist())   # 정지
        ok = self._full_cov >= self.approach_coverage
        self.get_logger().info(f'  정면접근 종료({reason}) — 전체프레임 점유율 {self._full_cov:.0%}')
        return ok

    def _range_at(self, msg, angle):
        """전방=0 기준 angle[rad] 빔의 유효 거리(주변 ±1빔 중 최소로 노이즈 완화). 없으면 inf."""
        i = int(round((angle - msg.angle_min) / msg.angle_increment))
        n = len(msg.ranges)
        best = float('inf')
        for di in (-1, 0, 1):
            j = i + di
            if 0 <= j < n:
                r = msg.ranges[j]
                if r == r and msg.range_min <= r < msg.range_max and r < best:
                    best = r
        return best

    def align_to_wall(self):
        """전방 벽에 '수직'이 되도록 제자리 회전(정면 보기). 전방 ±align_probe_deg 두 빔
        거리가 같아지면 평평한 벽에 수직이다. 좌우 거리차로 P 제어해 균형을 맞춘다.
        벽이 안 잡히거나 시간 상한이면 그대로 종료. (모든 타겟 도착 후 마지막 벽에서 호출.)"""
        probe = math.radians(self.align_probe_deg)
        twist = Twist()
        end = time.time() + self.align_max_secs
        reason = '시간 상한'
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self._scan_msg
            if msg is None:
                continue
            rl = self._range_at(msg, +probe)   # 좌측(+, CCW) 빔
            rr = self._range_at(msg, -probe)   # 우측(-, CW) 빔
            if not (math.isfinite(rl) and math.isfinite(rr)):
                reason = '벽 미검출'
                break
            diff = rl - rr                     # >0: 좌측이 더 멂(=좌로 틀어짐) → 우회전 필요
            if abs(diff) <= self.align_tol:
                reason = f'수직 정렬(Δ={diff*100:.1f}cm)'
                break
            wz = -self.align_kp * diff
            twist.angular.z = max(-self.align_speed, min(self.align_speed, wz))
            self.cmd_pub.publish(twist)
        self.cmd_pub.publish(Twist())   # 정지
        self.get_logger().info(f'  벽 정렬 종료({reason})')

    def _rotate_by(self, dyaw):
        """제자리에서 dyaw[rad] 만큼 회전(부호=방향). TF yaw 피드백으로 닫힌 루프."""
        y0 = self.get_robot_yaw()
        if y0 is None:
            return
        target = y0 + dyaw
        tol = math.radians(self.recenter_turn_tol_deg)
        twist = Twist()
        end = time.time() + 8.0
        while time.time() < end and rclpy.ok():
            cur = self.get_robot_yaw(timeout=0.3)
            if cur is None:
                rclpy.spin_once(self, timeout_sec=0.05); continue
            err = math.atan2(math.sin(target - cur), math.cos(target - cur))
            if abs(err) <= tol:
                break
            wz = max(-self.align_speed, min(self.align_speed, 1.5 * err))
            # 너무 느려 멈추지 않게 최소 각속도 보장
            if abs(wz) < 0.15:
                wz = math.copysign(0.15, err)
            twist.angular.z = wz
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())

    def _drive_forward(self, dist):
        """직진으로 dist[m] 만큼 전진(부호=방향). TF 위치 피드백으로 닫힌 루프.
        전방 LIDAR 가드(approach_min_range)도 함께 적용(전진 시 벽 충돌 방지)."""
        start = self.get_robot_xy(timeout=2.0)
        if start is None:
            return
        twist = Twist()
        end = time.time() + 8.0
        while time.time() < end and rclpy.ok():
            here = self.get_robot_xy(timeout=0.3)
            if here and math.hypot(here[0] - start[0], here[1] - start[1]) >= abs(dist):
                break
            if dist > 0 and self._scan_front <= self.approach_min_range:
                self.get_logger().info(f'    전진 가드(LIDAR {self._scan_front:.2f}m) — 중단')
                break
            twist.linear.x = math.copysign(self.recenter_fwd_speed, dist)
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())

    def _fresh_cx(self, secs=0.6):
        """secs 동안 스핀하며 최신 target 색 cx 를 읽는다. 그 사이 target 색이 한 번도
        안 잡혔으면(=신선한 cx 없음) None 을 반환해 '정면(cx=0)'과 '미검출'을 구분한다."""
        t0 = time.time()
        end = t0 + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._color_cx if self._cx_stamp >= t0 else None

    def recenter_on_target(self):
        """옆에서 보는 문제의 '진짜' 해결: 벽과 평행하게 타겟 쪽으로 이동(측면 정렬)해
        패널 정면으로 들어간 뒤, 벽을 향해 돌아 정면으로 본다. cx 가 tol 이내가 될 때까지 반복.
          1) 벽에 수직 정렬(align_to_wall)
          2) cx(가로 오프셋) 확인 — tol 이내면 종료(이미 정면)
          3) 타겟 쪽으로 90° 회전 → recenter_step 만큼 전진(벽 평행이동) → -90° 복귀
          4) 다시 수직 정렬 후 2)로
        diff-drive 라 평행이동을 회전·전진·복귀로 구현. 오프셋이 오히려 커지면(부호 반대)
        recenter_sign 으로 뒤집을 수 있고, 한 번 악화하면 중단해 발산을 막는다."""
        if not self.recenter_enabled:
            return
        self.align_to_wall()
        prev_abs = None
        for it in range(self.recenter_max_iters):
            cx = self._fresh_cx()
            if cx is None:
                self.get_logger().info('  측면정렬: 타겟색 미검출 — 중단')
                return
            if abs(cx) <= self.recenter_tol:
                self.get_logger().info(f'  측면정렬 완료 — 정면(cx={cx:+.2f})')
                return
            if prev_abs is not None and abs(cx) > prev_abs + 0.05:
                self.get_logger().info(
                    f'  측면정렬: 오프셋 악화(cx={cx:+.2f}) — 중단(부호 의심 시 recenter_sign 뒤집기)')
                return
            prev_abs = abs(cx)
            # cx<0=타겟 좌측 → 좌로 평행이동(+90° CCW). recenter_sign 으로 좌우 반전 보정.
            turn = math.copysign(math.pi / 2.0, -cx * self.recenter_sign)
            self.get_logger().info(
                f'  측면정렬 {it+1}/{self.recenter_max_iters}: cx={cx:+.2f} → '
                f'벽 평행 {self.recenter_step:.2f}m {"좌" if turn > 0 else "우"}로 이동')
            self._rotate_by(turn)
            self._drive_forward(self.recenter_step)
            self._rotate_by(-turn)
            self.align_to_wall()
        self.get_logger().info('  측면정렬 반복 상한 — 그대로 진행')

    def dwell_observe(self, secs):
        """secs 동안 '정지'한 채 확인 신호를 관찰(조기 종료 없이 전체 시간 머묾).
        인식되든 안 되든 그 앞에 머물다 이동하기 위함. 확인되면 True."""
        self.cmd_pub.publish(Twist())   # 정지 유지
        self._confirmed_now = False
        seen = 0
        need_live_digit = (self.target_digit is not None and not self.target_digit_known)
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            color_ok = self._confirmed_now
            digit_ok = (not need_live_digit or self._detected_digit == self.target_digit)
            if color_ok and digit_ok:
                seen += 1
        return seen >= self.confirm_min_true

    def relocalize_in_place(self):
        """실로봇 시작용 자기위치 재추정. AMCL 전역 파티클을 흩뿌린 뒤, 제자리에서
        'AMCL 공분산이 임계 이하로 수렴할 때까지'(상한 relocalize_max_turns) 회전한다.
        고정 바퀴수와 달리 수렴을 보장해, 이후 confirm 이 요구하는 정밀도를 맞춘다.
        (SLAM 매핑은 위치 고정이라 불필요 → 런타임/AMCL 전용.)"""
        # 1) ★ '무적' 초기화: 시작 추정을 맵 중앙에 심는다(/initialpose). 전역 분산(global_localization)은
        #    평균 추정이 맵 밖으로 튀어 planner 가 'start outside map'으로 거부 → 안 움직임.
        #    맵 중앙은 '절대 맵 밖 아님' 보장 → 무조건 진행. 실제 위치는 이어지는 회전으로 수렴.
        ctr = self._map_center() or (0.0, 0.0)
        self._seed_center = ctr
        self._set_initial_pose(ctr[0], ctr[1], 0.0)
        # 2) 제자리 회전으로 수렴(스캔매칭 정밀화). 임계 미달이어도 '무조건 진행'.
        self._spin_to_converge()
        # 2.5) 혹시 회전 중 발산했으면 중앙으로 복구(무적 가드)
        self._ensure_in_map()
        # 3) ★ 맵 중앙으로 '실제 주행' 후 재수렴 — 파티클(점)만 모으지 말고 로봇이 중앙
        #    개활지로 이동해 위치추정을 다진다.
        if self.goto_center:
            ctr = self._map_center()
            if ctr is None:
                self.get_logger().warn('맵 중앙 계산 불가(랜드마크 없음) — 중앙 이동 생략')
            else:
                here = self.get_robot_xy(timeout=2.0)
                yaw = math.atan2(ctr[1] - here[1], ctr[0] - here[0]) if here else 0.0
                self.get_logger().info(f'맵 중앙({ctr[0]:.2f},{ctr[1]:.2f})으로 실제 이동 후 재수렴')
                if self.nav_to(ctr[0], ctr[1], yaw, '맵 중앙'):
                    self._spin_to_converge()

    def _spin_to_converge(self):
        """제자리 회전하며 AMCL 공분산이 임계 이하로 수렴할 때까지 대기(상한 max_turns)."""
        max_dur = self.relocalize_max_turns * 2.0 * math.pi / max(0.05, self.relocalize_speed)
        self.get_logger().info(
            f'자기위치 추정 회전 — 수렴까지(상한 {self.relocalize_max_turns:.1f}바퀴/'
            f'~{max_dur:.0f}s, 임계 pos<{self.relocalize_pos_std}m yaw<{self.relocalize_yaw_std}rad)')
        twist = Twist()
        twist.angular.z = self.relocalize_speed
        end = time.time() + max_dur
        converged = False
        last_log = 0.0
        settle_start = None   # 임계 이하가 '연속 유지'되기 시작한 시각(한 번 튄 샘플 무시)
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
            # 임계 이하가 relocalize_settle_secs 동안 '연속' 유지돼야 수렴 인정.
            # (초기화 직후 가짜 0 이나 한 프레임 튄 저공분산에 속아 조기 종료하던 문제 차단.)
            if self._amcl_converged():
                if settle_start is None:
                    settle_start = now
                elif now - settle_start >= self.relocalize_settle_secs:
                    converged = True
                    break
            else:
                settle_start = None
        self.cmd_pub.publish(Twist())   # 정지
        c = self._amcl_cov
        if converged and c is not None:
            ps = max(math.sqrt(max(c[0], 0.0)), math.sqrt(max(c[7], 0.0)))
            ys = math.sqrt(max(c[35], 0.0))
            self.get_logger().info(f'AMCL 수렴 완료(pos_std={ps:.3f}m, yaw_std={ys:.3f}rad)')
        else:
            self.get_logger().warn(
                '최대 회전까지 공분산 임계 미달 — 그대로 진행(confirm 실패 가능). '
                'relocalize_max_turns 를 늘리거나 맵/스캔 품질 점검 권장.')

    def _map_center(self):
        """color_landmarks.yaml 전체 랜드마크의 무게중심 ≈ 맵 중앙(개활지 추정)."""
        try:
            with open(self.landmarks_path) as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            return None
        xs, ys = [], []
        for walls in data.values():
            for w in (walls or []):
                if 'x' in w and 'y' in w:
                    xs.append(float(w['x'])); ys.append(float(w['y']))
        if not xs:
            return None
        return (sum(xs) / len(xs), sum(ys) / len(ys))

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
            # 한 번의 순회에서 어떤 예외가 나도 서비스 노드 자체는 살아남아 다음 색을
            # 계속 받도록 가드(매핑/맵 품질 문제로 죽어버리면 재시작 부담이 큼).
            try:
                self.run_tour()
            except Exception as e:
                self.get_logger().error(f'순회 중 예외 — 무시하고 다음 색 대기: {e}')
            if self.oneshot:
                self.get_logger().info('oneshot=true → 순회 1회 후 종료')
                return
            self.get_logger().info('=== 다음 색 대기 (/target_color) ===')

    # ── digit 발견 패스 ────────────────────────────────────────────
    def _discover_digits(self, walls):
        """각 벽을 빠르게 방문해 보이는 digit 을 기록. {wall_id: digit} 반환.
        digit 미감지 벽은 결과에서 제외."""
        from collections import Counter
        kor = KOR.get(self.target, self.target)
        digit_map = {}
        for w in walls:
            ax, ay, yaw = approach_pose(w['x'], w['y'], self.standoff)
            if not self.nav_to(ax, ay, yaw, f'탐색 {kor}{w["id"]}'):
                continue
            seen = []
            end = time.time() + 2.5
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._detected_digit >= 0:
                    seen.append(self._detected_digit)
            if seen:
                digit = Counter(seen).most_common(1)[0][0]
                digit_map[w['id']] = digit
                self.get_logger().info(f'  {kor} {w["id"]}번 → 숫자 {digit} 감지')
            else:
                self.get_logger().warn(f'  {kor} {w["id"]}번 숫자 감지 실패 — 건너뜀')
        return digit_map

    # ── 한 색 순회 ─────────────────────────────────────────────────
    def run_tour(self):
        self.get_logger().info(f'=== 색벽 순회 시작: target = {self.target} ===')
        self.pub_done.publish(Bool(data=False))   # 새 미션 시작 → done 리셋
        self.target_digit_known = False           # 이전 순회 상태 이월 방지

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

        if self.target_digit is None:
            # ── 전체 순회 모드 ────────────────────────────────────────
            walls_with_digit = [w for w in walls if w.get('digit') is not None]
            if len(walls_with_digit) == len(walls):
                # 매핑 때 digit 저장됨 → 바로 정렬 (발견 패스 불필요)
                order = sorted(walls, key=lambda w: w['digit'])
                ids = ' → '.join(f'{kor}{w["digit"]}' for w in order)
                self.get_logger().info(f'저장된 digit 순 방문: {ids}')
            else:
                # digit 정보 없음 → 발견 패스
                self.get_logger().info(
                    f'{kor} 전체 순회 — 각 벽의 숫자를 탐색합니다.')
                nn_order = order_walls(walls, rxy)
                digit_map = self._discover_digits(nn_order)
                if not digit_map:
                    self.get_logger().error('숫자 감지 실패 — 순회 중단.')
                    return False
                order = sorted(
                    [w for w in walls if w['id'] in digit_map],
                    key=lambda w: digit_map[w['id']])
                ids = ' → '.join(f'{kor}{digit_map[w["id"]]}' for w in order)
                self.get_logger().info(f'digit 순 방문 순서: {ids}')
        else:
            # ── 특정 숫자 모드: 해당 digit 벽으로 바로 이동 ───────────────
            # 매핑 때 저장된 digit 을 '우선' 사용한다 → 라이브 OCR 의존 제거. 런타임에
            # digit_recognizer 가 잠깐 못 읽어도 매핑에서 확정한 벽으로 직행할 수 있다.
            # 저장 digit 이 전혀 없을 때만 모든 벽을 돌며 라이브 OCR 로 확인하는 폴백.
            matched = [w for w in walls if w.get('digit') == self.target_digit]
            if matched:
                order = order_walls(matched, rxy)
                self.target_digit_known = True   # 도착 확인은 색만(숫자는 맵으로 확정)
                self.get_logger().info(
                    f'{kor} {self.target_digit}번 — 저장 digit 일치 {len(order)}개 후보로 직행')
            else:
                order = order_walls(walls, rxy)
                self.target_digit_known = False
                self.get_logger().info(
                    f'{kor} {self.target_digit}번 — 저장 digit 없음, '
                    f'라이브 OCR 로 {len(order)}개 벽 탐색')

        confirmed = []          # [(id, x, y, ax, ay, yaw), ...] 확인된 벽
        failed = []             # [(id, 사유), ...] 접근/확인 실패한 벽
        for w in order:
            wid = w.get('digit', w['id'])   # 표시·로그는 '숫자' 기준(id 인덱스와 혼동 방지)
            ax, ay, yaw = approach_pose(w['x'], w['y'], self.standoff)
            label = f'{kor} {wid}번 ({w["x"]:.2f},{w["y"]:.2f})'
            if not self.nav_to(ax, ay, yaw, label):
                self.get_logger().error(f'{kor} {wid}번 접근 실패')
                failed.append((wid, '접근'))
                continue
            self.recenter_on_target()   # 옆에서 보면 벽 평행이동으로 패널 정면 진입(+벽 수직 정렬)
            self.visual_approach()      # 정면으로 전진해 점유율↑
            self.get_logger().info(f'{kor} {wid}번 도착 — {self.dwell_secs:.0f}s 정지 관찰(인식 무관 머묾)')
            ok = self.dwell_observe(self.dwell_secs)
            tries = 0
            while not ok and tries < self.confirm_retries:
                tries += 1
                self.get_logger().info(
                    f'{kor} {wid}번 확인 실패 → 조금 뒤로 물러나 카메라 재확보 ({tries}/{self.confirm_retries})')
                self._nudge_back()
                self.recenter_on_target()   # 물러난 뒤 측면정렬로 정면 재진입
                self.visual_approach()      # 다시 정면으로 다가가 점유율 회복
                ok = self.await_confirmation()
            if ok:
                self.get_logger().info(f'{kor} {wid}번 확인({CONFIRM_THRESHOLD:.0%} 이상)')
                confirmed.append((wid, w['x'], w['y'], ax, ay, yaw))
                # 특정 digit 모드: 일치하는 벽 하나 찾으면 바로 완료
                if self.target_digit is not None:
                    break
            else:
                if self.target_digit is not None:
                    # 다른 숫자 벽 → 스킵(실패 아님)
                    self.get_logger().info(f'{kor} {wid}번 숫자 불일치 — 다음 벽으로')
                else:
                    self.get_logger().error(f'{kor} {wid}번 확인 실패')
                    failed.append((wid, '확인'))

        # 엄격 완료: target 색 '모든' 벽이 확인돼야 미션 완료(사양 'after confirming all').
        # 확인/접근 실패 벽이 하나라도 있으면 정상 상태가 아님 → 에스컬레이션, /maze_done 미발행.
        # (그런 벽이 생긴다는 건 보통 매핑/SLAM·랜드마크 품질 문제이므로 매핑을 다시 제대로 할 것.)
        # 특정 digit 모드: confirmed 가 하나도 없으면 미발견
        if self.target_digit is not None:
            if not confirmed:
                self.get_logger().error(
                    f'=== {kor} {self.target_digit}번 벽을 찾지 못했습니다 — /maze_done 미발행 ===')
                return False
        elif failed:
            detail = ', '.join(f'{kor} {fid}번({why})' for fid, why in failed)
            self.get_logger().error(
                f'=== 미션 실패: {len(confirmed)}/{len(order)}개만 확인, '
                f'미확인 [{detail}] — /maze_done 미발행. ===')
            return False

        # 전부 확인됨 → 마지막(=방문 순서상 마지막) 확인 벽에서 정지.
        last_id, last_x, last_y, ax, ay, yaw = confirmed[-1]
        here = self.get_robot_xy(timeout=2.0) or (ax, ay)
        if math.hypot(here[0] - ax, here[1] - ay) > 0.3:
            self.get_logger().info(f'마지막 확인 벽({kor} {last_id}번)으로 복귀 후 정지')
            self.nav_to(ax, ay, yaw, f'{kor} {last_id}번')

        # ★ 모든 타겟 도착 후: 마지막 벽에 정면(수직)으로 자세 정렬하고 정지.
        if self.align_on_finish:
            self.get_logger().info(f'마지막 벽({kor} {last_id}번)에 정면 정렬(벽 정렬)')
            self.recenter_on_target()   # 옆으로 치우쳤으면 벽 평행이동으로 정면 진입
            self.align_to_wall()        # 벽에 수직으로 자세 맞춤

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
