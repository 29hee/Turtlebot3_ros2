#!/usr/bin/env python3
"""
mode_guard.py — 매핑/런타임 '동시 구동' 차단.

[왜] 매핑과 런타임은 공유 자원이 겹친다:
  · TF map→odom : 매핑=slam_toolbox, 런타임=AMCL  → 둘이 동시에 쏘면 위치가 튄다.
  · /cmd_vel    : 매핑=maze_explorer, 런타임=Nav2 → 둘이 동시에 쏘면 로봇이 발작한다.
한쪽을 안 끄고 다른 쪽을 켜면(실제로 고아 explorer 로 겪음) 조용히 서로를 망친다.
→ 시작 직전에 '반대 모드' 시그니처 노드가 살아있는지 보고, 있으면 ERROR + exit 1.
   런치는 이 종료코드를 보고 Shutdown 한다(아래 launch 의 OnProcessExit).

사용(런치가 자동 호출):  python3 mode_guard.py --expect mapping|runtime
"""
import sys
import time

import rclpy
from rclpy.node import Node

# 각 모드를 켤 때 '있으면 안 되는' 반대 모드 노드들(베이스 이름).
#   ※ Nav2(controller/planner/bt)는 이제 매핑(2-pass Phase2)·런타임 둘 다 쓰므로 구분에서 제외.
#     매핑↔런타임을 가르는 결정적 노드: 매핑=slam_toolbox, 런타임=amcl/maze_tour.
CONFLICT = {
    'mapping': ['amcl', 'maze_tour', 'color_confirm'],
    'runtime': ['slam_toolbox', 'maze_explorer', 'color_mapper', 'digit_finalizer'],
    # finalize(2-pass Phase2): 동결맵+AMCL+Nav2 위에서 digit_finalizer 가 돈다.
    #   매핑 스택(slam_toolbox/maze_explorer/color_mapper)과 동시구동 금지(map→odom 충돌).
    #   digit_finalizer 자신은 finalize 가 띄우므로 제외.
    'finalize': ['slam_toolbox', 'maze_explorer', 'color_mapper'],
}


def main():
    expect = 'mapping'
    for i, a in enumerate(sys.argv):
        if a == '--expect' and i + 1 < len(sys.argv):
            expect = sys.argv[i + 1]

    rclpy.init()
    node = Node('mode_guard')
    # 그래프 디스커버리에 잠깐 시간을 준다(이웃 노드들이 광고될 때까지).
    end = time.time() + 2.5
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    names = set(node.get_node_names())
    bad = [c for c in CONFLICT.get(expect, []) if c in names]

    rc = 0
    if bad:
        node.get_logger().error(
            f"[mode_guard] '{expect}' 시작 차단 — 반대 모드 노드 실행 중: {bad}. "
            f"먼저 그 스택을 완전히 종료할 것(매핑↔런타임 동시구동 금지: "
            f"map→odom·/cmd_vel 발행자 충돌).")
        rc = 1
    else:
        node.get_logger().info(f"[mode_guard] '{expect}' 충돌 없음 — 진행.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
