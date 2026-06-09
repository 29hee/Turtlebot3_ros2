#!/bin/bash
# 터틀봇 자가진단 — 한 번 실행하면 '왜 안 움직이는지' 원인을 스스로 찍어준다.
# 사용(ROS 소스된 터미널에서):  bash scripts/check_robot.sh
echo "===== 터틀봇 자가진단 (약 25초) ====="

ros2 node list 2>/dev/null | grep -q slam_toolbox && S=O || S=X

if timeout 6 ros2 topic echo /scan --once >/dev/null 2>&1; then SCAN=O; else SCAN=X; fi

SUB=$(ros2 topic info /cmd_vel 2>/dev/null | grep -i "Subscription count" | grep -o '[0-9]*')
SUB=${SUB:-0}

timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q Translation && T1=O || T1=X
timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -q Translation && T2=O || T2=X

echo "  slam_toolbox : $S"
echo "  /scan 수신    : $SCAN"
echo "  cmd_vel 구독자: $SUB 개   (로봇 bringup 이 들으면 1 이상)"
echo "  TF map->odom  : $T1   (slam 이 만듦)"
echo "  TF odom->base : $T2   (로봇이 만듦)"
echo "-------------------------------------"
if [ "$SCAN" = "X" ]; then
  echo "[원인] /scan 이 안 들어옴 → scan_explorer 가 첫 스캔을 못 받아 출발도 못 함."
  echo "       라이다 안 돔 / ROS_DOMAIN_ID 불일치 / 로봇-PC 연결 확인."
elif [ "$SUB" = "0" ]; then
  echo "[원인] /cmd_vel 듣는 노드 0개 → 로봇이 명령을 못 받음."
  echo "       robot.launch.py 가 '진짜 터틀봇(라즈베리파이)' 에서 떠 있는지 확인(PC에서만 돌면 모터 없음)."
elif [ "$T2" = "X" ]; then
  echo "[원인] odom->base_link TF 없음 → 로봇 bringup/odom 문제. bringup 재확인."
elif [ "$T1" = "X" ]; then
  echo "[원인] map->odom TF 없음 → slam 이 스캔을 못 받아 맵을 못 만드는 중(rviz base_scan 드롭 원인)."
else
  echo "[판정] 소프트웨어/연결 정상. 그래도 안 움직이면 => 로봇 전원/OpenCR/배터리/모터,"
  echo "       또는 바퀴가 바닥에서 들려있는지 확인. (cmd_vel 직접: ros2 topic pub /cmd_vel ...)"
fi
echo "====================================="
