#!/usr/bin/env python3
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()

    # Set initial pose so AMCL can localize without RViz / 초기 위치 설정 (RViz 없이 자동 로컬라이제이션)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate / nav2 활성화 대기
    navigator.waitUntilNav2Active()

    # Define a goal pose / 목표지점 설정
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.5
    goal_pose.pose.position.y = 0.5
    goal_pose.pose.orientation.w = 1.0

    # Send the goal pose to the navigator  / 주행명령
    navigator.goToPose(goal_pose)

    # Wait for the robot to reach the goal / 로봇 주행 상태 확인
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'남은 거리 : ({feedback.distance_to_goal:.2f})')
        time.sleep(0.1)

    print('목적지 도착!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()  