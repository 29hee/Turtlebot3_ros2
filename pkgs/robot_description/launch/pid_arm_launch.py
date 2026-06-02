import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'one_dof_arm.xacro')


    # 페이로드(하중) 질량을 런치 인자로 선언
    declare_payload = DeclareLaunchArgument(
        'payload_mass',
        default_value='1.0',
        description='Payload mass in kg (default: 1.0kg)'
    )

    # xacro 파일을 처리하여 robot_description 파라미터로 전달
    robot_description = Command([
        'xacro ', xacro_file,
        ' payload_mass:=', LaunchConfiguration('payload_mass')
        ])
    

    # Gazebo 실행 명령어 (gazebo classic 백엔드 구동프로세스)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            ],
        output='screen'
    )

    # 로봇 상태 발행 노드 (robot_state_publisher) 설정
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # gazebo 월드에 urdf spawn 명령어 (spawn_entity.py) 설정
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'turtlebot',
                   '-z', '1.0'],
        output='screen'
    )

    # 로봇 스폰 후 조인트 브로드캐스터 컨트로러 가동
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    #브로드캐스터 구동 완료 후 토크 제어기 가동
    load_effort = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['effort_controller', '-c', '/controller_manager'],
        output='screen'
    )

    rviz_file = os.path.join(pkg_share, 'rviz', 'arm_practice.rviz')

    return LaunchDescription([
        declare_payload,
        gazebo,
        rsp,
        spawn,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_effort],
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}, {'config_file': rviz_file}]
        ),
    ])