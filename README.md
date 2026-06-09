# Turtlebot3_ros2

TurtleBot3 Burger · ROS2 Humble 기반 자율주행 프로젝트입니다.
SLAM으로 환경 맵을 스캔하고, 컬러판에 적힌 숫자 지점을 인식한 뒤,
사용자가 선택한 번호로 자율주행해 패널 정면에 정렬합니다.
미술관에서 장애인 관람을 돕는 안내 로봇 시나리오를 목표로 합니다.

## Installation

```bash
# ROS2 Humble 환경 소스
source /opt/ros/humble/setup.bash

# 워크스페이스로 이동 후 외부 의존성 가져오기
vcs import src < deps.repos

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# 환경 변수 (매 터미널 또는 ~/.bashrc)
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

# 시뮬레이션 주행 예시
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 캡스톤 컬러 미로 패키지
ros2 launch capstone_color_maze <launch_file>
```

자세한 명령어는 [commands.md](commands.md) 치트시트를 참고하세요.

## Usage

1. SLAM으로 맵을 스캔하고 저장합니다.
2. 저장된 맵에서 컬러판·숫자 지점을 인식해 좌표를 등록합니다.
3. 사용자가 번호를 선택하면 Nav2로 해당 지점까지 자율주행합니다.
4. 도착 후 로봇이 패널 정면을 바라보도록 정렬합니다.

## Generated Structure

```text
.
├── AGENTS.md              # 에이전트 탐색 규칙 및 읽기 순서
├── ARCHITECTURE.md        # 저장소 구조와 불변식
├── CLAUDE.md              # Claude Code 작업 가이드
├── commands.md            # ROS2/TurtleBot3 명령어 치트시트 (참고용)
├── deps.repos             # 외부 의존성 저장소 목록
├── docs/
│   ├── DESIGN.md
│   ├── PRODUCT_SENSE.md
│   ├── design-docs/
│   ├── exec-plans/
│   ├── generated/
│   ├── product-specs/
│   └── references/        # *-llms.txt 에이전트 참조 파일
├── pkgs/                  # ROS2 패키지
└── scripts/init.sh        # 디렉터리 초기화 스크립트
```
