# Architecture

This document explains the stable structure of the repository. It changes only when
the high-level module boundaries change, not on every feature. Read it first to learn
where things live and which invariants must hold.

## System Map

The system is a ROS2 Humble graph running on (or simulating) a TurtleBot3 Burger.
The end-to-end pipeline has four stages:

1. Mapping — SLAM scans the environment and produces a saved occupancy-grid map.
2. Perception — a camera/vision node detects colored panels and reads their numbers,
   registering each number against a map coordinate.
3. Goal selection — a user picks a target number; the system resolves it to a pose.
4. Navigation — Nav2 drives the robot to the pose and aligns it front-facing the panel.

```text
sensors (LiDAR, camera)
        │
        ▼
  SLAM / mapping ──────► saved map (occupancy grid)
        │                      │
        ▼                      ▼
  vision (OpenCV) ──► number→pose registry
        │                      │
        ▼                      ▼
  goal selection ───────► Nav2 navigation ──► front-facing alignment
```

## Module Boundaries

The work surface is `pkgs/`. Packages are split by responsibility:

- Description / model: `robot_description`, `my_robot_description` — URDF, RViz, worlds.
- Sensing: `hee_lidar`, `camera_pkg` — sensor drivers and processing.
- Interfaces: `my_robot_interfaces` — custom `msg`/`srv`/`action` definitions.
- Behavior building blocks: `my_robot_action`, `my_robot_service` — action/service nodes.
- Capstone: `capstone_color_maze` — worlds, maps, and navigation scripts for the goal.
- Learning samples: `hello_ros2_pkg`, `hello_cmake_pkg`, `tf_tutorial_pkg`,
  `py_launch_example` — reference/tutorial packages, not production code.

External dependencies (TurtleBot3, Nav2 explore, DynamixelSDK) are pulled via
`deps.repos`, not vendored.

## Invariants

- ROS2 distribution is Humble; nodes must build under Humble's ament toolchains.
- Hardware target is TurtleBot3 Burger; world and URDF assumptions follow Burger.
- Custom message/service/action types live only in `my_robot_interfaces`.
- Generated artifacts (`build/`, `install/`, `log/`, `*.pt`, `*.db3`, `*.bag`) are
  never committed; they are reproducible from source.
- The number→pose registry is the single contract between perception and navigation.
- `ROS_DOMAIN_ID` is environment configuration, never hard-coded into nodes.
