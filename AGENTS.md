# AGENTS

This repository is a ROS2 Humble / TurtleBot3 Burger autonomous-driving project.
The product goal is a museum accessibility-assistant robot: scan a map, recognize
colored panels with numbers, and on request navigate to a chosen number and stop
front-facing the panel. This file is a navigation map, not a manual.

## Read Order

1. `ARCHITECTURE.md` — stable repository structure and invariants.
2. `docs/product-specs/index.md` — what is being built and its done-when criteria.
3. `docs/exec-plans/active/` — in-progress execution plans.
4. `docs/design-docs/` — core beliefs and design decisions.
5. `docs/references/` — `*-llms.txt` quick references for ROS2, Nav2/SLAM, Gazebo, OpenCV.
6. `commands.md` — ROS2/TurtleBot3 command cheatsheet (reference only).

## Repository Map

- `pkgs/` — all ROS2 packages (ament_python and ament_cmake). Primary work surface.
- `pkgs/capstone_color_maze/` — color-maze worlds, maps, and navigation scripts.
- `pkgs/rosout_otel_bridge/` — exports `/rosout` logs into the observability stack.
- `observability/` — local OTel stack (collector + VictoriaLogs/Metrics/Traces) that
  collects and manages logs; see `observability/docs/ROS2.md` for ROS2 log ingestion.
- `data/` — datasets and recordings (rosbag `.db3`, model weights are git-ignored).
- `deps.repos` — external dependencies imported via `vcs import`.
- `docs/` — harness documentation set.
- `scripts/` — repository tooling (`init.sh`).
- `commands.md` — command cheatsheet (existing, reference only).

## Core Constraints

- ROS2 distribution is fixed to Humble; do not introduce other-distro dependencies.
- Target hardware is TurtleBot3 Burger only (`TURTLEBOT3_MODEL=burger`).
- Manage `ROS_DOMAIN_ID` to avoid cross-robot collisions on shared networks.
- Never commit build artifacts: `build/`, `install/`, `log/`, weights, rosbags.

## Done When

- A map can be scanned via SLAM and saved.
- Colored panels and their numbers are reliably detected and recognized.
- The robot autonomously navigates to a selected number via Nav2 and stops
  oriented front-facing the panel.
- The full pipeline is verified on a physical TurtleBot3 Burger.

## Working Agreements

- Treat existing `commands.md` and package `README.md` files as reference, not as
  authoritative specs. Authoritative intent lives under `docs/`.
- Independent tasks SHOULD be executed in parallel.
- TODO items MUST be tracked explicitly.
