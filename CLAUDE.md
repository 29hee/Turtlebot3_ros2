# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this project is

A ROS2 Humble / TurtleBot3 Burger autonomous-driving project. Goal: a museum
accessibility-assistant robot that scans a map, recognizes numbered color panels, and
on request navigates to a chosen number and stops front-facing the panel. Start from
[AGENTS.md](AGENTS.md) for the read order and repository map.

## Hard constraints

- ROS2 distribution is Humble. Do not add other-distro dependencies.
- Hardware is TurtleBot3 Burger only; `TURTLEBOT3_MODEL=burger`.
- The Burger runs on a Raspberry Pi — on-board compute is slow. Keep the on-board
  hot path light; offload heavy vision to a remote PC. See [docs/RELIABILITY.md](docs/RELIABILITY.md).
- The camera is fixed to the robot heading. Front-facing a wall panel is an
  orientation problem; encode it in the Nav2 goal yaw. See [docs/DESIGN.md](docs/DESIGN.md).
- Set a deliberate `ROS_DOMAIN_ID` to avoid cross-robot collisions on shared networks.
- Never commit `build/`, `install/`, `log/`, `*.pt`, `*.db3`, `*.bag`.

## Where to work

- Packages live in `pkgs/`. Custom msg/srv/action types go only in `my_robot_interfaces`.
- The capstone surface is `pkgs/capstone_color_maze/`.
- Treat `commands.md` and package `README.md` files as reference; authoritative intent
  lives under `docs/`.

## Workflow expectations

- Validate in Gazebo before running on hardware.
- Independent tasks SHOULD run in parallel; track TODO items explicitly.
- Commit messages follow Conventional Commits with a bullet-list body, no footers.
