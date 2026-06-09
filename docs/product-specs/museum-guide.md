# Feature Spec: Museum Guide

## Problem

A museum visitor — especially a visitor with limited mobility — wants to view a
specific exhibit but cannot easily move to it or locate it. The robot should act as a
guide: the visitor selects a number, and the robot leads to that exhibit and positions
itself so the numbered panel is clearly visible from the front.

## Scope

In scope:

- Scan the environment with SLAM and save an occupancy-grid map.
- Detect colored panels mounted on walls and recognize the numbers on them.
- Register each recognized number to a pose in the map frame.
- On a user-selected number, navigate to that pose with Nav2.
- On arrival, orient the robot so its forward-facing camera sees the panel front-on.

Out of scope (for now):

- Multi-robot coordination.
- Voice interaction or a rich visitor UI.
- Dynamic re-mapping while guiding.

## Constraints

- ROS2 Humble; TurtleBot3 Burger only.
- The camera is fixed to the robot heading (forward-facing). Facing a wall-mounted
  panel front-on therefore requires a deliberate orientation maneuver — the robot
  cannot look sideways. See [DESIGN.md](../DESIGN.md) for the alignment approach.
- Registering panel coordinates during map scanning is hard, because the robot may
  detect a panel only obliquely and must convert a camera detection into a stable
  map-frame pose. This is a primary design risk; see [DESIGN.md](../DESIGN.md).
- Safe, predictable motion around people takes priority over speed.

## Done When

- A map can be scanned via SLAM and saved.
- Colored panels and their numbers are reliably detected and recognized, with a
  reported accuracy.
- Each number resolves to a single map-frame pose.
- The robot autonomously navigates to a selected number and stops front-facing the
  panel within a defined orientation tolerance.
- The full pipeline is verified on a physical TurtleBot3 Burger.
