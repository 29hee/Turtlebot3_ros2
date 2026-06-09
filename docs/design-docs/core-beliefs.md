# Core Beliefs

The principles that guide product, engineering, and verification decisions on this
project. When a trade-off is unclear, resolve it in favor of these beliefs.

## Product Principles

- The robot serves accessibility: it helps a museum visitor reach and view a chosen
  exhibit. Arriving near the panel is not enough — the robot must face it front-on so
  the visitor can see the number clearly.
- A selected number must map to one unambiguous physical destination.
- Predictable, safe motion beats fast motion in a space shared with people.

## Engineering Principles

- One responsibility per package; cross-package contracts go through
  `my_robot_interfaces`.
- The number→pose registry is the single source of truth between perception and
  navigation; keep it explicit and inspectable.
- Prefer ROS2-native tools (Nav2, SLAM Toolbox, tf2) over bespoke reimplementations.
- Configuration (model, domain id, topic names) is data, not code.

## Verification Principles

- Simulation first, hardware second: validate in Gazebo before running on the Burger.
- A feature is done only when the full pipeline is verified on the physical robot.
- Perception claims require measured detection/recognition accuracy, not anecdotes.
- Navigation success means reaching the pose AND achieving front-facing alignment
  within a stated tolerance.
