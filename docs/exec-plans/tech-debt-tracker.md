# Tech Debt Tracker

Track known shortcuts, fragile areas, and deferred cleanups. Move items down the
sections as they are triaged and resolved.

## Open Debt

- Learning/tutorial packages (`hello_ros2_pkg`, `hello_cmake_pkg`, `tf_tutorial_pkg`,
  `py_launch_example`) live alongside production packages and may confuse navigation.
- Number→pose registry format is not yet specified as a shared interface.

## Prioritized Debt

| Priority | Item | Rationale |
| --- | --- | --- |
| High | Define the number→pose registry contract | It is the core perception↔nav boundary |
| Medium | Separate tutorial packages from production packages | Reduces ambiguity for agents |

## Resolved Debt

| Date | Item | Resolution |
| --- | --- | --- |
| | | |
