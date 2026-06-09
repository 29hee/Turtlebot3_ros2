# Reliability

The TurtleBot3 Burger runs on a Raspberry Pi SBC. On-board compute is limited and
slow, which shapes where work runs and how the pipeline degrades. Treat compute as a
scarce resource.

## Platform Constraints

| Platform | Constraint | Impact |
| --- | --- | --- |
| Raspberry Pi (Burger) | Low CPU, no usable GPU, limited RAM | Heavy vision/OCR is too slow on-board |
| Raspberry Pi (Burger) | Camera bandwidth/throughput limits | High-res streaming over the network is costly |
| Wi-Fi (shared) | Variable latency/bandwidth | Remote offload adds latency; ROS_DOMAIN_ID must be isolated |

## Caching Strategy

- Persist the scanned map and the number→pose registry to disk; do not recompute
  perception at guide time.
- At navigation time, read the registry only — perception does not run in the hot path.

## Error Handling and Observability

- Each detection carries a confidence; reject low-confidence numbers rather than
  registering a wrong pose.
- Log per-stage timing (perception, tf2 transform, Nav2 planning) to find the SBC
  bottleneck.
- If a Nav2 goal is rejected or unreachable, surface it instead of silently retrying.

## Graceful Degradation

| Failure | Degraded Behavior |
| --- | --- |
| On-board vision too slow for real-time | Move perception to a remote PC; stream compressed images, return results over ROS2 |
| Recognition confidence low | Skip registration; flag the panel for a slow second pass |
| Nav2 cannot reach the panel pose | Stop at the nearest safe reachable pose and report |
| Network/offload unavailable | Fall back to a lightweight on-board detector (color + simple template) |

## Recovery Runbook

1. Confirm `TURTLEBOT3_MODEL=burger` and a unique `ROS_DOMAIN_ID` on every machine.
2. Verify the bringup graph: `ros2 node list`, `ros2 topic hz <camera_topic>`.
3. If perception lags, switch to remote-PC processing (set `use_sim_time` correctly,
   confirm the camera topic crosses the network) and re-check stage timings.
4. If navigation stalls, check localization (AMCL) and that the goal pose lies in free
   costmap space.

## Compute Placement

- Keep on the Burger: sensor drivers, teleop, lightweight color masking, motion.
- Offload to a remote PC where possible: heavy detection, number recognition/OCR,
  map building if it strains the SBC.
- The contract between the two halves stays small: images out, number→pose in.
