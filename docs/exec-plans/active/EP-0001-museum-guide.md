# Execution Plan: EP-0001 Museum Guide Pipeline

## Goal

Build the end-to-end pipeline: scan a map, recognize numbered color panels, and
navigate on request to a chosen number, stopping front-facing the panel. Verify on a
physical TurtleBot3 Burger.

## Context

- Hardware: TurtleBot3 Burger (Raspberry Pi SBC, slow on-board compute, forward-fixed
  camera). ROS2 Humble.
- The forward-fixed camera means front-facing a wall panel is an orientation problem,
  not just a position problem. See [DESIGN.md](../../DESIGN.md).
- Registering panel coordinates during scanning is the primary risk.
- Heavy vision should be offloaded to a remote PC; see [RELIABILITY.md](../../RELIABILITY.md).
- Spec: [museum-guide.md](../../product-specs/museum-guide.md).

## Tasks

- [ ] Bring up SLAM and save an occupancy-grid map of the test environment.
- [ ] Implement color-panel detection (HSV mask + contours) on the camera stream.
- [ ] Implement number recognition on the detected ROI, with a confidence output.
- [ ] Project detections to map-frame poses via tf2 and snap to the nearest wall.
- [ ] Build the number→pose registry (position + outward wall normal + confidence).
- [ ] Compute Nav2 goals that offset along the wall normal and face the panel.
- [ ] Wire goal selection: user picks a number → resolve pose → NavigateToPose.
- [ ] Split compute between Burger and remote PC; confirm stage timings are acceptable.
- [ ] Validate the full pipeline in Gazebo (capstone_color_maze world).
- [ ] Verify the full pipeline on the physical Burger.

## Done When

- A map is scanned and saved.
- Panels and numbers are recognized with a reported accuracy.
- Each number resolves to one map-frame pose.
- The robot navigates to a selected number and stops front-facing within tolerance.
- The pipeline is verified on the physical TurtleBot3 Burger.
