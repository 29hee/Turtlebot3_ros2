# Design Decisions

## Decision Log

| Date | Decision | Rationale | Alternatives Rejected |
| --- | --- | --- | --- |
| 2026-06-09 | Treat the number→pose registry as the single perception↔navigation contract | Keeps the boundary explicit and inspectable | Implicit coupling via shared topics |
| 2026-06-09 | Encode front-facing requirement in the Nav2 goal yaw, not a separate step | A goal is a full pose; the alignment falls out of the navigation goal | Reaching position then ad-hoc rotation |
| 2026-06-09 | Validate in Gazebo before hardware | Cheaper, repeatable, safer iteration | Hardware-first testing |
| 2026-06-09 | Offload heavy vision off the Raspberry Pi Burger to a remote PC | The SBC is too slow for real-time CNN/OCR | Running full recognition on-board |
| 2026-06-09 | Run perception offline at scan time, navigation reads only the registry | Keeps the SBC hot path light | Recomputing perception during guiding |

## Key Challenge 1: Front-facing a wall-mounted panel with a forward-fixed camera

The Burger's camera points in the robot's heading direction; it cannot look sideways.
A panel on a wall therefore cannot simply be approached "to its coordinate" — the
robot must end up positioned in front of the panel with its heading pointing toward
the wall normal.

Approach:

- Represent each panel as a pose: the panel's wall position plus an outward-facing
  wall normal.
- Derive the goal pose by offsetting a standstill distance along the wall normal,
  away from the wall, with the robot yaw set to the inward normal (pointing at the
  panel). This makes "arrive" and "face front" a single Nav2 goal.
- Estimate the wall normal from the map geometry near the panel (the occupied wall
  cells) and/or from the panel's apparent skew in the camera (perspective gives the
  relative angle).

## Key Challenge 2: Acquiring a stable map-frame coordinate during scanning

During SLAM scanning the robot often sees a panel only obliquely or briefly, and a
camera detection is in the camera frame, not the map frame. Converting that into a
reliable, single map-frame coordinate is the primary risk.

Candidate methods (to evaluate):

- tf2 projection: detect the panel in the camera image, estimate range/bearing
  (panel size or depth), and transform through `camera → base_link → map` via tf2 at
  the detection timestamp.
- Wall association: snap the detection to the nearest occupied wall segment in the
  map so the registered pose lies on a real wall, then compute the outward normal.
- Multi-view fusion: aggregate several detections of the same number across the scan
  and fuse them (e.g., average/cluster) to reduce single-frame error.
- Deliberate registration pass: after the rough map exists, drive a slow second pass
  that stops in front of each detected panel to capture a clean, high-confidence pose.

The chosen method must output, per number: a map-frame panel position, an outward
wall normal, and a confidence. Navigation consumes only that registry.

## Open Questions

- How is panel range estimated — monocular size prior, or add a depth source?
- What orientation tolerance counts as "front-facing" (degrees)?
- One-pass (detect during initial SLAM) vs two-pass (map, then registration drive)?
- How are duplicate or conflicting detections of the same number resolved?
- What is the split between Burger-local and remote-PC compute, and the network
  contract between them (compressed image topic out, number→pose in)?
