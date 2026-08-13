---
description: Multi-waypoint patrol with photo capture — navigate to a sequence of waypoints via nav2, capture and save photos at each stop.
---

# patrol_rbnx — multi-waypoint patrol skill with photo capture

Drives the robot through a user-specified sequence of waypoints using
nav2 navigation, captures a photo at each stop via the camera primitive,
and saves images to disk. User-invocable via the LLM/pilot.

## Interface (3 MCP tools)

### `robonix/skill/patrol/patrol`

Start a patrol task. Returns immediately with a `run_id`; poll
`status` to track progress.

| param             | type     | default | meaning                                          |
|-------------------|----------|---------|--------------------------------------------------|
| `waypoint_x`      | float[]  | —       | x coordinates in map frame (metres), required    |
| `waypoint_y`      | float[]  | —       | y coordinates in map frame (metres), required    |
| `waypoint_yaw`    | float[]  | `[]`    | yaw at each waypoint (radians), 0 = no constraint |
| `waypoint_names`  | string[] | `[]`    | human-readable names for each waypoint           |
| `save_dir`        | string   | —       | directory to save photos (relative or absolute)  |
| `settle_time_s`   | float    | 2.0     | wait after arriving before capturing photo       |

Returns `{accepted, run_id, message}`. `accepted=false` if a task is
already running or inputs are invalid.

### `robonix/skill/patrol/patrol/status`

Poll a patrol task's progress. Empty `run_id` returns the most
recent task.

Returns `{known, state, total_waypoints, completed_waypoints, current_waypoint, current_photo_path, detail}`.
`state ∈ {PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED}`
(terminal: SUCCEEDED / FAILED / CANCELED).

### `robonix/skill/patrol/patrol/cancel`

Abort the active patrol. Idempotent.

## Usage pattern (IMPORTANT — thread the run_id)

1. Call `patrol` ONCE with waypoint arrays. It returns immediately with
   a `run_id`. **Save that exact `run_id`.**
2. To monitor, call `status` with that SAME `run_id` — repeatedly, until
   `state` is a terminal value (`SUCCEEDED | FAILED | CANCELED`). Do
   not call `patrol` again to monitor; that starts a new task.
3. To stop it, call `cancel` with that SAME `run_id`.

Passing an empty `run_id` uses the most recent patrol task.

## Behaviour

1. Validate waypoint arrays (x/y must have same length).
2. Sort waypoints by nearest-neighbor from the robot's current position
   (via tf2 `map→base_link` lookup). The closest unvisited waypoint is
   always visited next, minimising total travel distance.
3. For each waypoint in sorted order:
   - Navigate to (x, y, yaw) via `service/navigation/navigate` MCP.
   - Poll nav status until terminal or 120s timeout.
   - Wait `settle_time_s` for the robot to stabilize.
   - Capture a photo via `primitive/camera/snapshot` MCP.
   - Save the photo as `{index}_{name}.jpg` in `save_dir`.
4. If a waypoint navigation fails, skip to the next waypoint.
5. Declare `SUCCEEDED` when all waypoints are visited.

## Call examples

```json
{
  "waypoint_x": [1.0, 2.0, 3.0],
  "waypoint_y": [0.0, 1.0, 0.0],
  "waypoint_yaw": [0.0, 1.57, 3.14],
  "waypoint_names": ["entrance", "shelf", "exit"],
  "save_dir": "patrol_20260813",
  "settle_time_s": 3.0
}
```

```json
{
  "waypoint_x": [0.5, 1.5],
  "waypoint_y": [0.0, 0.5],
  "save_dir": "quick_check"
}
```

## What this skill does NOT do

- No SLAM or mapping (relies on existing map and nav2 service).
- No collision avoidance (navigation service handles that).
- No image analysis (photos are saved raw; use VLM/scene for analysis).

All sensor/actuator access is through atlas-resolved contracts; no
hardcoded inter-package topic names.

## Dependencies it Resolves on atlas at startup

| key            | contract                                 | transport |
|----------------|------------------------------------------|-----------|
| nav_navigate   | robonix/service/navigation/navigate      | MCP       |
| nav_status     | robonix/service/navigation/navigate/status | MCP     |
| nav_cancel     | robonix/service/navigation/navigate/cancel | MCP     |
| camera_snap    | robonix/primitive/camera/snapshot         | MCP       |

Skill refuses to start if any of these is missing — there is no
hardcoded fallback.

## Output structure

Photos are saved as:
```
<save_dir>/
├── 000_entrance.jpg
├── 001_shelf.jpg
└── 002_exit.jpg
```

File naming: `{3-digit index}_{waypoint_name}.jpg`
