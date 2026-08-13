---
description: Autonomous frontier exploration — drives the robot to map and catalog its surroundings (occupancy grid + semantic object registry).
---

# explore_rbnx — autonomous environment exploration skill

Drives the robot through frontier-based exploration of its surroundings,
building a spatial occupancy map (via the mapping service) and a
semantic object registry (via scene's built-in VLM detector) along the
way. User-invocable via the LLM/pilot.

## Interface (3 MCP tools)

### `robonix/skill/explore/explore`

Start an exploration task. Returns immediately with a `run_id`; poll
`status` to track progress.

| param            | type   | default | meaning                                    |
|------------------|--------|---------|--------------------------------------------|
| `area_hint`      | string | `""`    | free-form ("this room", "ground floor")    |
| `timeout_s`      | float  | 600     | hard ceiling, 0 = no timeout               |
| `max_speed_m_s`  | float  | 0.25    | safety cap on forward speed during explore |

Returns `{accepted, run_id, message}`. `accepted=false` if a task is
already running.

### `robonix/skill/explore/explore/status`

Poll an exploration's progress. Empty `run_id` returns the most
recent task.

Returns `{known, state, area_m2, frontiers_left, elapsed_s, eta_s, detail}`.
`state ∈ {PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED | TIMEOUT}`
(terminal: SUCCEEDED / FAILED / CANCELED / TIMEOUT).

### `robonix/skill/explore/explore/cancel`

Abort the active exploration. Idempotent.

## Usage pattern (IMPORTANT — thread the run_id)

1. Call `explore` ONCE. It returns immediately with a `run_id`. **Save that
   exact `run_id`.**
2. To monitor, call `status` with that SAME `run_id` — repeatedly, until
   `state` is a terminal value (`SUCCEEDED | FAILED | CANCELED | TIMEOUT`). Do
   not call `explore` again to monitor; that starts nothing new (a task is
   already running) and loses your handle.
3. To stop it, call `cancel` with that SAME `run_id`.

Passing an empty `run_id` uses the most recent exploration task.

## Behaviour

1. Wait for `/map` (mapping service publishes it).
2. Loop:
   - Find frontier cells (free cells adjacent to unknown).
   - Cluster them with size ≥ 8 cells; reject any centroid
     - within 30 cm of an obstacle (skill-side safety filter), or
     - more than 6 m from the robot (local exploration only), or
     - inside cells we've already visited
   - Pick highest-scoring frontier (info_gain ÷ travel_cost).
   - Send goal via `service/navigation/navigate` MCP; poll status.
   - On arrival, conditionally do a 360° in-place sweep to fill
     camera viewing-angle coverage at the leg endpoint.
3. Declare `done` when 0 frontier clusters remain and area hasn't
   grown for 30 seconds.

## What this skill does NOT do

- No SLAM (mapping service does that — atlas-resolved input).
- No collision avoidance (navigation service does that — atlas-resolved).
- No object detection (scene service does that, in-process VLM, on
  the camera frames published by the camera primitive).

The skill is purely a goal-selection loop with safety pre-filters. All
sensor/actuator access is through atlas-resolved contracts; no
hardcoded inter-package topic names.

## Dependencies it Connect()s on atlas at startup

| key             | contract                                | transport |
|-----------------|------------------------------------------|-----------|
| map_topic       | robonix/service/map/occupancy_grid       | ROS2      |
| nav_navigate    | robonix/service/navigation/navigate      | MCP       |
| nav_status      | robonix/service/navigation/navigate/status | MCP     |
| nav_cancel      | robonix/service/navigation/navigate/cancel | MCP     |

Skill refuses to start if any of these is missing — there is no
hardcoded fallback (packaging-spec invariant #1).

## Portability

The skill code is identical between webots and the real robot. What
differs is which packages back the consumed contracts:

- webots: mapping=rtabmap (2D lidar + RGBD), nav=simple_nav, chassis=tiago_chassis
- jetson AGX + Mid360: mapping=DLIO (3D livox + IMU), nav=nav2-wrapper, chassis=ranger_chassis

Both back the same `service/map/occupancy_grid` + `service/navigation/*`
contracts; explore neither knows nor cares which is running.
