# Acceptance matrix

| Layer | Pass condition | Evidence |
|---|---|---|
| Chinese ASR | Exact spoken phrase transcribes as “走到前面自动售货机那里” or a harmless punctuation variant | Liaison/ASR event log |
| Semantic target | Resolver selects exactly `vending_machine_front`; saved pose matches the live `(map_id, generation, localization mode)` lifecycle tuple | Skill response + local landmark checksum + latched lifecycle sample |
| Robonix navigation | `navigate` accepts a run, status progresses PENDING/RUNNING to SUCCEEDED, cancel works | Atlas channel + navigation status log |
| Planner/controller | Global plan stays in known free space; local costmap reacts to a placed obstacle | Nav2 diagnostics/plan/costmap |
| Chassis input | Only the navigation velocity guard publishes canonical `/cmd_vel` | ROS graph audit |
| State/TF | Fresh `/odom`, `map -> odom -> base_link`, no duplicate authority, bounded drift; the selected SportModeState stream has a non-zero source stamp that strictly advances before motion is enabled | topic/TF audit |
| Sensors | Fresh PointCloud2, RGB Image, explicitly uncalibrated-or-measured CameraInfo, and IMU with verified frames/QoS | sensor audit/dashboard |
| Stop | At goal tolerance, cancel, stale command and fault all result in zero motion/StopMove | supervised timestamped test log |
| UI | Camera, lidar, map, map-frame robot pose, odom and task state visible together | dashboard screenshot/video |
| Map operator UI | Mapping Save/Load controls are reachable only on `127.0.0.1:8091` and persist the selected map under the ignored deployment data directory | loopback listener audit + saved map lifecycle sample |

## Offline status

Code/config tests can establish schema, safety defaults, phrase resolution and
watchdog logic. They cannot establish physical calibration, DDS discovery,
localization quality, obstacle clearance, traction or real stopping distance.
Those cells remain pending until the hardware checklist is completed.

Run `bash scripts/demo_offline_voice_e2e.sh` for the deterministic offline
contract proof. It covers ASR final → Liaison/Pilot-equivalent tool selection →
verified saved Pose/map generation → fake Robonix navigation RUNNING →
SUCCEEDED → read-only dashboard status, with socket and ROS/motion-surface
tripwires. Its fixture result must never be reported as a physical pass.

Run `bash scripts/gate2_replay_acceptance.sh` for Gate 2 preflight, or add a
reviewed local scenario and `--run` for the real isolated replay. The harness
measures Mapping/Nav2 TF, both costmaps marking and clearing, goal cancel,
sensor-loss stop output, and simultaneous Dashboard telemetry. Without a real
bag/map/dependencies it returns `SKIP` (77); the checked-in
`tests/fixtures/gate2_evidence.json` path returns `FIXTURE_ONLY` (77). Neither
is an acceptance pass. See
[GATE2_REPLAY_ACCEPTANCE.md](GATE2_REPLAY_ACCEPTANCE.md).

## First usable-version scope

The target is a pre-mapped laboratory with a saved vending-machine approach
pose. Real-time visual discovery of an unknown vending machine is a later
bonus and is not an acceptance dependency.
