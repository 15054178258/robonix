# Go2 real-hardware Nav2 readiness audit

This is an offline/source audit. It does not authorize a live goal or any
physical motion. `UNKNOWN`, skipped evidence, fixture evidence, and a working
raw-data UI are all **not ready**.

## Current decision

The shortest path is blocked before Nav2:

1. The 2026-07-18 read-only capture shows the chassis, cloud and lidar IMU
   source clocks about 748 seconds behind workstation receipt time. The
   chassis and sensor relays correctly reject data older than their 0.20/0.50
   second limits. Do not restamp it.
2. Successive bounded chassis captures report `SportModeState.mode=0` with
   undocumented non-zero `error_code` values (`100`, later `1013`, and most
   recently `2010`). Zero remains the default-safe marker. A robot-specific
   value may be used only through the explicit opaque-marker compatibility
   policy; it assigns no mode/health meaning and any value change latches
   closed. The current session still needs that explicit reviewed input.
3. There is no saved map below `rbnx-build/data/maps`, no verified local
   vending-machine landmark, and no fingerprinted real-bag Gate 2 pass.

The raw hardware evidence is nevertheless useful: front video 28.337 Hz,
LowState 500.755 Hz, SportModeState 306.147 Hz, lidar cloud 15.394 Hz, lidar
IMU 239.730 Hz and vendor robot odometry 128.676 Hz were observed. Native TF,
TF static and LaserScan were not observed. See
`docs/HARDWARE_EVIDENCE_2026-07-18.md` and the ignored evidence directories
`logs/go2-readonly/20260718T034021Z-sensors` and
`logs/go2-readonly/20260718T034425Z-time-probe`.

## Minimum sequential gates

### 1. Source clock and SportModeState

Keep the robot stationary and motion disabled. Re-run the subscription-only
probe:

```bash
cd packages/robot-unitree-go2
./scripts/probe_go2_time_readonly.sh
```

Pass only when the relevant source stamps are non-zero, strictly advancing,
have no regressions, are comparable to the runtime clock inside the committed
age/future bounds, and the chassis reports `error_code==0` or one exact
operator-reviewed opaque value configured for this robot/session. The latter
is compatibility only, not a healthy-state interpretation. Clock adjustment
is a separate privileged/high-impact gate; do not perform it as part of this
read-only probe.

Source enforcement:

- `packages/go2_chassis/config/adapter.yaml`: 0.20 s state/source age, 0.05 s
  future skew;
- `scripts/staged_nav2_readiness.py` and
  `scripts/staged_nav2_motion_guard.py`: the state receipt and source-age
  checks share that same 0.20 s live-state boundary;
- `packages/go2_sensors/config/go2_sensors.yaml`: 0.50 s cloud and 0.20 s IMU
  source age, 0.05 s future skew;
- `docs/TIME_SYNC.md`: evidence and formal clock-discipline acceptance.

### 2. Canonical stationary state and sensors

Start only the motion-disabled deployment (`GO2_ALLOW_MOTION=false`). Require
one publisher, exact type, fresh source stamp and the following frames:

| Output | Type | Required frame/relationship |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `odom`, child `base_link` |
| `/scanner/cloud` | `sensor_msgs/msg/PointCloud2` | `utlidar_lidar` |
| `/scanner/imu` | `sensor_msgs/msg/Imu` | `imu` |
| `/scanner/scan` | `sensor_msgs/msg/LaserScan` | `base_link` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | `front_camera` |

Capture bounded, subscription-only evidence from the workspace root:

```bash
bash scripts/check_sensors.sh
```

`robonix_manifest.yaml` is the authoritative routing contract. The camera
calibration currently deliberately reports uncalibrated intrinsics; it is
sufficient for display, not metric visual perception.

### 3. URDF, extrinsics and TF ownership

Collect bounded TF evidence:

```bash
bash scripts/check_tf.sh
```

Require exactly one owner for each part of this tree:

```text
RTAB-Map:               map -> odom
Go2 chassis adapter:           odom -> base_link
robot_state_publisher:                 base/imu/lidar/camera static links
```

The transforms in `packages/go2_description/urdf/go2_robonix.urdf` are
provisional. Measure the actual sensor model, mount origin, orientation and
camera intrinsics on this robot. In particular, do not assume that the current
Go2 `radar` plus MID-360 embedded-IMU offsets describe the photographed
hardware exactly.

### 4. Mapping, then localization

Keep Robonix motion disabled. The operator may use only the official Unitree
remote to walk the robot slowly while RTAB-Map is in mapping mode. Save the
map through the loopback Mapping UI, then restart in localization mode.

Pass when all of the following are simultaneously measured:

- a non-empty saved RTAB-Map database under the ignored deployment map data;
- fresh `/map` and one `map -> odom` publisher;
- stable `map -> base_link` while stationary and during supervised manual
  relocation;
- a single `/robonix/map/lifecycle` sample with the exact non-empty `map_id`,
  `mode: localization`, and positive `generation`.

Read the lifecycle tuple without publishing:

```bash
timeout 5s ros2 topic echo --once \
  /robonix/map/lifecycle map/msg/MapLifecycle
```

### 5. Verified semantic approach pose

Using the official remote, place the robot at a collision-free approach pose
about 0.8 m in front of the vending machine and facing it. Measure
`map -> base_link`; never save the object centre as the chassis goal. Then
create the ignored binding from that same localization generation:

```bash
python3 scripts/set_landmark.py \
  --map-id <EXACT_MAP_ID> \
  --generation <EXACT_POSITIVE_GENERATION> \
  --x <MEASURED_X> --y <MEASURED_Y> --yaw <MEASURED_YAW_RAD> \
  --measured-by <OPERATOR> \
  --confirm-free-space YES_POSE_AND_FOOTPRINT_ARE_CLEAR
```

Set `SEMANTIC_LANDMARKS_FILE=config/semantic_landmarks.local.yaml`. The public
template has generation zero, `verified: false`, and a zero pose, so it must
continue to fail closed.

### 6. Full-stack read-only readiness

After the stack settles:

```bash
bash scripts/check_stack_readiness.sh
```

Every result must be `PASS`; `UNKNOWN` blocks. The gate requires fresh camera,
PointCloud2, projected LaserScan, IMU, odometry, map, TF, map lifecycle,
semantic binding, active Nav2 lifecycle nodes/actions and loopback Robonix/UI
health. Evidence is written under ignored `logs/readiness`.

### 7. Fingerprinted real-bag Gate 2

Copy `config/gate2_replay.example.yaml` to ignored
`config/gate2_replay.local.yaml` and replace every phase/goal/region with
measurements from a real read-only bag and matching saved map. Then run:

```bash
bash scripts/gate2_replay_acceptance.sh \
  --scenario config/gate2_replay.local.yaml

bash scripts/gate2_replay_acceptance.sh \
  --scenario config/gate2_replay.local.yaml \
  --run
```

Only a fingerprinted real-bag `PASS` counts. `SKIP`, `READY_NOT_RUN` and
`FIXTURE_ONLY` do not. The replay must prove obstacle marking/clearing,
LaserScan loss/recovery, goal cancel and the stop/silence contract, with
`/cmd_vel` terminating at the subscription-only sink.

### 8. Stop evidence before any live movement

First prove in replay/no-motion evidence that stale chassis state, lidar loss,
navigation failure/cancel, velocity-guard failure, adapter disconnect and SDK
disconnect all terminate in zero/silence within the committed watchdogs.

Only a later, separately approved supervised session may progress through:

```text
zero-command arm preparation
  -> centimetre-scale low-speed motion
  -> cancel and official-remote stop proof
  -> short known-free Nav2 goal
  -> semantic text goal
  -> Chinese voice goal
```

Do not make the first live motion test a voice or semantic navigation task.

## Offline evidence from this audit

The following focused tests pass without ROS, network access or motion:

```bash
python3 -m unittest \
  tests/test_stack_readiness.py \
  tests/test_navigation_configuration.py \
  tests/test_nomotion_navigation_route.py
```

The readiness gate now explicitly requires `/scanner/scan`, the projected
LaserScan consumed by both Nav2 obstacle layers. Previously a fresh cloud
could pass the topic section even if scan projection itself was absent.
