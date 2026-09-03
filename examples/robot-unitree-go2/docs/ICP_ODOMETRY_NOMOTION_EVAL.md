# Private ICP odometry stationary evaluation

This path measures whether RTAB-Map ICP odometry from the Go2 MID-360 is more
stable at rest than the current robot-provided yaw. It is diagnostic evidence
only and is deliberately disconnected from Mapping, Nav2 and chassis control.

## Isolation contract

- one `rtabmap_odom/icp_odometry` producer;
- default inputs: `/scanner/cloud`, `/scanner/imu`, `/tf_static`; canonical dynamic
  `/tf` is deliberately disconnected because both MID-360 extrinsics used by
  this stationary test are static;
- output: `/robonix/nomotion/icp_eval/odom` (plus node-relative diagnostics
  below the same private namespace);
- odometry frame: `robonix_nomotion_icp_odom`;
- the child remains `base_link` only so the existing verified static transforms
  can place `utlidar_lidar` and `utlidar_imu`; `publish_tf=false` prevents TF
  messages;
- RTAB-Map eagerly creates a TF publisher endpoint even with publication
  disabled, so that endpoint and its dynamic-TF listener are remapped with the
  exact absolute-name rule `/tf:=/robonix/nomotion/icp_eval/tf_disabled`.
  The observer derives the ICP DDS participant prefix from the private odom
  publisher, so it also recognizes TF2's hidden `transform_listener_impl_*`
  node. It requires the same participant to own exactly one private TF
  publisher and the private/static TF subscriptions, zero messages on the
  private endpoint, and zero same-participant publisher or subscriber on
  canonical `/tf` throughout the run;
- no `/odom`, motion-topic, service, action or Unitree SDK output;
- a second ICP run is rejected by an exclusive lock and ROS graph ownership
  check.

The supervisor is subscription-only. It compares the private result with
`/odom`, checks source/publisher ownership, watches the active timestamp
session for `fault.json`, and terminates its exact child process on every exit.
It also records private `odom_info` loss state, ICP ratio/correction,
correspondence count and guess magnitude directly, rather than inferring them
from interleaved process logs. Valid-pose rate/gap/freshness are gated
separately from the continuously published null messages used to signal loss.

Both input profiles explicitly set `Odom/GuessMotion=false` and
`Odom/ResetCountdown=1`. A prior stationary run accepted one false transform,
then RTAB-Map's default constant-velocity model extrapolated that error by
about 0.10 m per cloud until the guess diverged. The diagnostic now disables
that unverified extrapolation and resets immediately after a lost match; it
does not loosen either motion limit.

### Controlled cloud-only comparison

`--input-profile cloud-only` changes only the IMU initialization/check flags
and remaps the node's generic IMU input to the publisher-free private topic
`/robonix/nomotion/icp_eval/imu_disabled`. The supervisor proves from the DDS
participant prefix that this run never subscribes to `/scanner/imu`. This is
an A/B diagnostic for the raw MID-360 attitude path; it does not make the
result a canonical odometry source.

## Required gates

Run only while the corrected workstation **no-motion** stack is current. The
runner requires matching approval/ready/identity session IDs and writer GIDs,
`allow_motion: false`, isolated chassis input, one cloud publisher, one IMU
publisher, and no existing private ICP publisher. The approval must remain
valid for the requested duration plus 90 seconds.

The default stationary gate observes 10 minutes after a 30-second warm-up and
requires at least 5 Hz output, no gap over 1 second, no invalid/frame-mismatched
messages, at most 5 cm translation drift, at most 2 degrees end-to-end yaw
drift, and at most 0.25 degrees/minute fitted yaw drift. These are evaluation
thresholds, not a claim that the source is navigation-ready.
CLI threshold options may only tighten these defaults; the parser rejects a
weaker limit.

## Run

Use the current session and its exact active v3 approval. Do not point this at
an expired or faulted session.

```bash
cd /home/zxq/workspace/robonix-go2/packages/robot-unitree-go2
source /opt/ros/humble/setup.bash
source rbnx-build/unitree_ros2/install/setup.bash

python3 scripts/icp_odometry_nomotion_eval.py \
  --session-dir "$(sed -n 's/^run_dir=//p' rbnx-build/run/workstation-nomotion-current.session)" \
  --approval rbnx-build/run/<current-v3-approval.json> \
  --duration-seconds 600 \
  --ack I_APPROVE_PRIVATE_READONLY_ICP
```

For the controlled IMU-ablation run, add:

```bash
  --input-profile cloud-only
```

For a full 15-minute observation, use `--duration-seconds 900` only with an
approval that still has at least 990 seconds remaining. Evidence is created
under `logs/icp-odom-nomotion/<UTC>-stationary/`; existing evidence is never
overwritten. `summary.json` always says `safe_for_motion: false`.

Do not wire the private result into `/odom`, TF, Mapping or Nav2 unless a later
review accepts repeated stationary and staged-motion evidence and separately
implements canonical-odometry ownership and fail-closed health handling.
