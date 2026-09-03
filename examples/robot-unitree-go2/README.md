# Robonix deployment for Unitree Go2

This repository adapts a Unitree Go2 to Robonix and Nav2 using only the
high-level Unitree Sport service. It standardizes chassis odometry, IMU,
onboard lidar and video; supplies the Go2 model/TF tree; configures RTAB-Map
and Nav2; maps Chinese semantic landmark names to verified map poses; and
provides loopback-only Client, Scene, Mapping and dashboard operator pages.

All non-DDS control/capability/audio/dashboard listeners are loopback-only.
Scene and Mapping administration remain unavailable on public interfaces;
remote viewing uses an authenticated SSH tunnel.

The supported semantic task shape is:

> “请带我去<已验证地标>” / “请带我回到地图起点”

The production path is:

```text
Chinese speech -> Robonix ASR/Liaison/Pilot
-> semantic_navigation.navigate_landmark("<verified landmark>")
-> verified pose in the active saved map
-> Robonix navigation service -> Nav2 NavigateToPose
-> guarded /cmd_vel -> Go2 chassis adapter -> Unitree SportClient
```

Motion is **off by default** and boot never arms the chassis. No posture or
low-level motor API is implemented. Read [docs/SAFETY.md](docs/SAFETY.md)
before connecting hardware.

On 2026-07-31, the supervised reference deployment completed a real Chinese
Client voice request to a long-distance mapped target and a Client-dispatched
autonomous return. Stop, cancel, watchdog and remote takeover remained in the
control path. See the
[supervised full-stack validation note](docs/reports/2026-07-31/README.md).

Publisher ownership is explicit. `GO2_RUNTIME_PLACEMENT=workstation-local`
keeps the existing all-on-workstation behavior. The reviewed split profiles
are `workstation-full-nx-sensors` (NX runs `--sensors-only --camera`) and
`workstation-ui-nx-full` (NX runs its full read-only profile with `--camera`,
while the workstation starts only the telemetry UI). Every start performs a
bounded, read-only publisher-count preflight for camera, lidar/IMU, odometry
and `/tf_static`; a missing or duplicate owner stops startup.

An additional default-disabled
`workstation-full-nomotion-corrected` commissioning profile can qualify one
reviewed fixed source-clock offset and expose only private corrected state,
MID-360 IMU/cloud and lidar-odometry copies. It cannot authorize motion,
cannot satisfy canonical chassis `/odom`, and is not a navigation acceptance
result. A local generator derives the offset and four exact writer GIDs from
same-session read-only evidence; a long-lived graph monitor keeps those GIDs
bound throughout the run. See
[the no-motion timestamp correction contract](docs/WORKSTATION_NOMOTION_TIMESTAMP_CORRECTION.md).

## Clone and inspect safely

```bash
git clone --recursive https://github.com/syswonder/robot-unitree-go2.git
cd robot-unitree-go2
git submodule update --init --recursive
./scripts/validate_offline.sh
```

The validation command is offline: it does not initialize ROS/DDS, start a
Unitree SDK client or contact hardware. See
[known upstream and commissioning limits](docs/KNOWN_LIMITATIONS.md) before
attempting a build or boot.

To inspect the deterministic Chinese voice-to-dashboard contract proof by
itself, run:

```bash
bash scripts/demo_offline_voice_e2e.sh
```

This command uses fixed in-memory ASR, Pilot and navigation fixtures together
with the real map-lifecycle guard, landmark resolver, semantic run registry and
dashboard state. It opens no ROS graph or network socket and has no publisher
or motion API. A pass proves interface wiring only, not physical navigation.

Gate 2 has a separate localhost-only rosbag harness:

```bash
bash scripts/gate2_replay_acceptance.sh
```

With no real capture/map or ROS navigation dependencies it deliberately
returns `SKIP` (exit 77). Its checked-in evaluator fixture deliberately returns
`FIXTURE_ONLY` (exit 77). A real `PASS` requires all measured Mapping/Nav2,
costmap, TF, cancel, sensor-loss stop and simultaneous UI checks described in
[the Gate 2 runbook](docs/GATE2_REPLAY_ACCEPTANCE.md); the harness never starts
the Unitree transport or connects to a physical Go2 interface.

## What is ready without hardware

- Deployment/Catalog manifests and Soma robot model.
- Pinned official `unitree_ros2` and `unitree_sdk2` git submodules; Unitree
  motion examples are never built or run by this repository.
- Official Go2 model assets with a `base_link` navigation root.
- Fail-closed chassis and sensor adapters with process isolation.
- Conservative RTAB-Map/Nav2 configuration and non-spinning recovery tree.
- Deterministic Chinese landmark resolver and exact-phrase tests.
- Read-only UI for image, lidar, map, pose, odometry and task state.
- Static and offline unit tests; no test publishes a velocity command.

## Hardware-dependent gates

The operator must identify the dedicated wired NIC, verify the real Go2 topic
names/QoS/frames, measure sensor extrinsics, create/load the laboratory map,
and save a verified approach pose in
`config/semantic_landmarks.local.yaml`, including the exact `(map_id,
generation)` from the mapping lifecycle while it is in localization mode.
Until those checks pass on a particular deployment, semantic navigation rejects
the target and motion remains disarmed. Passing them on the supervised reference
Go2 does not authorize another robot or site automatically.

See [docs/HARDWARE_CHECKLIST.md](docs/HARDWARE_CHECKLIST.md) for the exact
operator steps and [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) for build/start
commands.

## Licenses

Integration code is Apache-2.0. The aggregate deployment also redistributes
BSD-3-Clause Unitree code/model assets, MIT dependencies and the private
CycloneDDS runtime under EPL-2.0 OR EDL-1.0. See [NOTICE](NOTICE),
[THIRD_PARTY.md](THIRD_PARTY.md), and the authoritative license files retained
in the pinned submodules.
