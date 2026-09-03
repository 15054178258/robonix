# Gate 2 isolated rosbag replay acceptance

This gate validates the ROS navigation stack before it is allowed to share a
graph with a physical Go2. It is a replay test, not a motion test. The runner
does not select a physical interface, import Unitree SDK2, start a chassis
adapter, or forward a velocity command.

## Hard isolation

Every real run enforces all of these settings in its child environment:

- a scenario-owned non-zero `ROS_DOMAIN_ID`;
- `ROS_LOCALHOST_ONLY=1` and `ROS2CLI_NO_DAEMON=1`;
- `ROS_LOCALHOST_ONLY=1` selecting only loopback, with CycloneDDS multicast
  disabled (the URI intentionally does not select `lo` a second time);
- all inherited `GO2_*` and `UNITREE_*` values removed, followed by
  `GO2_ALLOW_MOTION=false` and `GO2_OPERATOR_PRESENT=false`;
- browser voice disabled and Dashboard bound to `127.0.0.1`;
- an empty isolated ROS node graph before any child starts.

The runner starts only:

1. the subscription-only `/cmd_vel` evidence sink;
2. the Gate 2 telemetry/Nav2-action observer;
3. `robot_state_publisher` from the checked-in Go2 URDF;
4. `pointcloud_to_laserscan`;
5. the audited RTAB-Map Mapping launch in localization mode;
6. Nav2 `navigation_launch.py` with the conservative Go2 parameters;
7. the loopback read-only Dashboard;
8. `ros2 bag play` with an explicit sensor/state topic allowlist.

The Nav2 action observer sends and cancels a goal only inside this isolated
domain. Nav2's `/cmd_vel` ends at the no-op sink. The sink has no publisher,
client, service, network socket, Unitree import, or forwarding callback.

Do not connect this process to the Go2 NIC. Do not source a live Go2 DDS
profile in the same terminal. The enforced loopback profile is defense in
depth, not permission to attach hardware.

## Inputs that must be measured

Copy the template to the ignored local path:

```bash
cp config/gate2_replay.example.yaml config/gate2_replay.local.yaml
```

Place a real read-only capture and saved RTAB-Map database below the ignored
repository-local directory:

```text
rbnx-build/gate2/input/<bag-directory>/metadata.yaml
rbnx-build/gate2/input/<bag-directory>/*.db3
rbnx-build/gate2/input/<saved-map>.db
```

Do not point the scenario at `/mnt`, an NTFS path, another checkout, or a live
device. The runner rejects paths outside this repository. It also rejects a
bag if its metadata contains `/cmd_vel`, `/lowcmd`, or
`/api/sport/request`, even when that topic is not in the replay allowlist.

The capture must contain, with the exact configured types:

- chassis odometry and the recorded `odom -> base_link` TF;
- PointCloud2 with a measured obstacle-present interval, a later
  obstacle-removed interval, and a bounded cloud gap;
- IMU and camera Image;
- enough duration for goal dispatch, lidar loss/recovery, and goal cancel.

Record or prepare the bag while reading sensors only. The cloud gap must be a
real absence of samples in the configured time window; changing the YAML to
hide existing samples does not satisfy the check. The saved map must match the
recording environment. The runner copies it into the run directory before
starting RTAB-Map, so the source database is not used as mutable runtime state.

Edit these scenario values from the capture rather than accepting the example
numbers:

- replay topic names and ROS message types;
- obstacle, cleared and sensor-gap windows relative to the first positive
  replay `/clock` sample;
- the obstacle centre/radius in both `odom` and `map` frames;
- a known-free Nav2 goal and its dispatch/cancel timestamps;
- marked/cleared cell thresholds and the velocity stop grace.

Preflight rejects missing/empty/wrong-type topics, an undersized bag, generated
output topics in the replay allowlist, forbidden motion topics anywhere in the
bag, a missing map database, an occupied Dashboard port, missing ROS packages,
or an already-populated isolated domain.

## Commands and result meanings

Run preflight without replay:

```bash
bash scripts/gate2_replay_acceptance.sh \
  --scenario config/gate2_replay.local.yaml
```

Run the real isolated replay only after preflight reports
`READY_NOT_RUN`:

```bash
bash scripts/gate2_replay_acceptance.sh \
  --scenario config/gate2_replay.local.yaml \
  --run
```

Exercise the evaluator with checked-in synthetic evidence:

```bash
bash scripts/gate2_replay_acceptance.sh \
  --scenario config/gate2_replay.example.yaml \
  --fixture-evidence tests/fixtures/gate2_evidence.json
```

Result and exit-code contract:

| Status | Exit | Meaning |
|---|---:|---|
| `PASS` | 0 | A real fingerprinted bag run satisfied every required check. |
| `FAIL` | 1 | Invalid input, unsafe metadata, orchestration failure, or failed measured check. |
| `SKIP` | 77 | A real bag, map, ROS dependency, or another prerequisite is absent. |
| `READY_NOT_RUN` | 77 | Preflight passed, but no replay was requested. |
| `FIXTURE_ONLY` | 77 | The evaluator was exercised with synthetic evidence; never an acceptance pass. |

Evidence and per-process logs are written only below the ignored directory:

```text
rbnx-build/gate2/runs/<UTC timestamp>/report.json
rbnx-build/gate2/runs/<UTC timestamp>/observer.json
rbnx-build/gate2/runs/<UTC timestamp>/cmd_vel_sink.json
rbnx-build/gate2/runs/<UTC timestamp>/*.log
```

Cleanup signals only process groups created by the runner. It never uses
`killall` or `pkill`.

## Required real-run checks

A `PASS` requires all of the following in the same run:

- evidence source is `rosbag` with a SHA-256 fingerprint;
- all expected processes stayed alive through replay;
- sink declares subscription-only behavior and zero forwarded commands;
- `map -> odom` and `odom -> base_link` were both observed;
- map, odom, cloud, generated scan, IMU, camera and Nav2 status were received;
- the isolated `NavigateToPose` goal was accepted, cancel was accepted, and
  its final result was `CANCELED`;
- both local and global costmap regions became marked in the obstacle window
  and cleared by the configured amount in the cleared window;
- scans existed before and after, but not during, the sensor-gap window;
- a non-zero Nav2 command existed before sensor loss, followed by no non-zero
  command after the grace deadline and an explicit zero or timeout-length
  command silence;
- one Dashboard snapshot simultaneously showed camera, lidar, map, map-frame
  pose, odometry and Nav2 status while telemetry remained read-only.

The non-zero-before-gap requirement prevents a permanently idle Nav2 stack
from being mislabeled as a successful sensor-loss stop. No check is converted
to a warning, and fixture evidence can never satisfy the source-authenticity
check.

## Current expected status

When no real capture/map exists, or when Nav2/RTAB-Map/projection packages are
not installed, preflight must report `SKIP` and enumerate each missing item.
That is the correct Gate 2 result. It is not permission to proceed to a live
robot gate.
