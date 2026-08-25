# Go2 EDU read-only hardware evidence — 2026-07-18

This record covers one powered Go2 EDU session with the Unitree 100 TOPS NX
expansion computer and MID-360.  It is evidence of network discovery and
subscriber-only sensor access.  It is **not** evidence of physical navigation
or motion-control acceptance.

## Safety state

- No Unitree motion example was run.
- Nothing was published to `/cmd_vel`, `/api/sport/request`, or `/lowcmd`.
- Robonix motion remained disabled.
- The robot was supervised with the paired handheld remote available.

## Dedicated network

The reviewed NetworkManager profile `go2-readonly` was active on
`enp108s0` with exactly `192.168.123.99/24`.  The interface had no gateway,
DNS, IPv6 address, or default route.  The host's Internet default route
remained on Wi-Fi; `192.168.123.0/24` alone was routed through the Go2 cable.

CycloneDDS was bound explicitly to `enp108s0` with
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

## Live subscriber evidence

The bounded audit discovered 121 ROS 2 topics.  Representative observed rates
were:

| Source topic | Type | Observed rate | Frame |
| --- | --- | ---: | --- |
| `/frontvideostream` | `unitree_go/msg/Go2FrontVideoData` | 28.337 Hz | proprietary stream |
| `/lowstate` | `unitree_go/msg/LowState` | 500.755 Hz | n/a |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 306.147 Hz | n/a |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 15.394 Hz | `utlidar_lidar` |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 239.730 Hz | `utlidar_imu` |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 128.676 Hz | `odom -> base_link` |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | 18.808 Hz | `odom` |

The audit also discovered `LowState`, controller state, lidar map products, and
robot-state API endpoints.  It did not discover native `/tf`, `/tf_static`, or
`sensor_msgs/msg/LaserScan`; the Go2 integration must supply the required URDF,
static transforms, dynamic odometry transform, and any PointCloud2-to-LaserScan
conversion under explicit ownership.

The initial `SportModeState` sample reported `mode=0` and `error_code=100`.
Later bounded, subscriber-only captures observed `1013` and then `2010`; the
latest capture held `mode=0`, `gait_type=0`, and `error_code=2010` across 1,578
samples while the supervised robot remained stationary. Unitree's public
message definition separates `mode` from `error_code`, and the pinned public
Go2 headers do not document these telemetry values. Motion therefore remains
fail-closed until the field is resolved from a supported source and a
documented healthy state is measured.

## NX expansion-computer audit

The operator established an interactive SSH session to `unitree@192.168.123.18`
and collected the following read-only facts from the physical 100 TOPS NX
module:

| Item | Measured value |
| --- | --- |
| Operating system | Ubuntu 20.04.5 LTS (Focal) |
| Architecture/kernel | `aarch64`, Linux `5.10.104-tegra` |
| Jetson release | L4T R35.3.1 (`/etc/nv_tegra_release`) |
| Wired interface | `eth0`, `192.168.123.18/24` |
| Docker | 24.0.5 |
| Root filesystem | 469 GiB total, 225 GiB available |
| Memory | 15 GiB total, 13 GiB available; 7.5 GiB swap |
| System clock | `1970-01-02T09:43:28+08:00` |
| Time service | `systemd-timesyncd` active, `NTPSynchronized=no` |
| RTC | approximately 1970-01-01 |

The missing `nvidia-smi` command is not evidence that CUDA is unavailable on a
Jetson; GPU/toolchain availability must be checked with the JetPack/L4T tools
before selecting an accelerated container.

The NX host clock is conclusively unsuitable for ROS/Nav2 work in its current
state.  No time, network, package, or service setting was changed during this
audit.  The NX identity matches the reviewed time-profile preconditions
(`aarch64`, `eth0`, `192.168.123.18/24`).

## Source-clock blocker

A fresh 15-second, subscription-only probe collected 8,717 samples.  Median
source-minus-receipt offsets were:

- `/sportmodestate`: `-748.331103143 s`
- `/lf/sportmodestate`: `-748.3312912595 s`
- `/utlidar/cloud`: `-748.405752 s`
- `/utlidar/imu`: `-748.329476714 s`

The probe returned `safe_for_clock_discipline=false`.  The Ubuntu workstation
was NTP-synchronized, used `Asia/Shanghai`, and kept its RTC in UTC.  Therefore
the current evidence points to the robot-side source clock domain, not the
workstation's Wi-Fi proxy settings.

The DDS writer observed for the Go2 state stream is `192.168.123.161`, which is
distinct from the NX host at `192.168.123.18`.  Correcting the NX wall clock
alone would therefore not prove alignment with the Go2 DDS source clock.  A
same-session correlation and no-adjust probe remain mandatory before any
clock step, timestamp-sensitive ROS deployment, or motion gate.

The adapters correctly reject these stale source stamps.  Do not raise the
freshness bounds or overwrite timestamps without a separately validated,
fail-closed clock-discipline design.  Mapping, localization, costmaps, Nav2,
and physical motion remain blocked while this offset is unresolved.

## Evidence paths

The ignored local evidence is retained under:

- `logs/go2-readonly/20260718T034021Z-sensors/`
- `logs/go2-readonly/20260718T034425Z-time-probe/`

These directories can contain large runtime samples and are intentionally not
committed.  This document records the bounded summary needed for review.
