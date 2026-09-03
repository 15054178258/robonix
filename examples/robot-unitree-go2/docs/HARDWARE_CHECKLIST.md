# Operator hardware checklist

These are the only steps that require physical access or privileged input.

## 1. Cable and NIC identification

1. Power the Go2 normally and keep it stationary in the official app.
2. Connect one Ethernet cable directly between the Ubuntu computer and Go2.
3. In a terminal run `ip -br link` once with the cable disconnected and once
   connected. The interface whose state changes to `UP` is the candidate.
4. Record its exact name. Do not choose Wi-Fi, loopback, Docker or a VPN.
5. Send that interface name to the maintainer before any network change.

The interface will later be configured alone as `192.168.123.99/24`, without
gateway or DNS. That is a privileged network change and must be approved at
the time it is performed. Follow this repository's self-contained
[read-only bring-up and recovery procedure](GO2_READONLY_BRINGUP.md).

## 2. Read-only topic audit

With motion still locked, run this repository's tools to capture topic
names, types, QoS, rates and frame IDs. In particular verify:

- `/sportmodestate` or `/lf/sportmodestate`;
- `/lowstate`, IMU and robot state;
- `/utlidar/cloud` (PointCloud2) and its real frame;
- video/camera availability and resolution;
- `/wirelesscontroller`, `/tf`, `/tf_static`.

Do not run any Unitree example program. Do not run `ros2 topic pub`.

For this EDU firmware, measured `SportModeState.error_code` values including
`100`, `1013`, and `2010` are **not documented in Unitree's public message or
SDK error tables**. Never label them as modes or healthy states. Zero is the
default-safe value. A reviewed robot-specific value may only be configured as
an opaque compatibility marker through `GO2_ALLOWED_STATE_MARKERS`; any later
change latches canonical state and motion closed. The observed dog also
reported `mode=0` across remote state changes, so mode alone is not a
control-health proof on this firmware.

The current hardware baseline is Go2 EDU hardware V2.0, system software
V1.1.11, and Unitree lidar software 1.0.0.38.  The App currently offers
V1.1.15 with only the note “优化配网问题”.  Do not install that update during
commissioning: the dedicated wired DDS link is already healthy, and changing
firmware would invalidate the recorded topic, QoS, service, timestamp, and
error-code baseline.  Treat any later update as a separate maintenance window
and repeat the complete read-only audit afterwards.

The App's grey `sport_mode` entry has the same name as Unitree's switchable
default motion-control service, but public source does not establish the App
button's exact implementation, and the current dog continued publishing
SportModeState while the App showed it grey. Keep `advanced_sport`, `ai_sport`
and OTA off. Do not toggle `sport_mode` until the explicit single-service
control-authority experiment after all preceding gates pass.

Primary references: Unitree's pinned
[ROS 2 state/API documentation](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/README.md),
[SportModeState message](https://github.com/unitreerobotics/unitree_ros2/blob/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg),
and mutating
[`sport_mode` ServiceSwitch example](https://github.com/unitreerobotics/unitree_sdk2/blob/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b/example/go2/go2_robot_state_client.cpp).

## 3. Physical measurements

Provide or measure, in metres/radians:

- navigation-state maximum moving footprint, including leg sway;
- `base_link -> radar` translation and rotation;
- `base_link -> front_camera` translation and rotation;
- confirmation of `base_link -> imu`;
- camera resolution and calibration file;
- Go2 hardware edition and onboard compute/firmware versions.

Compare against the official model before changing URDF. Record the result in
the acceptance log, not in a file containing device credentials.

## 4. Map and vending-machine pose

1. Build the lab map at walking speed under direct supervision, or load an
   existing map whose `map_id` is known.
2. In localization mode, verify the robot pose remains aligned while the dog
   is stationary and while it is manually relocated according to the mapping
   procedure.
3. Scene's debug UI and Mapping's unauthenticated administration UI are
   deliberately disabled. Physically place the robot base, using only official
   Unitree controls and the approved supervised procedure, at a collision-free
   approach point about 0.8 m in front of the vending machine and facing it.
   Do not use the machine centre as the goal.
4. Check the entire Go2 footprint plus clearance in the read-only dashboard's
   occupancy map and physically on the floor. A separately launched local RViz
   may be used only for visual cross-checking; never click `2D Goal Pose`.
5. While stationary, use a bounded read-only `tf2_echo map base_link` sample
   to measure the robot-base x, y and yaw at that verified approach point.
6. With mapping in `localization` mode, use a bounded, read-only echo of
   `/robonix/map/lifecycle` and record its exact `map_id` and `generation`.
7. Record x, y and yaw. The maintainer will run `scripts/set_landmark.py` with
   that `--generation` to create the ignored local semantic file. Any later
   map reset/load requires re-verifying and re-saving the pose.

## 5. Credentials and login

- Complete GitHub CLI web login locally when asked. Never paste the one-time
  code, password or token into chat.
- If a Pilot model endpoint is used, put its URL/key/model only in `.env`.
- Local Chinese ASR/TTS needs no cloud credential. If a private remote backend
  is later selected, enter its credentials only in `.env` or an approved
  secret store.
- SSH to an onboard computer is optional and initially read-only. Enter the
  password interactively; never save it in this repository.
