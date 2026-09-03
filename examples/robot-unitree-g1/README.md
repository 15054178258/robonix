# Unitree G1 Deployment

Unitree G1 bipedal humanoid robot deployment for Robonix semantic navigation.

## Hardware

| Component | Model | Location |
|---|---|---|
| LiDAR | Livox Mid-360 3D | Head |
| RGB-D Camera | Intel RealSense D435i | Head |
| Locomotion Controller | Unitree SDK2 LocoClient | — |
| Arms | Dual arms (Dex3 hand option) | Torso |

## Getting Started

```bash
# 1. Initialize SDK2 submodule
git submodule update --init --recursive third_party/unitree_sdk2

# 2. Build
bash build.sh

# 3. Boot (no-motion for first-time)
bash start.sh --no-motion

# 4. Boot with motion (after operator confirms)
bash start.sh
```

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `G1_NETWORK_INTERFACE` | `lo` | Ethernet interface to the G1 robot |
| `G1_ALLOW_MOTION` | `false` | Enable motion commands (set `true` for supervised operation) |
| `G1_MAP_ID` | — | Map identifier for the current environment |
| `G1_MAP_MODE` | `localization` | `mapping` or `localization` |
| `SPEECH_BACKEND` | `local` | `local` (FunASR) or `tencent` (cloud ASR) |
| `G1_DASHBOARD_PORT` | `8092` | Dashboard web UI port |

## Architecture

```
[ROS2 /cmd_vel] → g1_chassis_adapter → Unix socket → g1_loco_daemon → SDK2 LocoClient → G1
```

- **g1_chassis** — Guarded locomotion primitive with 300 ms watchdog, velocity clamping, and motion gating.
- **mid360_lidar / mid360_imu** — Head-mounted Livox Mid-360 3D LiDAR and IMU (UDP direct read).
- **realsense_camera** — Head-mounted Intel RealSense D435i depth camera for RGB-D mapping.
- **mapping** — RTAB-Map with Mid-360 LiDAR + Realsense depth fusion.
- **nav2** — Nav2 with bipedal-safe scan projection (tall torso clearance).
- **speech** — FunASR wake-word detection + Azure TTS.
- **g1_dashboard** — Operator web UI with camera stream, map, and navigation controls.

## Safety

- Motion is **disabled by default**. Requires `G1_ALLOW_MOTION=true`.
- 300 ms watchdog — missing cmd_vel triggers automatic StopMove.
- Velocity limits clamped in the daemon (vx ≤ 1.0 m/s, ω ≤ 2.0 rad/s).
- No grasp/IK pipeline — arms accept joint commands only.

## Configuration Files

| File | Purpose |
|---|---|
| `robonix_manifest.yaml` | Top-level deployment manifest |
| `soma.yaml` | Robot body description, component tree, capabilities |
| `config/nav2_params_g1.yaml` | Nav2 planner, controller, and costmap parameters |
| `config/rtabmap_params.yaml` | RTAB-Map mapping parameters |
| `config/navigate.xml` | Nav2 BT behavior tree for navigation goals |
| `config/navigate_through_poses.xml` | BT tree for multi-waypoint navigation |
| `config/g1_topics.yaml` | ROS2 topic registry (inputs, outputs, TF frames) |
| `config/robonix_client_settings.yaml` | Robonix Client workspace template |
| `config/semantic_landmarks.yaml` | Semantic landmark definitions for Chinese navigation |
