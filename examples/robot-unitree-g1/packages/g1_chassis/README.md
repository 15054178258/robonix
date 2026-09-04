# G1 Chassis Primitive

Guarded Unitree G1 chassis adapter using the SDK2 LocoClient.

## Architecture

```
[ROS2 /cmd_vel] → g1_chassis_adapter_node → IPC socket → g1_loco_daemon → SDK2 LocoClient → G1 robot
```

### Components

| Component | Language | Purpose |
|---|---|---|
| `g1_loco_daemon` | C++ | IPC server wrapping SDK2 LocoClient. 300 ms watchdog, velocity clamping, motion gating. |
| `g1_chassis_adapter_node` | C++ (ROS2) | Subscribes to `/cmd_vel`, forwards commands to daemon via Unix socket, and publishes odom plus waist joint-state heartbeats. |
| `g1_chassis/main.py` | Python | Robonix primitive provider — spawns adapter + daemon, declares ROS2 capabilities. |

### Safety

- **Motion disabled by default** — daemon rejects all velocity commands unless `--allow-motion` is passed at daemon startup, which requires the `G1_PHYSICAL_MOTION_APPROVED` acknowledgement.
- **300 ms watchdog** — if no valid cmd_vel arrives within 300 ms, the daemon issues StopMove and faults.
- **Velocity limits** — vx, vy, omega are clamped to configured maximums.
- **Zero-velocity stop** — a cmd_vel with all zeros updates the watchdog but does not issue movement.
- **Adapter disconnect** — if the IPC peer disconnects, the daemon immediately issues StopMove.

### ROS outputs

- `/odom` carries the stationary odometry heartbeat used by mapping and navigation while real G1 odometry is not integrated.
- `/joint_states` carries neutral waist joint positions so `robot_state_publisher` can connect `base_link` to torso-mounted sensors such as `mid360_link`.

### IPC Protocol

- **Socket**: Unix stream socket, path set by `G1_IPC_SOCKET` env var.
- **Command packet** (24 bytes): type (1) + sequence (1) + reserved (6) + vx (4) + vy (4) + omega (4) — velocities are fixed-point ×10000.
- **Reply packet** (16 bytes): type (1) + sequence (1) + code (1) + armed (1) + faulted (1).

## Building

```bash
export UNITREE_SDK2_DIR=/path/to/unitree_sdk2-main
bash scripts/build.sh
```

## Runtime

The provider (`main.py`) is launched by the Robonix primitive engine. It resolves:

- `G1_NETWORK_INTERFACE` — ethernet interface to the G1 (default: `lo`)
- `G1_ALLOW_MOTION` — `"true"` to enable motion (default: `false`)
