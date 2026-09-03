# Go2 real-hardware read-only UI diagnostic

This profile exists only to inspect real receive-side hardware data while the
Go2 source-clock offset is unresolved. It is **READ-ONLY DIAGNOSTIC / NOT
NAVIGATION READY** and is not evidence that mapping, localization, TF, Nav2,
semantic navigation, speech, or motion works.

It reads `/utlidar/cloud` and `/utlidar/robot_odom` directly. It starts only the
audited `go2_camera_daemon`, the camera-only `go2_camera_bridge`, and the
subscriber-only dashboard. It does not start the sensor relay, robot
description, chassis adapter, Robonix systems, Mapping, Nav2, or browser voice.

With the separately approved `go2-readonly` NetworkManager profile already
active on the dedicated wired interface, run:

```bash
cd /home/zxq/workspace/robonix-go2/packages/robot-unitree-go2
GO2_NETWORK_INTERFACE=enp108s0 \
  bash scripts/start_readonly_ui_diagnostic.sh
```

Then open `http://127.0.0.1:8092/`. Do not treat raw odometry as localized
`map -> base_link` pose. The UI banner and JSON `profile` fields deliberately
report `navigation_ready: false` and `source_time_trusted: false`.

Stop with `Ctrl-C`. The launcher has one atomic runtime-placement lease and
records every child PID, PGID, and `/proc` start tick. Cleanup signals only
those identity-validated process groups; it never uses name-based process
killing.

Before and after startup, the graph-only gate requires exactly one vendor raw
camera/lidar/odom writer. It also requires the standardized map, TF, odom,
scanner, and Nav2 publisher surfaces to remain absent. All graph queries are
subscriber-free `ros2 topic info --no-daemon` calls bounded by `timeout`.
