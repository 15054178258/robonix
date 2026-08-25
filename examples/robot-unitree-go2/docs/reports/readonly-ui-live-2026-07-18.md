# Go2 read-only UI live evidence — 2026-07-18

This report records a real receive-side hardware run. It is not evidence of
navigation or physical motion.

## Runtime profile

- profile: `readonly-diagnostic`
- interface: `enp108s0` (`192.168.123.99/24`)
- UI: `http://127.0.0.1:8092/`
- health: `ok=true`, `read_only=true`, `ros_connected=true`
- navigation stack: not started
- source time: not trusted
- robot-control publishers and services: not started

## Measured live inputs

The `/api/status` snapshot was captured after approximately 12 minutes of
continuous runtime.

| Input | Observed result |
| --- | --- |
| Front camera | `/camera/color/image_raw`, fresh, 1920x1080 BGR8, approximately 10 Hz |
| Camera quality | healthy, 0% decode error in the current 100-frame window |
| 3D lidar | `/utlidar/cloud`, fresh `PointCloud2`, frame `utlidar_lidar` |
| Raw lidar odometry | `/utlidar/robot_odom`, fresh, frame `odom`, child `base_link` |
| 2D scan | not present |
| Occupancy map | not present |
| `map -> base_link` | not present; `map` frame does not exist |
| Nav2 task status | not present |
| Browser voice | disabled in this diagnostic-only profile |

The raw lidar and odometry source stamps were approximately 748 seconds behind
the workstation clock. They are displayed for diagnosis only and are not
accepted as canonical navigation state.

## Screenshot

![Live Go2 read-only dashboard](go2-readonly-ui-live-20260718-1318.png)

## Acceptance meaning

This run proves that the workstation can receive and render the real Go2
camera, MID-360 point cloud and raw lidar odometry without creating a motion
path. It does not yet prove trusted odometry, TF, mapping, localization, Nav2,
speech, semantic landmark resolution or physical movement.
