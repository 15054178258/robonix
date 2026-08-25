# Go2 description

The URDF and DAE assets are sourced from the official
`unitreerobotics/unitree_ros` repository at commit
`d96d8f63ae17a7108d4f7229c00ef875ba7129c9` and retain the included
BSD-3-Clause license.

`urdf/go2_robonix.urdf` adds a zero-offset `base_link -> base` transform around
the otherwise unchanged official model. The official `base -> radar` joint is
retained. The robot-specific `radar -> utlidar_lidar` topic-frame alias was
calibrated on 2026-07-23 by rigidly matching 22,801 points from same-stamp
`/utlidar/cloud` and vendor `/utlidar/cloud_base` messages. The matched-point
residual was 0.000085 mm median, 0.000751 mm p99 and 0.00192 mm maximum. The
resulting composite `base_link -> utlidar_lidar` transform is:

```text
xyz = 0.282160001 0 0
rpy = -2.92072011 -0.14132399 -1.01052989
```

The calibrated alias itself is:

```text
xyz = -0.005152655 0 -0.047108077
rpy = -0.001508482 -0.000981520 -2.146758680
```

The model also connects the live Unitree UniLiDAR IMU topic frame as
`utlidar_lidar -> utlidar_imu` using Unitree's common L1/L2 IMU position
`(-0.007698, -0.014655, 0.00667) m` and aligned axes from the official
[UniLiDAR SDK](https://github.com/unitreerobotics/unilidar_sdk).
Runtime topics and rates identify the device as Unitree UniLiDAR with high
confidence; the exact L1/L2 hardware revision must still be confirmed from its
physical label.
The model already contains the official Go2 `imu`, `radar`, and `front_camera`
frames. The camera extrinsic still requires physical
measurement/verification before metric visual perception or supervised
navigation.

This directory is also the deployment's Robonix robot-description primitive.
It accepts only the URDF served by Soma when it exactly matches the pinned
file, then runs the standard ROS 2 `robot_state_publisher` under
`rmw_cyclonedds_cpp`. It exposes no movement or posture capability.
