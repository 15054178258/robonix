---
description: Read-only Unitree Go2 lidar, IMU, and RGB camera sensor provider.
---

# Go2 sensor capability

Provider ID: `go2_sensors`

This provider exposes only standard ROS 2 sensor streams:

- `robonix/primitive/lidar/lidar3d` → `/scanner/cloud`
- `robonix/primitive/imu/imu` → deployment-configured IMU relay output
  (`/scanner/imu` in the root Go2 manifest; package default
  `/sensors/imu/data`)
- `robonix/primitive/camera/rgb` → `/camera/color/image_raw`

The bridge also publishes a truthful ROS `CameraInfo`. Its checked-in zero
camera matrix (`K[0] == 0`) means uncalibrated, so this release intentionally
does not advertise `robonix/primitive/camera/intrinsics`. That contract must be
added only in a versioned release after the exact deployment camera is measured
and the calibration is validated.

The lidar driver lifecycle starts and stops the combined read-only provider.
RGB is mandatory: activation requires fresh lidar, IMU, RGB, and CameraInfo
samples before all manifest data endpoints are declared. Activation failure
terminates every child process.
