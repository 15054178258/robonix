---
description: Read-only external Intel RealSense D435i RGB-D topic registrar.
---

# Go2 D435i external capability

Provider ID: `go2_d435i`

This provider does not open the USB camera and does not launch
`realsense2_camera`. The Jetson Orin owns the RealSense ROS 2 publishers. This
workstation-side package observes those external topics without creating a ROS
publisher, applies a bounded RGB/depth/intrinsics quality gate, and only then
registers:

- `robonix/primitive/camera/rgb`;
- `robonix/primitive/camera/depth`;
- `robonix/primitive/camera/intrinsics`.

The camera lifecycle capability represents validation and Atlas registration
of the external streams. It does not own the external camera process.

The first version intentionally exposes no camera extrinsics, snapshot tool, or
IMU capability. The physical `base_link` to D435i mount transform must be
measured and reviewed before RGB-D data is fused into mapping or navigation.
The D435i IMU does not replace the deployment's canonical navigation IMU.
