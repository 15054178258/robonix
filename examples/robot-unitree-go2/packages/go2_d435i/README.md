# Go2 external D435i registrar

This package is the workstation-side Robonix adapter for a RealSense D435i
physically connected to the Go2's Jetson Orin. It is deliberately an
`external`-only registrar:

- the Orin owns `realsense2_camera` and every camera ROS publisher;
- this package creates three ROS subscriptions and no ROS publisher;
- it never starts a camera process, changes networking, or calls a robot API;
- Atlas data endpoints appear only after a bounded quality gate passes.

## Expected streams

The checked-in defaults follow the upstream RealSense namespace convention:

| role | default topic | expected payload |
| --- | --- | --- |
| RGB | `/go2/d435i/color/image_raw` | `sensor_msgs/Image`, `rgb8` or `bgr8` |
| aligned depth | `/go2/d435i/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`, `16UC1` or `mono16` |
| intrinsics | `/go2/d435i/color/camera_info` | calibrated `sensor_msgs/CameraInfo` |

The RGB and aligned-depth frames default to
`d435i_color_optical_frame`. Deployment configuration must be replaced
with the exact topic and frame names observed from the installed Orin driver;
do not make the root deployment depend on these defaults without that check.

## Runtime configuration

```yaml
source_mode: external
rgb_topic: /go2/d435i/color/image_raw
depth_topic: /go2/d435i/aligned_depth_to_color/image_raw
camera_info_topic: /go2/d435i/color/camera_info
rgb_frame: d435i_color_optical_frame
depth_frame: d435i_color_optical_frame
sentinel_timeout_s: 30.0
quality_window_s: 5.0
min_rate_hz: 5.0
max_stamp_age_s: 0.50
max_future_skew_s: 0.05
max_rgb_depth_skew_s: 0.05
```

`source_mode` must be exactly `external`. RGB and depth must remain distinct
absolute topics. Since the depth stream is required to be aligned to color,
the two configured optical frames must be identical.

Activation observes all three streams concurrently. It rejects missing or
stale data, non-monotonic source timestamps, unexpected frames or encodings,
malformed image layout, all-zero depth, invalid pinhole intrinsics,
RGB/depth/intrinsics geometry disagreement, insufficient sustained rate, and
RGB-depth timestamp mismatch. It declares Atlas endpoints only after the whole
window passes.

The observer uses Humble's low-level `_rclpy` node, subscriptions, and wait set.
It does not use the high-level `rclpy.node.Node`, which would silently create a
`/parameter_events` publisher.

## Build and start

The package build only generates Robonix contract bindings. It does not build
or install a RealSense driver.

```bash
bash build.sh
bash start.sh
```

Starting the package without live external topics fails closed. Do not add this
package to the root manifest until the Orin driver and exact topics have passed
read-only validation.

## Mapping boundary

RGB-D data must not be connected to RTAB-Map, Scene depth projection, or Nav2
until the physical D435i mounting transform is measured and represented in the
reviewed robot description. This package does not invent or publish that
transform.

## Offline tests

```bash
bash tests/run_offline_tests.sh
```

The offline suite uses synthetic messages and a fake Robonix API. It opens no
ROS graph, USB device, network interface, or robot connection.
