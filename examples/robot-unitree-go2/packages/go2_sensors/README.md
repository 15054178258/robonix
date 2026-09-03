# Go2 read-only sensor adapters

This package standardizes Go2 sensor data for the Robonix navigation stack on
ROS 2 Humble. It has no chassis-control responsibility and does not configure a
network interface. Hardware use starts only after the repository's read-only
bring-up gate has been approved.

## Data paths

```text
/utlidar/cloud (PointCloud2) ── go2_sensor_relay ──> /scanner/cloud
/imu/data (Imu)              ── go2_sensor_relay ──> /scanner/imu

Go2 camera API (JPEG)
  └─ SDK-only go2_camera_daemon
       └─ private Unix SOCK_STREAM (framed JPEG)
            └─ ROS-only go2_camera_bridge
                 ├─> /camera/color/image_raw (Image, bgr8)
                 ├─> /camera/color/camera_info (CameraInfo)
                 └─> /go2/sensors/status (DiagnosticArray)
```

The paths above show the root deployment override. The package-only default is
`/sensors/imu/data`; the provider always declares the endpoint actually
selected by its delivered config.

Before publishing either standard output, the relay compares the input ROS
header timestamp with its ROS clock. A zero timestamp, malformed timestamp,
sample older than the configured maximum, or sample farther in the future than
the configured tolerance is rejected. Rejected data never reaches
`/scanner/cloud` or the standardized IMU output. The lidar relay otherwise
copies the complete `PointCloud2` message, including its original timestamp and
`frame_id`. The IMU relay also copies the complete message;
`imu.frame_override` is empty by default and should only be set when the
installed URDF uses a verified equivalent frame name.

The checked-in navigation-oriented limits are conservative: lidar may be at
most 0.50 s old, IMU at most 0.20 s old, and either may lead the local clock by
at most 0.05 s. Configure them independently with
`lidar.max_stamp_age_seconds`, `lidar.max_future_stamp_offset_seconds`,
`imu.max_stamp_age_seconds`, and `imu.max_future_stamp_offset_seconds`. Fix a
clock-synchronization fault instead of widening these bounds to admit delayed
or replayed data. Runtime overrides may tighten these four limits but cannot
raise them above the checked-in ceilings; an attempted widening makes the
relay fail at startup.

Both ROS nodes report topic names, frame IDs, observed rates, received and
published sample counts, age, rejected frames, connection state, and
calibration state through `diagnostic_msgs/DiagnosticArray` on
`/go2/sensors/status`. Timestamp rejection counters distinguish zero, stale,
future, and malformed stamps. A rejection produces at least one `ERROR`
diagnostic interval; a continuously invalid stream remains in `ERROR` until a
fresh sample is accepted.

## Why the camera uses two processes

ROS 2 Humble and the vendor SDK can carry different CycloneDDS builds. Linking
both into one process risks an ABI collision. The isolation boundary is
therefore deliberate:

- `go2_camera_daemon` links only the official vendor SDK and calls only the
  image-sample API. It never writes JPEG files. Its installed binary uses
  `$ORIGIN/../lib` and ships the SDK's `libddsc`/`libddscxx` in that private
  sibling directory.
- `go2_camera_bridge` links ROS, OpenCV, and cv_bridge, but never links or
  imports the vendor SDK.
- The provider replaces `LD_LIBRARY_PATH` only in the camera-daemon child
  environment. The provider, lidar/IMU relay, and camera bridge retain the ROS
  environment and never receive the Unitree DDS directory.
- The daemon accepts only a same-UID peer over a mode-`0600` Unix socket inside
  a mode-`0700` directory. The bridge checks the socket owner, mode, and peer
  credentials before reading.

The protocol is fixed-width little-endian framing over `SOCK_STREAM`. Every
record carries magic, version, monotonically increasing sequence, realtime and
monotonic timestamps, bounded payload length, and cumulative API acceptance,
source rejection, API error, and IPC connection counters. Error/status records
carry no JPEG payload. The bridge rejects unknown versions, reserved bits,
oversized data, non-increasing sequences, stale timestamps, malformed complete
JPEG marker streams, libjpeg warnings/failures, and images beyond the configured
dimensions. Only a fully decoded image reaches ROS. It reconnects after a
bounded read timeout.

The bridge drains complete IPC records on a dedicated reader thread and hands
them to JPEG decode/ROS publication through a one-frame latest-value mailbox.
If decode or a downstream ROS path is slower than acquisition, a newly received
complete frame replaces the single pending frame; an older frame is also
dropped before processing if its connection has been invalidated. Once a valid
in-flight frame is being decoded it is allowed to complete, avoiding permanent
publication starvation when decode is consistently slower than acquisition;
connection generation and the original 2 s timestamp freshness are checked
again after decode. This prevents the Unix stream from acting as an
unbounded-latency FIFO while preserving the original capture timestamp.
Connection changes and shutdown invalidate pending work. The image publisher
is best-effort/volatile with `keep_last(1)`, and diagnostics expose
`superseded_count` and `pending_frame_depth` so overload remains visible without
weakening the 2 s freshness gate.

The first physical read-only camera probe was not stable enough for navigation:
it observed roughly 1.88 valid frames/s, malformed or oversized JPEG samples,
IPC reconnects, and vendor API return code `3104`. That number is retained as
an opaque diagnostic value; this repository does not infer its meaning. The
quality diagnostic therefore fails closed on a stale stream, a valid rate below
1.0 Hz, or an API/rejection ratio above 20% over the bounded 10 s window. The UI
shows the measured valid rate, rejection ratio, and gate result.

## Dependencies and build

Required ROS packages are `rclcpp`, `sensor_msgs`, `std_msgs`,
`diagnostic_msgs`, `cv_bridge`, OpenCV, libjpeg, `ament_cmake`, and `launch_ros`. The
camera daemon additionally needs a local official `unitree_sdk2` checkout.
Neither build helper installs packages or uses sudo.

Build the ROS-only package:

```bash
bash build.sh
```

The package build runs `rbnx codegen --ros2`, builds the generated canonical
message overlay at `rbnx-build/codegen/ros2_idl/install/setup.bash`, sources it,
and only then compiles the C++ sensor nodes. Runtime sources the base ROS setup,
that canonical overlay, and the local package overlay in the same order.
`scripts/build_ros.sh` is a lower-level helper and expects codegen to have
already produced `rbnx-build/codegen/ros2_idl`.

Build the SDK-only daemon in a separate CMake graph:

```bash
export UNITREE_SDK2_ROOT=/absolute/path/to/unitree_sdk2
bash scripts/build_camera_daemon.sh
```

`UNITREE_SDK2_DIR` is also accepted. If neither variable is set, the helper
looks for `third_party/unitree_sdk2` at the deployment repository root.
The helper explicitly forces `BUILD_EXAMPLES=OFF`, builds only the
`go2_camera_daemon` target, and installs the runnable private image at
`.build/camera-daemon/install/{bin,lib}`. Do not run a build-tree binary: only
the installed binary has the reviewed private-library layout.

This separation is structural: the ament `CMakeLists.txt` cannot see or link
the SDK target, and the daemon's standalone CMake project has no ROS
dependencies. The child-only loader environment plus installed RPATH prevents
the ROS 2 Humble CycloneDDS ABI and the Unitree SDK2 CycloneDDS ABI from
entering the same process.

## Read-only start sequence

Do not run these commands until the physical read-only connection checklist is
complete. The scripts do not alter IP address, route, DNS, or link settings.

Terminal 1, with the explicitly verified wired interface:

```bash
bash scripts/run_camera_daemon_readonly.sh <go2-wired-interface>
```

Terminal 2:

```bash
bash scripts/run_ros_bridges_readonly.sh
```

To omit the camera while validating lidar and IMU:

```bash
bash scripts/run_ros_bridges_readonly.sh enable_camera:=false
```

Stop both processes with `Ctrl-C`. No process is enabled automatically at boot.

When started by Robonix, `package_manifest.yaml`, `build.sh`, and `start.sh`
wrap the same binaries in provider `go2_sensors`. The wrapper waits for fresh
PointCloud2, IMU, RGB, and CameraInfo samples before declaring ROS endpoints in
Atlas. Its deployment keys match the parent `robonix_manifest.yaml`, including
`network_interface`, input/output topics, `camera_ipc_socket`, `camera_fps`,
and `sentinel_timeout_s`. RGB is a required manifest capability, so a Robonix
activation fails closed if either camera process or its fresh outputs are
missing. The standalone `enable_camera:=false` command above is only a local
ROS lidar/IMU diagnostic mode; it is not a valid Robonix package activation.

The provider has two exact `source_mode` values:

- `local` is the default and preserves the original behavior: it owns and
  starts `go2_sensor_relay`, `go2_camera_daemon`, and `go2_camera_bridge`.
- `external` starts none of those three publisher processes and does not
  require their local runtime artifacts. It waits for the standardized
  PointCloud2, IMU, Image, and CameraInfo topics from the NX, then registers
  those same topic contracts with Atlas. Missing samples fail activation.

Do not set `source_mode` directly in normal deployment. Root
`GO2_RUNTIME_PLACEMENT=workstation-full-nx-sensors` derives it as `external`
and first verifies one NX publisher for both camera topics, `/scanner/cloud`
and `/scanner/imu`, with no existing `/odom` or `/tf_static` publisher. The NX
must have been started using `--sensors-only --camera`; its default full mode
also owns odometry and TF and is intentionally rejected by that workstation
placement.

## Parameters and safe defaults

The checked-in defaults are in `config/go2_sensors.yaml`:

- lidar: `/utlidar/cloud` to `/scanner/cloud`;
- IMU: `/imu/data` to `/scanner/imu` in the root deployment
  (`/sensors/imu/data` package-only default);
- lidar stamp age/future limits: 0.50 s / 0.05 s;
- IMU stamp age/future limits: 0.20 s / 0.05 s;
- camera: `/camera/color/image_raw` plus matching `CameraInfo`;
- maximum JPEG: 4 MiB, absolute protocol ceiling 16 MiB;
- maximum decoded image: 4096 × 4096;
- IPC read timeout: 1.5 s;
- maximum camera frame age: 2 s;
- quality window: 10 s after a 5 s startup grace period;
- minimum valid camera rate: 1.0 Hz;
- maximum source/API/strict-decode rejection ratio: 20%;
- diagnostics publish period: 1 s.

Input and output topics must be absolute and must differ, preventing accidental
relay loops. The SDK daemon requires an explicit `--interface`; there is no
implicit default network device.

The default camera matrix deliberately has `K[0] == 0`, the ROS convention for
an uncalibrated camera. Record a calibration for the exact physical Go2 and
replace `d`, `k`, `r`, and `p` before using depth projection or metric visual
localization. Do not invent calibration or sensor extrinsics.

CameraInfo is published reliable/transient-local so late subscribers receive
the latest record. While the checked-in matrix is uncalibrated, this package
does not list or declare the Robonix intrinsics capability. Add that contract
only in a versioned release after measured, per-deployment intrinsics (and
verified sensor extrinsics where required) are installed and validated.

## Offline verification

The offline suite compiles no ROS code, opens no network device, and contacts
no robot. It validates protocol encoding/rejection/stream framing/timeouts, the
single-slot latest-frame overload behavior, and statically enforces the process
boundary and absence of control surfaces. If `cmake` is available, it also
configures (but does not build) the standalone camera graph and verifies that
Unitree examples remain forced off:

```bash
bash tests/run_offline_tests.sh
```

After ROS dependencies exist, a normal colcon build is still required. On
hardware, validate topic type, QoS, rate, timestamp monotonicity, `frame_id`, TF
connectivity, and the diagnostics stream before enabling mapping or
localization.
