# Jetson D435i ephemeral read-only bridge

This directory contains a deliberately small first-stage camera bridge for the
Go2's Jetson Orin and a ROS 2 Humble workstation. The workstation streams one
Python source file over SSH stdin directly into `python3 -`; the source is not
copied to an Orin path and bytecode generation is disabled.

The Orin process opens only the RealSense, aligns depth to color, compresses
color as JPEG and depth losslessly with zlib, then writes bounded checksummed
packets to SSH stdout. It does not import ROS. The workstation validates and
decodes each packet and owns the three ROS 2 publishers. This isolates the
camera path from the Orin's incompatible Foxy/CycloneDDS library mix while
preserving standard ROS `sensor_msgs` topics for Robonix.

This is sensor streaming only. It does not publish or call any chassis,
posture, navigation, velocity or Unitree API.

## Fixed first-version surface

- D435i color and depth are captured at `640x480 @ 30 Hz`.
- One frame in every six is selected, limiting the preview to `5 Hz`. This
  stays above the workstation preview provider's 3 Hz sustained-rate gate.
- Before compression and SSH transport, the Orin downsamples the selected
  aligned frame set to 320x240: RGB uses area interpolation, depth uses
  nearest-neighbour sampling, and CameraInfo intrinsics are scaled by exactly
  0.5. This preserves the verified 640x480 hardware profile while reducing
  Wi-Fi load; it is not a navigation-resolution claim.
- Depth is aligned to RGB in `pyrealsense2`.
- A single workstation ROS receipt stamp is shared by RGB, aligned depth and
  `CameraInfo` for every validated frame set.
- Every publisher uses `KEEP_LAST(1)`, best effort and volatile QoS.
- Raw aligned depth is `16UC1`; startup fails unless the device reports the
  standard millimetre scale of `0.001 m/unit`.
- The transport has fixed magic/version fields, separate length ceilings and
  CRC32 checks for metadata, JPEG and lossless depth. Decompression has an
  exact 614400-byte ceiling.
- Frame conversion validates dimensions, dtype, continuity, payload size and
  sequence/source monotonic order. Both processes have a 1 GiB peak-RSS fuse.
- The bridge verifies that the selected RealSense product is a D435i.
- No IMU, TF, extrinsics or point cloud is published in this version.

Default topics:

| Topic | Type | Frame |
| --- | --- | --- |
| `/go2/d435i/color/image_raw` | `sensor_msgs/Image` (`bgr8`) | `d435i_color_optical_frame` |
| `/go2/d435i/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (`16UC1`) | `d435i_color_optical_frame` |
| `/go2/d435i/color/camera_info` | `sensor_msgs/CameraInfo` | `d435i_color_optical_frame` |

The common frame ID describes the alignment geometry only. This bridge does
not publish TF. A measured and independently reviewed
`base_link -> d435i_color_optical_frame` transform is still required before
Mapping, metric Scene output or navigation may consume the camera. Until then,
only the RGB/depth preview is considered valid.

## Orin prerequisites

The launcher does not install or change anything. Before an approved streaming
run, the Orin must already provide:

- importable `pyrealsense2`, OpenCV and NumPy;
- a D435i visible through a USB 3 data path;
- the intended Go2 private link on Orin `eth0`.

The workstation must already provide ROS 2 Humble, `sensor_msgs`, OpenCV and
NumPy.

If a prerequisite is absent, the bridge exits without attempting to install,
patch or reconfigure the Orin. No network settings are changed.

## Exact read-only acknowledgement and launch

The physical operator must explicitly approve camera sensor streaming with the
exact value below. Similar text, extra whitespace and different case are
rejected both by the workstation launcher and by the streamed bridge:

```bash
export D435I_READONLY_STREAMING_ACK=I_ACKNOWLEDGE_D435I_READONLY_SENSOR_STREAMING_ONLY
```

Then, from the repository root:

```bash
D435I_SSH_TARGET=unitree@192.168.123.18 \
  bash deploy/jetson-d435i-readonly/run-via-ssh.sh
```

The default SSH target is already `unitree@192.168.123.18`, so the override may
be omitted. Authentication remains with the operator/SSH agent; this directory
does not contain a password or key.

For an interactive hardware session, the launcher can reuse a previously
authenticated OpenSSH control socket by setting `D435I_SSH_CONTROL_PATH`.
The launcher accepts such a socket only beneath the repository's ignored
`.run/` directory. The socket contains no saved password and does not alter
the Orin or workstation network configuration.

The remote command is fixed and runs only `python3 - --remote-producer`; it
does not source Foxy or load an RMW implementation. The local side sources
Humble in the launcher and runs `--workstation-publisher`. It does not change
NetworkManager, addresses, routes, DNS or any persistent network setting.
The SSH launcher keeps the D435i at its verified `640x480 @ 30 Hz` hardware
profile, then uses the dashboard's native `320x240` preview size at 5 Hz on the
Orin-to-workstation transport. This avoids transporting a full-resolution
frame that the workstation would immediately downsample, while retaining a
live RGB/aligned-depth view.
`Ctrl-C`, `SIGTERM`, SSH hangup and runtime failures stop the pipeline; the
Orin path stops the RealSense pipeline and the workstation path destroys its
three publishers and ROS node.

## Offline validation

These tests import only standard-library code; they do not import ROS,
open a camera, connect over SSH or start hardware:

```bash
python3 -m unittest discover \
  -s deploy/jetson-d435i-readonly \
  -p 'test_*.py' -v
```

The tests cover the exact acknowledgement, bounded stream plan, 30-to-10 Hz
decimator, namespace and depth-scale validation, packet lengths and CRCs,
truncation, lossless depth limits, three-topic surface, QoS, shared local ROS
stamp, forbidden motion/IMU/TF/point-cloud surfaces, stdin-only SSH execution
and cleanup structure.
