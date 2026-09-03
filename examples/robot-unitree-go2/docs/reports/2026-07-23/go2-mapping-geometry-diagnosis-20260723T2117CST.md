# Go2 mapping geometry diagnosis — 2026-07-23 21:17 CST

## Scope and current state

This report records an offline diagnosis of the corridor map that appeared as
radial arcs instead of parallel walls. The robot and router may remain powered
off while the changes below are reviewed and built. No motion command was
published, and this report does not claim a physical mapping acceptance.

Saved pre-fix evidence:

- lab database:
  `rbnx-build/data/maps/go2_lab_20260723_mapping01/rtabmap.db`
- corridor database:
  `rbnx-build/data/maps/go2_corridor_20260723_mapping01/rtabmap.db`
- raw-versus-native geometry plot:
  `logs/go2-readonly/lidar-geometry-20260723/raw-vs-cloud-base-topdown.png`

## Root cause

The robot publishes same-stamp `/utlidar/cloud` (`utlidar_lidar`) and
`/utlidar/cloud_base` (`base_link`) samples. Rigidly matching 22,801
corresponding points recovered the robot-native transform with a residual of:

- median: 0.000085 mm
- p99: 0.000751 mm
- maximum: 0.00192 mm

The recovered `base_link -> utlidar_lidar` transform is:

```text
xyz = 0.282160001 0 0
rpy = -2.92072011 -0.14132399 -1.01052989
```

The previous URDF composed the upstream simulation `radar` pose with a
zero-offset topic-frame alias. Its rotation differed from the native transform
by about 123 degrees. The resulting floor plane was tilted by about 25.8
degrees, and only about 0.19% of points survived the configured obstacle height
band, compared with about 7.67% for the robot-native `cloud_base`. RTAB-Map
therefore accumulated mostly free-space rays and produced the radial pattern.

A second robot-specific issue was the occupancy height split. Go2's
`base_link` is body-centred, while the corridor floor in the native base frame
is around `z=-0.39 m`. The prior `Grid/MaxGroundHeight=0.08` assumed a
floor-centred base frame and discarded useful wall/obstacle geometry.

The `/utlidar/*` topics, firmware behavior and rates identify the sensor as a
Unitree UniLiDAR with high confidence, not a Livox MID-360. The exact L1/L2
revision still requires reading the physical label. Unitree's L1 and L2 SDKs
publish the same LiDAR-to-IMU translation, so the previously copied MID-360 IMU
offset was also corrected.

## Implemented changes

- preserve the upstream `base -> radar` joint and add the measured
  `radar -> utlidar_lidar` alias so the composite exactly matches the
  robot-native base geometry;
- use Unitree's common L1/L2 LiDAR-to-IMU transform:
  `xyz=-0.007698 -0.014655 0.00667`, `rpy=0 0 0`;
- use deterministic height segmentation for the sparse non-repetitive scan;
- set `Grid/MaxGroundHeight=-0.25`, matching Nav2's existing lower scan
  projection bound;
- add numerical regression tests for the composite rotation/translation and
  mapping-height contract.

## Official Mapping comparison

The deployed Mapping source is not a clean checkout of the current official
main branch:

- local Go2 branch: `a08f4d991291cafe11178942d32e104ef5e5f197`
- local branch base: `ce4092a1bee8847d6314af957f0225c8371d9aa6`
- official `syswonder/service-map-rbnx` main observed on 2026-07-23:
  `8b00faa8dbc7fb0bce540410604763ded54f83c4`

The local branch contains nine Go2-required CycloneDDS, QoS, parameter,
operator-UI and build-isolation patches. It must not be replaced wholesale.
Official later changes, including canonical TF odometry and split-odometry
support, should be integrated semantically after the corrected single-LiDAR
map is accepted.

Lite3 and Ranger deployments use the same `service-map-rbnx + RTAB-Map` core.
Their useful pattern is a robot-owned frame tree, odometry authority and
robot-specific occupancy parameters. Their numerical mounts, height bands and
footprints are not transferable to Go2.

## Camera and D435i staging

For the first corrected corridor recapture, keep occupancy generation
LiDAR-only. The Go2 RGB camera has no valid metric calibration in the current
profile. The D435i stream and depth validity have passed read-only preview
checks, but its six-degree-of-freedom mount transform and cross-machine
acquisition timing have not been calibrated.

After LiDAR-only wall geometry and scale pass:

1. measure `base_link -> d435i_color_optical_frame`;
2. validate RGB, aligned depth, CameraInfo, QoS and moving timestamp offset;
3. add RGB-D for texture and visual loop closure while keeping
   `occupancy_sources: [lidar]`;
4. only after replay and no-motion validation, evaluate depth as a secondary
   near-field occupancy source.

## Offline verification

- `go2_description` generated and built successfully;
- description tests: 10/10 passed;
- navigation configuration tests: 9/9 passed;
- complete offline validation: 562 tests passed, 5 intentionally skipped;
- shell, URDF, YAML, static safety, D435i, sensor, dashboard, voice and
  navigation contracts all passed.

## Next powered-on acceptance

After the robot and router restart, writer identities and time evidence are
new by definition. Perform a fresh read-only identity/time capture, launch the
motion-disabled stack, and verify:

1. corrected point-cloud top view contains the two corridor wall lines;
2. floor normal is near horizontal and obstacle fraction is no longer
   free-space dominated;
3. a fresh empty map gains occupied wall cells while stationary;
4. only then perform a short, specifically approved, low-speed manual corridor
   recapture with Robonix motion still disabled.
