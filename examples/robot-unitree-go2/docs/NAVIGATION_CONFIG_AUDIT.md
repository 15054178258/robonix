# Mapping and Navigation configuration audit

This deployment profile was checked offline against the local source snapshots
of `service-navigation-rbnx` at
`b1a923a25cb3bf75554b861fceb605a190ae641b` and `service-map-rbnx` at
`ce4092a1bee8847d6314af957f0225c8371d9aa6`. The audit runs no ROS node and
sends no robot command.

## Resolved data contracts

| Role | Robonix contract | Pinned provider | Runtime topic/result |
|---|---|---|---|
| map | `robonix/service/map/occupancy_grid` | `mapping` | materializes `__ROBONIX_MAP_TOPIC__` |
| odom | `robonix/primitive/chassis/odom` | `go2_chassis` | materializes `__ROBONIX_ODOM_TOPIC__` |
| lidar3d | `robonix/primitive/lidar/lidar3d` | `go2_sensors` | `/scanner/cloud` |
| scan | `robonix/primitive/lidar/lidar` | `go2_sensors` only | absent natively, so the navigation wrapper projects the pinned cloud to `/scanner/scan` |

Pinning the optional `scan` role is deliberate. If another robot publishes a
LaserScan in the same Atlas, it cannot silently replace this Go2's lidar.

## Nav2 Humble profile

- `default_nav_to_pose_bt_xml` selects the deploy-owned tree on Humble; the
  legacy key selects the same tree on older Nav2 versions.
- The tree replans, follows the path, clears costmaps once, and waits. It has no
  Spin, BackUp, DriveOnHeading or AssistedTeleop recovery.
- NavFn refuses unknown space. Both costmaps mark and clear from the generated
  LaserScan; the global costmap also consumes the saved/live occupancy grid.
- Empty projected rays are `+Inf`. `inf_is_valid: true` lets those rays clear a
  dynamic obstacle after it moves away. A 0.5 s expected scan period makes a
  stale obstacle layer non-current instead of silently navigating blind.
- The body-centred `base_link` projection slice extends below zero to retain
  low obstacles. Its LaserScan is then at z=0, so both costmap observation
  sources use `min_obstacle_height: 0.0`; a positive minimum would discard the
  entire scan. The self-filter margin is only 0.02 m because Soma's conservative
  circumscribed footprint already defines the radial exclusion zone.
- `velocity_smoother` has explicit Go2-aligned velocity/acceleration bounds and
  a 0.25 s input timeout. The navigation service rewrites its output to
  `/cmd_vel_guard_input`; its final velocity guard is the only navigation
  publisher to canonical `/cmd_vel`. The chassis command watchdog is the next
  independent stop boundary.
- Goal tolerance is 0.20 m / 0.25 rad. The terminal rotation guard begins no
  farther than the same 0.20 m window and cancels/latches on timeout or no
  progress.

## RTAB-Map profile

Mapping consumes the provider-pinned PointCloud2 and chassis odometry, owns
`map -> odom`, and is constrained to planar motion. The chassis remains the
sole `odom -> base_link` authority. The occupancy grid is lidar-only; RGB is
display-only and no depth stream is invented.

In the audited Mapping revision, selecting external chassis odometry does not
create a second direct IMU subscription inside the map service. The manifest
still requires the pinned IMU capability as a commissioning/readiness gate;
orientation already reaches Mapping through the validated
SportModeState-derived chassis odometry. Direct RTAB-Map IMU fusion remains a
future upstream enhancement and is not claimed by this profile.

The current Mapping launch consumes a separate IMU topic only when it owns ICP
odometry. This deployment deliberately selects external chassis odometry to
avoid a second `odom -> base_link` authority, so the `imu` provider binding is
an availability/diagnostic gate rather than a second RTAB-Map fusion input.
SportModeState orientation already contributes to the chassis odometry path.
Adding explicit IMU fusion later requires a reviewed localization/odometry
design; changing this YAML alone would create conflicting TF ownership.

Both Mapping deskew and Navigation projection deskew are disabled by default.
The upstream services require a usable per-point timestamp field, but the
public Unitree interface does not prove that this exact Go2 firmware publishes
one. Enabling deskew requires a read-only message-field/timestamp audit and a
bag-replay test first. Header timestamps alone are not sufficient evidence.

## Fail-closed commissioning checks still required

Before any physical navigation, verify while stationary that:

1. the saved `GO2_MAP_ID` exists and Mapping starts in localization mode;
2. only one publisher owns each of `map -> odom` and `odom -> base_link`;
3. `/scanner/scan` is fresh, framed in `base_link`, clears a moved test object,
   retains low obstacles without flooding on floor returns, and makes both
   costmaps non-current when replay/input is stopped;
4. stopping scan, odometry, Nav2, the velocity guard, or the adapter produces
   zero output and a stopped chassis in the supervised test harness;
5. footprint and sensor transforms match measured hardware.

Offline tests enforce the static configuration invariants, but they do not
claim DDS, localization, obstacle-detection or physical stopping success.
