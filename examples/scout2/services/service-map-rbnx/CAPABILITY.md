---
description: SLAM mapping — turns robot sensor streams into a live 2D occupancy grid, 3D point cloud, and SLAM-corrected pose; persists named maps for re-localization.
---

# mapping_rbnx — capability surface

SLAM mapping service. Turns wheeled/legged-robot sensor streams into a live
2D occupancy grid, a 3D point cloud, and a SLAM-corrected pose, all under a
fixed, algorithm-agnostic contract surface. Persists named maps so a robot
can re-localize against a previously-built map across restarts.

The active SLAM engine is config-selectable (`algo`); consumers never see
which engine runs — they only bind the `robonix/service/map/*` contracts.

## Provides (declared on atlas)

| contract | transport | payload | notes |
|---|---|---|---|
| `robonix/service/map/driver` | grpc | Driver lifecycle | INIT/SHUTDOWN |
| `robonix/service/map/occupancy_grid` | ros2 topic | `nav_msgs/OccupancyGrid` | 2D grid (lidar + depth fusion) |
| `robonix/service/map/pointcloud` | ros2 topic | `sensor_msgs/PointCloud2` | 3D fused cloud |
| `robonix/service/map/pose` | ros2 topic | `geometry_msgs/PoseWithCovarianceStamped` | SLAM-corrected, map frame; can jump on loop closure |
| `robonix/service/map/odom` | ros2 topic | `nav_msgs/Odometry` | SLAM-corrected, odom frame; continuous between closures |
| `robonix/service/map/lifecycle` | ros2 topic | `map/msg/MapLifecycle` | `{map_id, mode, generation}`, latched (transient_local): current map identity for consumers keying state by map (scene). Re-published on init / load / reset / mode switch; `generation` bumps whenever the map origin may have changed (mapping-mode session start, reset) and is persisted per named map, so a localization round-trip keeps the same value. Bridge-owned + algo-independent. |
| `robonix/service/map/save_map` | grpc + mcp | `map_id, note → ok, map_id, detail` | snapshot the live map under a stable id |
| `robonix/service/map/list_maps` | grpc + mcp | `→ ok, detail, maps_json` | list saved map metadata by map id; artifacts remain opaque |
| `robonix/service/map/load_map` | grpc + mcp | `map_id, mode, [x,y,theta] → ok` | switch onto a saved map (localization / mapping) |
| `robonix/service/map/pose_estimate` | grpc + mcp | `x, y, theta → ok` | seed a pose so localization re-converges |

Every `algo` backs this **same** surface (an adapter node is spawned when an
engine can't natively publish a contract), so `scene` / `nav` are
engine-agnostic.

## Consumes (via atlas discovery)

Resolved at `Driver(CMD_INIT)` from `sensor_providers` — only bound roles are
looked up. Topic names are **never** hardcoded; they come from the selected
Atlas provider.

| config flag | contract | role |
|---|---|---|
| `lidar3d` | `robonix/primitive/lidar/lidar3d` | 3D point cloud (e.g. MID-360) |
| `lidar2d` | `robonix/primitive/lidar/lidar` | 2D LaserScan |
| `imu` | `robonix/primitive/imu/imu` | IMU (lio engines) |
| `depth` | `robonix/primitive/camera/depth` | depth image |
| `rgb` | `robonix/primitive/camera/rgb` | rgb image |
| `odom` | `robonix/primitive/chassis/odom` | wheel/external odom (else rtabmap runs its own icp_odometry) |

If a sensor isn't on atlas yet when INIT lands, the launch disables that
subscription rather than blocking.

## Config (`config:` block → `Driver(CMD_INIT, config_json)`)

```yaml
config:
  sensor_providers:
    lidar2d: front_lidar
    rgb: front_camera
    depth: front_camera
    odom: chassis
  occupancy_sources: [lidar]           # lidar | depth | both
  params_file: config/rtabmap_params.yaml
  rtabmap_params:
    Grid/FootprintLength: 0.84
    Grid/FootprintWidth: 0.60
```

`sensor_providers` is the only required sensor-selection block. Its keys enable
roles and its values pin Atlas provider IDs. Copy
[`config/rtabmap_params.template.yaml`](config/rtabmap_params.template.yaml) to
the robot deployment and reference that copy with `params_file`. The upstream
template is never loaded at runtime. Inline `rtabmap_params` applies last.

### Sensor provider roles

Each key maps to an Atlas contract. Include both `rgb` and `depth` for RGB-D.

| key       | atlas contract                      | feeds rtabmap   |
| --------- | ----------------------------------- | --------------- |
| `lidar2d` | `robonix/primitive/lidar/lidar`     | `scan` (2D)     |
| `lidar3d` | `robonix/primitive/lidar/lidar3d`   | `scan_cloud`    |
| `rgb`     | `robonix/primitive/camera/rgb`      | `rgb` image     |
| `depth`   | `robonix/primitive/camera/depth`    | `depth` image   |
| `imu`     | `robonix/primitive/imu/imu`         | `imu`           |
| `odom`    | `robonix/primitive/chassis/odom`    | `odom`          |

The legacy `sensors` boolean block remains accepted for migration, but new
deployments should use only `sensor_providers`.

`occupancy_sources` is a separate policy. It accepts `lidar`, `depth`, or both,
and fails initialization if Atlas did not resolve a requested source. Omitting
it preserves the legacy automatic choice for existing deployments.

`deskew_lidar: true` requires a per-point `t`, `time`, `stamps`, or `timestamp`
field. External odom drives TF deskewing; without external odom the ICP node
uses its motion estimate.

## Map operations (`save_map` / `load_map` / `pose_estimate`)

Runtime RPC+MCP controls for managing maps without re-deploying. All three are
callable any time after init, by gRPC (scene / programmatic) or MCP (pilot /
LLM). Maps live under `{MAPPING_MAPS_DIR}/<map_id>/` (rtabmap.db + occupancy
pgm/png + cloud pcd + meta), one directory per `map_id` — the same id scene
keys its semantic objects to.

- **save_map** `(map_id, note)` → `(ok, map_id, detail)`. Roam to build
  coverage, then checkpoint the live spatial artifact under `map_id`. The
  provider may use RTAB-Map internally, but callers treat the artifact as opaque.
  Non-destructive — mapping continues.
- **load_map** `(map_id, mode, [x,y,theta])` → `(ok, detail)`. Switch onto a
  saved map in localization mode through a private runtime copy, keeping the
  published artifact immutable and its map frame stable across runs. Pass
  `has_initial_pose` + `x,y,theta` to seed convergence.
- **pose_estimate** `(x, y, theta)` → `(ok, detail)`. Publish a pose guess
  (map frame) to `/initialpose` so rtabmap's localization re-converges — global
  relocalization, kidnapped-robot recovery, or refining a rough operator guess.
  rtabmap snaps the guess to the true pose via scan matching.

## Map persistence (`map_id`)

A named map is stored under `{MAPPING_MAPS_DIR}/{map_id}/`:

```
maps/lab_3f/
  rtabmap.db       SLAM graph (relocalization + continued mapping)
  occupancy.pgm    nav2 map_server image (0=occ, 254=free, 205=unknown)
  occupancy.yaml   nav2 map_server metadata
  occupancy.png    same image, double-click to preview offline
  cloud.pcd        fused 3D cloud
  meta.yaml        map_id, saved_at, frame_id, resolution, size, origin
```

- **`map_mode: mapping`** always builds a fresh private runtime database.
  Call `save_map(map_id)` to atomically publish an immutable spatial artifact.
- **`map_mode: localization`** requires `map_id`; mapping copies the saved db
  to a private runtime DB, then re-localizes against that copy. The saved
  artifact and its map-frame origin remain unchanged across restarts.
- Omit `map_id` for normal fresh mapping. Set it only when intentionally
  booting directly into localization of a previously saved map.

**Binding contract:** after `load_map(map_id)`, scene binds its semantic
objects to the same id. During fresh mapping there is no saved map identity.

## Deployment targets

Selected by the deploy entry's `manifest:` field (see the package manifests):

| target | manifest | build / start |
|---|---|---|
| x86_64 + docker | `package_manifest.yaml` (default) | `docker/Dockerfile` / docker run |
| arm64 Jetson + docker | `package_manifest.jetson-docker.yaml` | `docker/Dockerfile.jetson` / docker run |
| arm64 Jetson + native | `package_manifest.jetson-native.yaml` | host ROS2 + apt rtabmap / `start_native.sh` |

## What this does NOT do (deliberately)

- **No path planning / navigation** — that's `nav` (simple_nav / nav2_wrapper),
  which consumes this map.
- **No semantic labels** — that's `scene`; this is the spatial substrate.
- **No multi-map merge / global relocalization across maps** — one `map_id`
  per session.
- **`fastlio2` is broken (drift)** — kept reachable for repro only; use
  `rtabmap` (or `dlio` for a 3D Livox real-robot path).
