---
description: Goal-based navigation using a deploy-configured ROS 2 Nav2 stack.
---

# Navigation capability

Provides:

- `robonix/service/navigation/driver`
- `robonix/service/navigation/navigate`
- `robonix/service/navigation/navigate/status`
- `robonix/service/navigation/navigate/cancel`
- `robonix/service/navigation/adjust_speed`
- `robonix/service/navigation/set_speed_limit`
- `robonix/service/navigation/get_speed_limit`

Consumes Atlas-selected providers for:

- `robonix/service/map/occupancy_grid`
- `robonix/primitive/chassis/odom`
- either `robonix/primitive/lidar/lidar` or
  `robonix/primitive/lidar/lidar3d`

The robot deployment must provide a complete Nav2 YAML through `params_file`.
Relative paths use the directory containing `robonix_manifest.yaml`. A 3D lidar
also requires explicit `scan_projection` config. Optional `bt_xml_file` selects
a deploy-owned BehaviorTree. See `config.spec` for the complete instance config.

The speed capabilities change Nav2's live controller limit without restarting
the service or replacing the active goal. `adjust_speed` handles relative and
normal commands, `set_speed_limit` sets an explicit bounded percentage, and
`get_speed_limit` is read-only. Goal-scoped changes restore automatically;
persistent changes intentionally survive across runs. The deploy config
defines the maximum planar linear speed in m/s and the normal percentage.
Angular constraints remain part of the deploy-owned Nav2 controller config.

The service does not own mapping, robot TF, body dimensions, or
robot-specific planner/controller tuning.
