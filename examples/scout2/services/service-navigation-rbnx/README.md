# service-navigation-rbnx

Robonix wrapper around a system-installed ROS 2 Nav2 stack. The service
discovers map, odometry, and lidar inputs through Atlas and exposes
`robonix/service/navigation/*` over gRPC and MCP.

## Deployment config

Navigation behavior belongs to the robot deployment, not this provider. Each
robot must carry a complete Nav2 YAML and reference it with `params_file`:

```yaml
service:
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan: lidar
      dynamic_speed:
        max_linear_speed_mps: 0.3
        default_percentage: 75
        step_percentage: 20
        min_percentage: 20
```

Relative paths are resolved from the directory containing
`robonix_manifest.yaml`. `config/nav2_params.example.yml` is a neutral example,
not a robot profile. Copy it into the deploy repository and set the robot's
frames, footprint, velocity and acceleration limits, costmap layers, goal
tolerances, topics, and planner/controller plugins there.

For a 3D lidar, bind `scan_cloud` and declare the adapter explicitly:

```yaml
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan_cloud: lidar3d
      scan_projection:
        enabled: true
        target_frame: base_link
        min_height_m: 0.1
        max_height_m: 1.5
        range_max_m: 12.0
```

Optional `bt_xml_file` points to a deploy-owned BehaviorTree XML. Existing
`params_profile` deployments remain supported and emit a migration warning;
new deployments should not use that field. See `config.spec` for every
accepted instance field and default.

The final velocity guard publishes to `/cmd_vel` by default for compatibility.
Set `config.velocity_output_topic` to a fully-qualified non-motion sink such as
`/robonix/nomotion/cmd_vel` while integrating a physical robot. The
`ROBONIX_VELOCITY_OUTPUT_TOPIC` environment variable is the fallback when the
config field is absent; an explicit empty, relative, or malformed topic fails
startup before the guard creates any ROS endpoint.

Dynamic speed config uses Nav2-compatible SI semantics.
`max_linear_speed_mps` is the actual hard planar limit
`sqrt(vx^2 + vy^2)` in m/s after considering both the selected controller's
`max_speed_xy` and stricter per-axis limits. The final velocity guard
independently enforces this linear limit. `default_percentage` scales that
maximum, so the example starts at `0.225 m/s`. `step_percentage` is an additive
number of percentage points. Angular constraints such as DWB's
`max_vel_theta` stay in the deploy-owned Nav2 YAML; a controller may
proportionally adjust its internal kinematics when it processes a Nav2 speed
limit, but Robonix does not expose a separate angular-speed policy here.

`adjust_speed` handles `faster`, `slower`, and `normal`;
`set_speed_limit` applies an explicit percentage; and `get_speed_limit` reads
the current and configured state. By default a mutation belongs to the
selected active run and automatically restores the session limit when that run
terminates. `persist=true` deliberately changes the provider-session limit
across navigation runs. None of these operations restart Navigation, cancel a
goal, or resubmit it.

## Runtime

At `Driver(CMD_INIT)`, the wrapper:

1. resolves the selected Atlas providers;
2. resolves and materializes the deployment-owned Nav2 YAML;
3. starts an optional PointCloud2-to-LaserScan adapter;
4. starts Nav2 and waits for the `navigate_to_pose` action server;
5. connects to Nav2's live `speed_limit` subscriber;
6. exposes navigate, status, cancel, and dynamic speed capabilities.

Missing required providers return `deferred`. Invalid config or a Nav2 startup
failure returns `error` and tears down every child process.

## Build and tests

Navigation generates only its Atlas MCP bindings on every deployment target.
It deliberately does not generate, build, or source a Robonix ROS 2 IDL
overlay: the provider talks to Nav2 through the ROS 2 interfaces supplied by
the selected Humble installation, while its own public capability transport is
gRPC/MCP. Jetson native builds source only the system ROS 2 installation and
the locally built terminal-controller plugin overlay.

```bash
bash scripts/build.sh
python3 -m unittest -v \
  test_configuration.py \
  test_runtime_integration.py \
  test_rotation_guard.py \
  test_scan_filter.py \
  test_speed_control.py \
  test_velocity_limit.py
```

Jetson native builds require ROS 2 Humble and Nav2 packages compatible with the
host JetPack image. Docker manifests remain available for simulator and CI
deployments.
