# service-navigation-rbnx provider instance configuration
#
# This file documents the YAML object passed through a robot deployment entry.
# It is a human- and model-readable contract, not a separately parsed schema.

required:
  params_file:
    type: path
    path_base: directory containing robonix_manifest.yaml
    description: Complete deploy-owned Nav2 parameter YAML.
    example: config/nav2_params.yaml

  provider_ids:
    type: mapping[string, string]
    description: Atlas provider IDs for Navigation inputs.
    supported_roles:
      map: robonix/service/map/occupancy_grid
      odom: robonix/primitive/chassis/odom
      scan: robonix/primitive/lidar/lidar
      scan_cloud: robonix/primitive/lidar/lidar3d

  dynamic_speed:
    type: mapping
    description: >-
      Deployment-owned policy for runtime navigation speed changes. Physical
      quantities use SI units. The provider sends percentage limits through
      nav2_msgs/SpeedLimit to Nav2's Controller Server and independently
      enforces the resulting planar linear-speed limit at the final velocity
      guard. Controller-specific angular behavior remains owned by the Nav2
      parameter file rather than this Robonix interface.
    fields:
      max_linear_speed_mps:
        type: number
        required: true
        unit: metres per second (m/s)
        constraints: finite and greater than 0
        nav2_equivalent: >-
          The effective planar ceiling formed by the selected controller's
          max_speed_xy and per-axis max_vel_x/max_vel_y parameters.
        description: >-
          Deployment hard ceiling for planar linear speed
          sqrt(vx^2 + vy^2). Set it to the actual maximum allowed by all
          selected-controller constraints, not merely max_speed_xy when a
          per-axis limit is stricter. For a non-holonomic DWB configuration
          with max_vel_y=0, max_vel_x=0.3, and max_speed_xy=0.3, use 0.3.
          The final guard never publishes a planar command above this value,
          including commands emitted by Nav2 recovery behaviors.
        example: 0.3
      default_percentage:
        type: number
        required: true
        unit: percent (%)
        constraints: min_percentage <= value <= 100
        description: >-
          Percentage of max_linear_speed_mps applied at provider startup and
          by a normal command. It is also the initial session limit; a
          goal-scoped adjustment restores the current session limit when the
          goal terminates. For example, max_linear_speed_mps=0.3 with
          default_percentage=75 produces a planar limit of 0.225 m/s.
        example: 75
      step_percentage:
        type: number
        default: 20
        unit: percentage points
        constraints: finite and in (0, 100]
        description: >-
          Additive change made by faster or slower. For example, a 20-point
          step changes 75% to 95%, not to 90%.
      min_percentage:
        type: number
        default: 20
        unit: percent (%)
        constraints: finite and in (0, default_percentage]
        description: >-
          Lowest percentage accepted by slower or set_speed_limit. This is not
          a stop command; cancel or the chassis stop capability must be used to
          stop motion.
      topic:
        type: string
        default: /speed_limit
        nav2_equivalent: controller_server.ros__parameters.speed_limit_topic
        description: >-
          Fully-qualified ROS topic carrying nav2_msgs/SpeedLimit. It must
          match Controller Server's speed_limit_topic. Initialization fails if
          Nav2 does not subscribe.

optional:
  bt_xml_file:
    type: path
    path_base: directory containing robonix_manifest.yaml
    description: >-
      Custom BehaviorTree XML used when params_file contains the
      __ROBONIX_BT_XML__ token.

  use_sim_time:
    type: boolean
    default: false
    description: >-
      Use ROS /clock for Nav2, the wrapper, and action timing. Enable this only
      when the complete simulator TF and sensor graph uses simulated time.
  action_wait_s:
    type: number
    default: 45.0
    unit: seconds
    description: >-
      Maximum time CMD_INIT waits for Nav2's navigate_to_pose action server to
      become ready. Expiry fails initialization and tears down the spawned
      Nav2 and guard processes. Must be greater than zero.

  scan_projection:
    type: mapping
    description: >-
      Explicit PointCloud2-to-LaserScan adapter. Omit for a native LaserScan.
    fields:
      enabled:
        type: boolean
        default: false
        description: Enable the PointCloud2-to-LaserScan adapter.
      target_frame:
        type: string
        default: Soma base_frame
        description: >-
          TF frame in which height, range, and robot self-filtering are
          evaluated. An empty value uses Soma's declared base frame.
      min_height_m:
        type: number
        default: 0.0
        unit: metres
        description: Minimum point height retained in target_frame.
      max_height_m:
        type: number
        default: 2.0
        unit: metres
        description: >-
          Maximum point height retained in target_frame. It must be greater
          than min_height_m.
      range_max_m:
        type: number
        default: 30.0
        unit: metres
        description: Maximum projected scan range. Must be non-negative.
      self_filter_margin_m:
        type: number
        default: 0.05
        unit: metres
        description: >-
          Extra margin added around Soma's robot footprint before points are
          treated as returns from the robot itself. Must be non-negative.
      transform_tolerance_s:
        type: number
        default: 0.15
        unit: seconds
        description: Allowed TF timestamp tolerance during cloud projection.
      deskewing:
        type: boolean
        default: false
        description: >-
          Correct motion distortion before projection. Enable only when the
          PointCloud2 contains usable per-point timestamps and odometry/TF is
          available for the scan interval.
      deskew_fixed_frame:
        type: string
        default: odom
        description: Fixed TF frame used to compensate motion during deskewing.
      deskew_wait_for_transform_s:
        type: number
        default: 0.2
        unit: seconds
        description: >-
          Maximum wait for the transforms required by deskewing. Must be
          non-negative.

  topic_remap:
    type: mapping[string, string]
    description: Advanced direct ROS topic override; provider_ids is preferred.
  trajectory_log_dir:
    type: path
    default: rbnx-build/data/trajectories
    description: >-
      Directory for per-goal JSONL trajectories and scan anomaly records.
      Use an absolute path when traces must live outside the package build
      directory.
  velocity_output_topic:
    type: string
    default: /cmd_vel
    env: ROBONIX_VELOCITY_OUTPUT_TOPIC
    description: >-
      Fully-qualified ROS topic on which the final velocity guard publishes.
      Set this to /robonix/nomotion/cmd_vel for motion-disabled integration.
      The deployment config takes priority over the environment. Empty,
      relative, private, substituted, or malformed topic names fail startup.
  guard_terminal_xy_m:
    type: number
    default: 0.45
    unit: metres
    description: >-
      Distance from the current global-plan endpoint at which stationary
      rotation is treated as terminal alignment and receives stricter limits.
      Must be non-negative and should not exceed the goal checker's XY window.
  guard_terminal_timeout_s:
    type: number
    default: 15.0
    unit: seconds
    description: >-
      Maximum continuous terminal-alignment rotation time before the guard
      publishes zero velocity and cancels the active navigation goal.
  guard_no_progress_s:
    type: number
    default: 3.0
    unit: seconds
    description: >-
      Maximum terminal rotation interval without a meaningful reduction in
      yaw error before the goal is stopped.
  guard_global_spin_timeout_s:
    type: number
    default: 25.0
    unit: seconds
    description: >-
      Maximum continuous stationary rotation time anywhere on the route,
      including planner/controller recovery loops.
  guard_global_spin_limit_rad:
    type: number
    default: 6.783185307
    unit: radians
    description: >-
      Maximum cumulative odometry rotation during one continuous stationary
      rotation episode before the guard stops and cancels the goal. The
      default is one full revolution plus 0.5 radian.

deprecated_compatibility:
  params_profile:
    type: string
    replacement: params_file
    behavior: Known legacy profiles still load and emit a migration warning.
    description: >-
      Deprecated selector for provider-packaged parameter files. Retained so
      existing deployments start while they migrate configuration into their
      robot repository.
  scan_deskewing:
    type: boolean
    replacement: scan_projection.deskewing
    description: Deprecated flat alias retained for existing manifests.
  scan_self_filter_margin_m:
    type: number
    replacement: scan_projection.self_filter_margin_m
    description: Deprecated flat alias retained for existing manifests.
  odom_frame:
    type: string
    replacement: scan_projection.deskew_fixed_frame
    description: Deprecated flat alias retained for existing manifests.
