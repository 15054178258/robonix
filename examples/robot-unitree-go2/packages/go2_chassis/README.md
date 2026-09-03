# Go2 chassis adapter

A split-ready Unitree Go2 chassis primitive for ROS 2 Humble and Robonix. The
ROS process consumes `unitree_go/msg/SportModeState`, publishes standard
odometry, IMU, TF and diagnostics, and forwards guarded velocity decisions over
a same-UID Unix `SOCK_SEQPACKET` connection. A separate SDK-only daemon owns the
Unitree `SportClient` so ROS and Unitree's bundled DDS libraries are never
loaded into one process.

The package is motion-disabled by default and has not been physically
validated merely because it builds. Do not enable its motion gates until the
read-only sensor/TF checks, remote takeover, clear test area, conservative
staged test, watchdog, cancel, and stop checks are approved at the robot.

## Components

- `ros2_ws/src/go2_chassis_adapter`: ROS-only adapter; never links SDK2.
- `sdk_daemon`: SDK2-only `Move`/`StopMove` process; no ROS dependency.
- `include/go2_chassis`: wire protocol and independently testable guards.
- `tests`: offline fake-client tests that cannot contact a robot.
- `go2_chassis/runtime_config.py`: strict `Driver(CMD_INIT)` config validation.
- `scripts`: package build/start plus motion-disabled diagnostic wrappers.

Set `UNITREE_ROS2_SETUP` to the setup file containing `unitree_go` messages and
`UNITREE_SDK2_DIR` to the official SDK2 checkout before building. The package
build uses `RBNX_PACKAGE_ROOT`, runs `rbnx codegen -p "$PKG" --ros2`, builds and
sources the generated ROS interface overlay, then builds the ROS-only adapter
and isolated SDK daemon. It does not install missing dependencies and forces
all upstream examples off.

The Robonix start command only launches the Python provider. Per-instance
topics, frames, limits, and gates arrive later through `Driver(CMD_INIT)`; the
provider owns child processes with `Primitive.spawn()`. With the public config,
only the ROS adapter starts, its motion gate is false, and its only allowed
SportModeState mode is the impossible sentinel `255`. The SDK daemon starts
only after all motion gates and an explicit read-only-audited
`GO2_ALLOWED_MODES` process value pass validation. Manual wrappers reject
motion-enabled use.

`SportModeState.error_code` is separate from `mode` in Unitree's public IDL.
Zero is the only value accepted by default. A non-zero value may reach
canonical state only when the operator explicitly lists that exact current
value in `GO2_ALLOWED_STATE_MARKERS`; it remains an opaque firmware
compatibility marker, not a claimed mode, healthy state, or RPC result. The
first eligible value is bound for the session. Any later value change blocks
canonical state and latches the motion guard, even if both values were listed.
The only exception is an explicit
`allow_passive_state_marker_transitions=true` deployment with
`allow_motion=false`, `odom_source=external_verified`, and at least two
reviewed non-zero markers; only transitions wholly inside that set remain
eligible. If that one passive profile observes any other marker, it immediately
pauses canonical odometry, TF, and IMU. It may resume only after one explicitly
reviewed marker is observed continuously for at least `0.50 s` and five
samples, with no sample gap over the `0.20 s` state-liveness ceiling.
Motion-capable and ordinary passive deployments still require an explicit
disarm/re-arm or reviewed restart, respectively.

The adapter accepts `/sportmodestate` as its primary state source and uses
`/lf/sportmodestate` only after the primary stream becomes stale. Its default
`odom_source=sport_state` preserves the isolated first-motion commissioning
path. A navigation deployment may instead select `external_verified` and feed
the private corrected `/robonix/time_corrected/raw/utlidar/robot_odom` input.
That input is never exposed directly as canonical state: the adapter requires
the configured `odom`/`base_link` frames, a fresh strictly advancing source
timestamp, finite pose/twist/covariances, a valid quaternion, and bounded
translation/yaw changes. The input and output topics must differ. Only the
adapter publishes canonical `/odom` and `odom -> base_link`; Sport state still
provides independent robot health and IMU evidence. It publishes standard
odometry, IMU, TF, status, and diagnostics locally. Runtime logs are
written below `rbnx-build/data`; the Unix socket uses a short per-package path
below `/tmp`. See `CAPABILITY.md` for every gate required before any future
physical motion test.

With `allow_motion=false`, the adapter has a strictly passive ROS graph. It
does not construct a `SOCK_SEQPACKET` client, subscribe to `/cmd_vel`, expose an
arm service, create a control timer, or attempt a daemon disarm during
shutdown. Only state subscriptions and the validated odometry, TF, IMU, status,
and diagnostics outputs exist. Motion-control entities are created together
only after `allow_motion=true` has passed the provider's independent runtime
acknowledgement gates; the existing explicit arm, state, timestamp, limit, and
watchdog gates still apply afterward.

Every accepted state message must carry a well-formed, non-zero source
timestamp within `0.20 s` behind to `0.05 s` ahead of the adapter's ROS clock.
These committed ceilings are **only-tighten** runtime policy: per-instance
configuration may select a smaller window, but cannot widen the state timeout,
source age, or future-skew limit. An unconfigured non-zero
`SportModeState.error_code`, or any marker change not covered by the narrowly
gated passive external-odometry policy above, immediately fails chassis health
and publishes no canonical odometry,
`odom -> base_link` TF, or IMU sample. An explicitly allowed stable opaque
marker is reported as a warning and does not bypass any other safety gate.
Rejected zero, stale, or future-dated samples neither refresh the motion
watchdog nor publish canonical odometry, TF, or IMU. Canonical outputs retain
the validated Unitree source timestamp; they are never restamped with receipt
time. A detected clock offset must therefore be fixed by synchronizing the
host/robot clocks, not hidden by restamping or relaxing the gate. When motion
is enabled, source timestamps must also strictly advance; duplicate or
regressing timestamps do not refresh the state watchdog. Initial linear motion
is forward-only: the adapter clamps reverse requests to zero and the SDK daemon
independently rejects them.

When `external_verified` is selected, arming additionally requires both a fresh
valid SportModeState and fresh accepted external odometry. Stale, regressing,
malformed, wrong-frame, non-finite, or jumping external odometry is rejected;
if commissioning is preparing or armed, that rejection immediately enters the
existing fail-closed stop/disarm path. The committed external freshness ceiling
is 0.20 seconds, the same only-tighten ceiling used for robot state. The
SportModeState witness remains mandatory for every canonical `/odom` and TF
publication, including passive/no-motion operation. In a motion-capable process,
any state/odometry timeout, invalid sample, timestamp fault, or continuity
violation latches external odometry closed for the lifetime of the adapter
process and recovery requires a reviewed restart. In a strictly passive
`allow_motion=false` process only, a pure receipt timeout or source-age
rejection suppresses `/odom` and TF, clears the passive pose-continuity epoch,
and waits for fresh state plus external odometry. It does not reset source
timestamp history, so replay or regression remains a permanent integrity
fault. The narrowly gated passive marker quarantine above is the only marker
exception and never accepts an unlisted value; all ordinary marker faults,
wrong frames, malformed/future/regressing timestamps, non-finite values,
invalid quaternions, and pose/yaw continuity violations remain process-lifetime
latches. Pose/yaw continuity is enforced within each established passive epoch;
only the passive liveness/marker-quarantine handlers may start a new one, and
only while motion is disabled.
