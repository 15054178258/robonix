---
description: Fail-closed Unitree Go2 chassis topics for guarded Nav2 control.
---

# Unitree Go2 chassis capability

This package provides the standard Robonix chassis contracts needed by Nav2:

- `robonix/primitive/chassis/twist_in` maps to `/cmd_vel`.
- `robonix/primitive/chassis/odom` maps to `/odom`.
- `robonix/primitive/chassis/driver` manages registration only.

It intentionally does not expose posture, stand, sit, low-level control, or a
single-shot movement RPC. Nav2 is expected to be the only velocity controller.

## Safety state machine

The committed configuration starts as `DISABLED`. A motion-capable deployment
requires all of the following independent gates:

1. Driver config contains `allow_motion=true`, `operator_present=true`, the
   exact safety acknowledgement, and an explicit network interface.
2. `GO2_ALLOWED_MODES` explicitly lists modes measured during read-only audit;
   the public/default list is the impossible sentinel `[255]`.
3. `SportModeState.error_code` is zero, or its exact current non-zero value was
   explicitly supplied in `GO2_ALLOWED_STATE_MARKERS` as an opaque firmware
   compatibility marker. The default marker allowlist is empty. A marker
   change always latches closed while motion is enabled and requires explicit
   disarm then re-arm.
4. The SDK daemon receives its internal `--allow-motion` gate only after the
   provider validates the preceding inputs.
5. The ROS adapter receives `allow_motion:=true`.
6. A fresh, numerically valid `SportModeState` has an allowed mode, an eligible
   stable marker, a non-zero source timestamp, and a source timestamp that
   strictly advances on every accepted state while motion is enabled.
7. If `odom_source=external_verified`, its private time-corrected Odometry
   input is distinct from `/odom`, fresh, strictly advancing, finite,
   frame-correct, quaternion-valid, and free of bounded pose jumps. The chassis
   adapter remains the only `/odom` and `odom -> base_link` publisher.
8. A fresh zero `/cmd_vel` is held for at least 0.5 seconds.
9. An operator explicitly calls the SetBool arm service with `data=true`.

The two commissioning profiles remain one-shot. The standard staged-Nav2
profile may be explicitly disarmed and re-armed in the same long-running
process, but every arm still repeats fresh state/odometry and zero-preamble
checks. Shutdown, stale input, disconnect and command timeout still stop and
disarm the daemon.

Per-instance topics, limits, and motion acknowledgements are delivered through
`Driver(CMD_INIT)`. `GO2_ALLOWED_MODES` and the exceptional, default-empty
`GO2_ALLOWED_STATE_MARKERS` are separate explicit process environment inputs
populated only from the read-only robot audit. Process
startup itself has no per-instance config and therefore starts only the Python
provider. Neither configuration nor daemon startup performs the arm action.
The provider must itself start with CycloneDDS already bound to the approved
Go2 NIC through `CYCLONEDDS_URI`; this cannot be deferred to `CMD_INIT`
because the provider's ROS context is created before per-instance config
arrives.

A dedicated manual-mapping manifest may set
`allow_passive_state_marker_transitions=true` only with
`allow_motion=false`, `odom_source=external_verified`, and at least two
explicit reviewed markers. In that one passive profile, transitions wholly
inside the set keep the independent external odometry chain available. An
unlisted transition pauses canonical output; it never becomes allowed, and
output resumes only after one listed marker remains continuous for `0.50 s`,
at least five samples, and no sample gap over `0.20 s`. The runtime rejects
this option for every motion-capable or `sport_state`-odometry configuration.

State or command staleness, a zero/repeated/regressing motion-state timestamp,
invalid numbers, disallowed mode, an unconfigured or changed opaque firmware
state marker, stale/regressing/wrong-frame external odometry, odometry jumps,
IPC failure, SDK failure, controller disconnect, shutdown, and daemon watchdog
expiry all fail closed. A repeated or regressing timestamp cannot refresh the
state watchdog. Motion-capable processes latch state/odometry liveness loss in
the ROS process until a reviewed restart. A strictly passive
`allow_motion=false` external-odometry process may suppress canonical outputs
and reacquire after a pure receipt timeout or source-age rejection; it never
resets source timestamp history. Frame, numeric, marker, timestamp-integrity,
and within-epoch pose-continuity faults remain process-lifetime latches in
every profile.
Read-only mode continues to accept zero or repeated source timestamps for
firmware compatibility while the robot is stationary.

The initial limits are 0 to 0.25 m/s forward (reverse is rejected), no lateral
motion, 0.40 rad/s yaw,
0.30 m/s² linear acceleration, and 0.80 rad/s² angular acceleration. These are
defense-in-depth limits in both processes, not a claim that physical motion has
been validated.
