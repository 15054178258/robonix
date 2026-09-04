---
description: Guarded Unitree G1 chassis topics with no-motion default and stationary odometry heartbeat.
---

# Unitree G1 chassis capability

This package provides the standard Robonix chassis contracts used by the G1
example:

- `robonix/primitive/chassis/twist_in` maps to `/cmd_vel`.
- `robonix/primitive/chassis/odom` maps to `/odom`.
- `robonix/primitive/chassis/driver` manages provider registration and process
  lifecycle.

It intentionally does not expose posture control, stand, sit, arm control,
hand control, low-level joint control, or a single-shot movement RPC. Nav2, or
another explicitly configured velocity controller, is expected to be the only
source of velocity commands.

## Current behavior

The provider starts a ROS 2 adapter and an SDK2 daemon connected through a
same-user Unix `SOCK_SEQPACKET` socket. The adapter subscribes to the
configured velocity input topic, publishes `/odom`, publishes neutral waist
joint states, and broadcasts `odom -> base_link` so the example graph has a
stable base frame for mapping and navigation.

The current `/odom` stream is a stationary heartbeat. It is useful for keeping
the ROS graph connected while real G1 odometry is not integrated, but it must
not be treated as measured chassis motion. A physical navigation deployment
needs a reviewed odometry source before motion is enabled.

The committed Robonix startup path launches the SDK daemon without
`--allow-motion`, so SDK2 motion commands are disabled by default. The daemon
can parse a motion-capable command line only when `--allow-motion`,
`--interface`, and the exact `--motion-ack G1_PHYSICAL_MOTION_APPROVED` value
are supplied together, with the audited `300 ms` watchdog.

## Safety boundaries

Motion-disabled operation does not initialize the Unitree SDK2 client and
therefore cannot issue `SetVelocity` or `StopMove` through SDK2. It still
starts the IPC daemon and ROS adapter so the package can register its chassis
contracts and publish the stationary graph heartbeat.

When motion is explicitly enabled outside the current default startup path, the
daemon applies the following local checks:

- The socket path must be absolute, owned by the current user, and fit
  `sockaddr_un`.
- The IPC peer must have the same UID as the daemon process.
- Motion requires an explicit network interface and the exact physical-motion
  acknowledgement string.
- Motion-capable watchdog configuration must be exactly `300 ms`.
- Velocity commands are clamped by the configured `vx`, `vy`, and yaw-rate
  limits before reaching SDK2.
- Adapter disconnect, watchdog expiry, shutdown, SDK errors, and malformed IPC
  packets fail closed by stopping or refusing motion.

These checks are defense-in-depth only. They are not evidence that G1 physical
motion has been validated.

## Open integration limits

This G1 package does not currently implement the Go2 SportModeState health
chain, opaque firmware marker review, external verified odometry quarantine,
or staged Nav2 arming profile. Those Go2-specific gates must not be assumed for
G1.

Before any motion-capable G1 deployment, the provider startup path, velocity
forwarding, odometry source, operator approval flow, watchdog behavior, stop
behavior, and field test procedure need a fresh review on the target robot.
