# Repository safety rules

This repository integrates Robonix with a physical Unitree Go2. Safety takes
precedence over convenience.

1. Motion is disabled by default. Never change a public/default configuration
   to arm motion automatically.
2. Never call StandUp, StandDown, Sit, RecoveryStand, Damp, BalanceStand or any
   other posture API from boot, shutdown, tests, or health checks.
3. Never use or expose Unitree low-level motor control or `/lowcmd`.
4. Never run Unitree motion examples, including `sport_mode_ctrl`,
   `go2_sport_client`, `go2_stand_example`, and `low_level_ctrl`.
5. `/cmd_vel` may only be consumed by the guarded chassis adapter. It must not
   be published by setup, audit, test, or read-only scripts.
6. Enabling motion requires all independent gates documented in
   `docs/SAFETY.md`; starting the deployment never arms the chassis.
7. Every command path has an independent watchdog and must request StopMove on
   stale input, fault, disarm, deactivate, or shutdown.
8. Sensor/network diagnostics are read-only. All `ros2 topic echo` operations
   require a finite timeout.
9. Do not change host networking, install packages with sudo/apt, or connect to
   a physical robot without the operator's explicit approval for that step.
10. Do not store passwords, tokens, API keys, SSH credentials, subscription
    URLs, device handover PDFs, maps containing sensitive data, or `.env`.
11. Preserve the Unitree BSD-3-Clause notice for the model in
    `packages/go2_description/`.
12. Hardware-dependent dimensions, frame transforms, QoS, topic names and
    modes must be measured and recorded before supervised motion acceptance.

## Current validated phase (2026-07-31)

Supervised wireless motion, StopMove, human takeover, generation-1
Mapping/localization, and real Client Chinese-voice long-distance outbound and
autonomous-return Nav2 runs have succeeded. The operator retained video of the
successful physical round trip and accepted the small final placement
deviation as non-blocking. A fresh conversation may resume the existing
persistent stack without repeating a no-motion-only or centimeter-scale stage,
and it must not create new permit bundles or approval artifacts merely because
the conversation is new.

The local and global Nav2 costmaps consume the live MID-360-derived 2D scan and
have obstacle and inflation layers enabled. This proves that the laser obstacle
chain was active during physical navigation; it does not claim a dedicated
placed-obstacle avoidance trial. D435i remains a read-only preview source and
is not a Nav2 costmap input.

The implementation is not restricted to upstream-only code. The local dirty
integration and the supplied same-Go2 ROS1 archive are valid adaptation
references. Keep the existing disarmed-at-start behavior and the working
stop/cancel/watchdog/single-owner path; avoid adding new hard gates unless they
address a reproduced failure.
