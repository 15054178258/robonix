---
description: Go2 telemetry, dual-camera preview, voice handoff and map-bound localization-seed dashboard.
---

# Go2 telemetry dashboard

This service owns a child web-dashboard process and exposes one read-only
status capability. It independently observes the Go2 camera plus D435i color,
aligned depth and calibration, along with lidar, map, mapping pose, odometry,
and Nav2 action-status data. It has no robot motion-command capability.

When `initial_pose_maps_dir` is explicitly configured, it also owns one narrow
localization-only publisher on `/initialpose`. Operator estimates are stored
atomically as a per-map sidecar and can be restored only when the live
`MapLifecycle(map_id, generation)` matches the immutable map artifact exactly
and mode is `localization`. Existing sidecars are archived before replacement;
reset is a recoverable rename. Automatic restore is disabled in the physical
profile because a seed is valid only at the same marked physical start. This
surface never creates a Nav2 goal or chassis command.

An optional browser push-to-talk route is disabled unless
`GO2_DASHBOARD_BROWSER_VOICE_ENABLED=1`. It accepts only bounded same-origin
loopback PCM WAV, feeds the pinned local `audio_client_bridge`, and invokes the
official `robonix/system/liaison/voice` server-streaming contract. It doesn't
call ASR, navigation, ROS command APIs, or Unitree directly, and it doesn't add
an Atlas command capability to this provider.

`robonix/service/telemetry/dashboard/status` returns the web URL and a bounded
JSON health snapshot. Calling it never starts a task or changes ROS state.
