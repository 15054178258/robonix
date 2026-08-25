# Safety gates

The gates are cumulative. Passing one never bypasses another.

All non-DDS control, capability, reverse-audio and dashboard endpoints are
owned by the deployment and bind to loopback. Scene's debug UI and Mapping's
unauthenticated administration UI are disabled. Remote viewing is permitted
only through an authenticated SSH tunnel; do not expose or port-forward these
services directly onto a LAN.
The audited Mapping launcher must honor `MAPPING_ENABLE_VIZ=false` without
probing X11 or invoking `xhost`; build/start reject an upstream revision that
lacks that contract.

The browser microphone route remains off unless
`GO2_DASHBOARD_BROWSER_VOICE_ENABLED=1`. Enabling it authorizes only a
same-origin, in-memory audio handoff to Liaison; it does not authorize motion.
Keep the dashboard and both downstream endpoints on literal loopback, keep
Liaison access/voiceprint policy enabled for a physical demonstration, and do
not set `ROBONIX_LIAISON_VOICE_SAVE_DIR` when recordings must not persist.
Only one reverse-audio client may be active during the demo.

## Gate 0 — physical setup

- Go2 is on a level, dry, non-slippery floor with at least 2 m clearance.
- A competent operator is within reach of the official remote/app stop.
- No person, cable, stair, glass wall or fragile object is inside the route.
- Battery, joint, temperature and error states are normal in the official app.
- The operator decides posture using the official Unitree controls; Robonix
  never stands, sits or recovers the dog.

## Gate 1 — read-only data

Keep `GO2_ALLOW_MOTION=false`. Verify SportModeState, low state, lidar, video,
IMU, QoS, timestamps and frames. Starting this repository in this mode must
not create the SportClient motion daemon.

## Gate 2 — offline and bag replay

Run `scripts/validate_offline.sh`, then use
`scripts/gate2_replay_acceptance.sh` as documented in
[GATE2_REPLAY_ACCEPTANCE.md](GATE2_REPLAY_ACCEPTANCE.md). The harness replays
only an explicit state/sensor allowlist in a localhost-only isolated ROS
domain and terminates `/cmd_vel` at a subscription-only evidence sink. No real
dog or Go2 NIC is attached to that domain. `SKIP`, `READY_NOT_RUN`, and
`FIXTURE_ONLY` are not passes; only a fingerprinted real-bag result with every
required measured check may report `PASS`.

## Gate 3 — localization while stationary

Load the saved map, verify one `map -> odom -> base_link` tree, confirm the
footprint and sensor extrinsics, and let costmaps update while the robot does
not move. Unknown space is not traversable. Stop lidar input and confirm both
obstacle layers become non-current and the velocity smoother times out to zero;
then restore input and confirm a removed obstacle is cleared. Do not enable
deskew until usable PointCloud2 per-point timestamps have been measured.

## Gate 4 — enable transport, still disarmed

Set the local environment values below. `GO2_ALLOWED_MODES` must contain only
the decimal `SportModeState.mode` value(s) observed while this exact robot was
stationary during the read-only audit; never copy or guess another robot's
values:

```text
GO2_ALLOW_MOTION=true
GO2_OPERATOR_PRESENT=true
GO2_SAFETY_ACK=I_UNDERSTAND_GO2_CAN_MOVE
GO2_ALLOWED_MODES=<audited comma-separated values>
# Normally empty. If the current error_code is non-zero, this must contain
# only the exact robot/firmware value explicitly approved for this session.
GO2_ALLOWED_STATE_MARKERS=<empty-or-audited-decimal-values>
```

An allowed mode is necessary but not sufficient. Zero remains the only
`SportModeState.error_code` accepted by default. The exceptional
`GO2_ALLOWED_STATE_MARKERS` input gives an undocumented non-zero value no
meaning beyond “this exact opaque firmware marker was reviewed for this
session”; it is not a mode, health proof, or RPC acknowledgement. Any marker
change always blocks and latches a motion-capable profile closed, requiring an
explicit disarm acknowledgement followed by a new arm. A dedicated
motion-disabled `external_verified` manual-mapping manifest may separately opt
in to transitions wholly inside an explicit set of at least two reviewed
markers; that exception is rejected whenever `allow_motion=true` and does not
bypass freshness, measurement, odometry, TF, or continuity gates. A value not
already in the operator-provided allowlist requires a reviewed restart with
updated configuration. Never copy another robot's values or infer meaning from
an App colour.

The dedicated Go2 NIC must remain a point-to-point trusted link with exactly
`192.168.123.99/24`, no other IPv4 address, IPv6 disabled, no gateway or DNS,
and no bridge to Wi-Fi/VPN/LAN. The
arm RPC is a local operational gate, not a substitute for DDS authentication;
do not enable motion on a ROS graph that other machines or untrusted processes
can join.

Boot then permits the SDK motion transport to exist, but the adapter remains
DISARMED. It requires fresh valid state in an allowed mode, a continuous zero
command preamble, and an explicit arm service call. A fault latches and needs
manual re-arm.

## Gate 5 — supervised low-speed test

Only after the operator announces the clear test window may the adapter be
armed. Initial limits are 0 to 0.25 m/s forward (reverse is rejected), zero
lateral velocity and 0.40 rad/s yaw. Command and SDK watchdogs are 0.25 s and
0.30 s. While motion is enabled, SportModeState source timestamps must be
non-zero and strictly advance; duplicates and regressions do not refresh the
state watchdog. Any stale state, bad quaternion, NaN, position jump, disarm,
shutdown, IPC failure or SDK error requests StopMove and latches a fault.

## Emergency response

1. Use the official Unitree remote/app emergency stop first.
2. Press Ctrl-C in the Robonix deployment terminal.
3. Keep people away until the dog is stable and the official app reports a
   safe state.
4. Preserve logs; do not re-arm until the cause is understood.
