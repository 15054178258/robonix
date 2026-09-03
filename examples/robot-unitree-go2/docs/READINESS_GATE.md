# Real full-stack readiness gate

`scripts/check_stack_readiness.sh` is the final read-only check between
`rbnx boot` and any later, separately approved physical-motion gate. Robonix
boot is intentionally best-effort for non-core packages, so seeing its
“components up” line is not proof that the Go2 navigation deployment is ready.

Run it in another terminal after the stack has settled:

```bash
cd /path/to/robot-unitree-go2
bash scripts/check_stack_readiness.sh
```

The script never publishes a ROS message, sends an action goal, invokes a
Robonix task capability, or calls a Unitree command API. Every ROS CLI read is
wrapped by `timeout`. It exits zero only when every check is `PASS`; `FAIL` and
`UNKNOWN` both exit non-zero. A timestamped, mode-0600 JSON evidence report is
written under `logs/readiness/`.

The gate verifies:

- the exact active runtime's `rbnx-boot/state.json`, live boot process and the
  complete direct-component set (including best-effort services); corrected
  no-motion sessions are selected only after their private metadata, wrapper
  PID/start identity and held discipline lock have all been validated;
- ACTIVE Atlas providers, exact required capabilities, the exact audited
  `go2_sensors` namespace diagnostics described below, and no other namespace
  mismatch or provider error state;
- Atlas, Executor, Pilot, Liaison, Soma, semantic router and dashboard listen
  only on literal loopback addresses;
- semantic router `/v1/models`, dashboard `/api/status` plus `/healthz`, ROS
  connection, and fresh UI telemetry;
- current-session Speech log evidence that streaming ASR initialized as `OK`
  and is not the mock backend;
- fresh, exactly typed camera, point-cloud, projected LaserScan, MID-360 IMU
  and odometry messages with audited frame IDs and non-stale source timestamps;
- a valid, exactly typed transient-local `/map` snapshot in frame `map`; map
  publication is event-driven, so its nonzero, non-future source timestamp is
  validated without imposing a streaming-age ceiling;
- the exact latched `(map_id, localization, generation)` tuple against the
  physically verified vending-machine landmark file;
- fresh `map -> odom -> base_link` TF, the composed map pose, and the static
  lidar/camera/MID-360 IMU links (including `utlidar_imu`);
- all Nav2 lifecycle nodes configured by this deployment are `active [3]` and
  exactly one correctly typed server exists for `navigate_to_pose`,
  `compute_path_to_pose`, and `follow_path`.

Current Robonix boot state does not contain a `failures` array. The gate does
not treat that absence as success: it requires every manifest-owned direct
component to be present and alive, then independently requires the Soma-owned
primitives/skill and every service to be ACTIVE in Atlas. Missing or
uninspectable evidence remains `UNKNOWN` and blocks readiness.

`go2_sensors` intentionally owns one atomic lifecycle for the lidar relay,
MID-360 IMU relay and camera bridge while retaining the primary runtime
namespace `robonix/primitive/lidar`. Atlas consequently reports exactly two
advisory diagnostics: the ROS 2 contracts `robonix/primitive/imu/imu` and
`robonix/primitive/camera/rgb` are outside that primary namespace. The gate
requires those two exact `(provider, runtime namespace, contract, transport)`
tuples to appear exactly once each and records them as
`accepted_namespace_diagnostics` in the JSON evidence. Every capability's
`namespace_mismatch` field must be an actual JSON boolean. A missing, malformed,
false or duplicate required diagnostic, a different provider, namespace,
contract or transport, any additional namespace diagnostic, or changing the
runtime namespace to a broad prefix remains a readiness failure. Splitting the
bundle into independently owned provider lifecycles remains the long-term
architectural cleanup.

When no corrected-session pointer exists, the gate keeps the legacy deployment
directory behavior. A present but invalid, stale, symlinked, mismatched or
unlocked corrected-session pointer is rejected rather than silently falling
back to root state or logs.

This gate is necessary but never grants motion authority. Physical movement
still requires all safety gates in `docs/SAFETY.md` and an explicit staged test
approval with a present operator and working takeover/stop path.
