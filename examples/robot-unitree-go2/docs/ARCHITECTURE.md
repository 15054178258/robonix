# Architecture

## Command path

```text
Robonix Liaison/Pilot
  -> semantic_navigation.navigate_landmark(name)
  -> map-id + verified-pose gate
  -> Atlas resolves robonix/service/navigation/navigate
  -> service-navigation-rbnx
  -> Nav2 NavigateToPose
  -> planner + controller + live costmaps
  -> bounded velocity_smoother (0.25 s timeout)
  -> /cmd_vel_guard_input -> navigation velocity guard -> /cmd_vel
  -> ROS-only go2_chassis_adapter
  -> local authenticated Unix SOCK_SEQPACKET
  -> SDK-only go2_sport_daemon
  -> Unitree SportClient::Move / StopMove
```

No process links ROS 2 Humble's CycloneDDS and Unitree SDK2's bundled
CycloneDDS into the same address space. The SDK daemon has its own watchdog,
validates the local peer UID and packet schema, and knows no ROS topic. The
ROS adapter has no Unitree SDK dependency.

Outside DDS, Robonix system services, capability providers, reverse audio and
the read-only dashboard all bind to `127.0.0.1`. The deployment owns those
values after loading local configuration, and a static compatibility gate
requires the upstream bind-host implementation and container forwarding.
Scene's debug UI and Mapping's unauthenticated administration UI remain off.

The dashboard's optional browser voice entry is a narrow exception to a
purely observational UI, not a hardware command surface. It is disabled by
default and accepts only strict 16 kHz mono PCM WAV from its own loopback
origin. The handoff path is browser → dashboard memory → the pinned local
`audio_client_bridge` → `robonix/system/liaison/voice`. Liaison remains the
only ASR/Pilot entry; the dashboard has no navigation client, ROS publisher,
or Unitree dependency. The telemetry capabilities remain read-only and the
independent chassis motion gates are unchanged.

## State and localization

```text
/sportmodestate
  -> position/velocity/quaternion validation
  -> IMU-oriented /odom + odom->base_link
  -> /imu/data -> read-only sensor relay -> /scanner/imu (health/UI)

/utlidar/cloud + /odom
  -> RTAB-Map mapping/localization
  -> /map + map->odom
  -> Nav2 local/global costmaps
```

The chassis owns `odom -> base_link`; localization owns `map -> odom`. There
must never be two publishers for either transform.

Lidar deskew is disabled until a read-only audit proves usable per-point
timestamps on the target firmware. Both RTAB-Map and the Nav2 scan projector
remain functional without deskew; they must not guess timestamps from the
PointCloud2 header.

Soma serves the pinned Go2 URDF to the local `robot_description` primitive.
That primitive rejects any byte mismatch, then the standard
`robot_state_publisher` publishes the remaining fixed joints on `/tf_static`
under CycloneDDS.

## Sensor path

The read-only sensor package relays the onboard PointCloud2 to
`/scanner/cloud`, republishes the validated chassis IMU under the sensor
provider, and isolates Unitree VideoClient from ROS through a second local IPC
bridge. The dashboard subscribes only; it has no publisher or motion control.
Camera availability is not inferred from one ROS Image. The bridge publishes a
sliding-window diagnostic and the Robonix provider declares the camera
capability only after `quality_ready=true`, `healthy=true`, an OK diagnostic,
fresh frames, sufficient valid FPS, and an acceptable rejection/API-error
ratio. The NX container health check independently enforces the same live
diagnostic, so persistent vendor API codes (including the observed SDK
``Call api timeout error`` code 3104), corrupt JPEGs, low FPS, or disconnects
make the runtime unhealthy.
The strict full-stack readiness report also requires the Dashboard's current
camera diagnostic to have `ready=true`, `healthy=true`, and level OK; a WARN or
ERROR window therefore cannot be reported as a camera/dashboard PASS merely
because the last ROS Image is still fresh.

## Runtime placement and publisher ownership

The deployment never chooses a publisher by discovery. Exactly one reviewed
placement is selected before startup:

| Placement | NX owns | Workstation owns |
| --- | --- | --- |
| `workstation-local` | nothing in this deployment | chassis/odom, sensors/camera, description/TF, Robonix and UI |
| `workstation-full-nx-sensors` | standardized camera, lidar and IMU topics only | chassis/odom, description/TF, full Robonix and UI |
| `workstation-ui-nx-full` | camera, lidar/IMU, chassis/odom and description/TF | read-only dashboard client only |

For the middle placement the NX skips both the passive chassis node and
`robot_state_publisher`; the workstation chassis supplies `/imu/data` over DDS
to the NX relay. For the UI-only placement the workstation does not boot Atlas,
Mapping, Nav2, speech, any local hardware provider, or a description publisher.
The UI therefore shows NX telemetry but no map/navigation state.

Each launcher first takes a non-blocking kernel `flock` for its complete
process lifetime. The lock—not a PID file—is authoritative, is released by the
kernel after crashes, and is accompanied by atomically replaced audit metadata.
This makes two local placement starts mutually exclusive without stale-PID
recovery or process-name matching.

Before creating any owned process, `scripts/check_runtime_ownership.sh` queries
publisher counts for both camera topics, `/scanner/cloud`, `/scanner/imu`,
`/odom`, and `/tf_static`. Counts must match the selected placement for three
stable samples. After children start, the same gate runs again and requires
the exact final publisher set; a missing or duplicate owner tears down that
startup. Queries are time-bounded and never publish, invoke a service, or call
an action. There is no automatic fallback because switching ownership after
discovery could briefly create two command-state or sensor writers.

## Semantic target

The public file `config/semantic_landmarks.yaml` is an unverified template.
The ignored local file contains the real saved approach pose. Matching is
Unicode-normalized longest-alias matching. It refuses unknown, ambiguous,
unverified, non-finite, wrong-frame, wrong-map and wrong-generation targets
before the navigation service is called. The skill resolves the mapping
service's `robonix/service/map/lifecycle` contract through Atlas, subscribes to
its transient-local `map/msg/MapLifecycle` topic, and requires the live
`(map_id, generation)` in `localization` mode before every goal. A lifecycle
change marks unfinished semantic runs failed and requests cancellation through
the navigation service; the skill does not publish a ROS or chassis command.
Its optional dashboard notifier sends display-only task phases over a validated
literal-loopback HTTP endpoint on a best-effort worker; it disables proxies and
redirects, and dashboard availability is not part of navigation acceptance.
Only one semantic navigation may be outstanding. The latch is released solely
by an explicit provider `SUCCEEDED`, `CANCELED`, or `FAILED` status—not by a
local semantic failure or an accepted asynchronous cancel request. Cancellation
therefore retries while polling status for a bounded window; a timeout remains
fail-closed and visibly high-risk until a later provider terminal confirmation.
