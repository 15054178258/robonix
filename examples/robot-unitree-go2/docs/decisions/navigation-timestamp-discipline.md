# Go2 navigation timestamp discipline

Status: **default-disabled no-motion implementation with offline tests; not yet
run against ROS or the physical Go2**.

## Current source audit

The live adapters already fail closed:

- `packages/go2_chassis/.../go2_chassis_adapter_node.cpp` validates the raw
  `SportModeState.stamp` against the ROS clock before updating watchdog state or
  publishing `/odom`, `odom -> base_link`, or `/imu/data`. The checked-in age
  setting is 200 ms and future setting is 50 ms.
- `packages/go2_chassis/include/go2_chassis/safety_guard.hpp` rejects zero,
  malformed, old, future, and (for motion) non-advancing source stamps.
- `packages/go2_sensors/src/sensor_relay_node.cpp` rejects lidar stamps older
  than 500 ms and IMU stamps older than 200 ms. Parameters may only tighten
  these limits.
- `packages/go2_sensors/include/go2_sensors/stamp_guard.hpp` performs the
  signed age check without silently substituting receipt time.

Therefore the observed approximately 748-second mismatch correctly produces no
canonical odometry/sensor output. Increasing the freshness thresholds or
copying receipt time into navigation messages would defeat the watchdog and is
not an acceptable fix.

The chassis now enforces the committed 200 ms/50 ms ceilings as only-tighten
policy.  It also rejects every nonzero `SportModeState.error_code` before
canonical odom/TF/IMU publication.  The observed value `100` still needs an
authoritative firmware/SDK interpretation; timestamp correction deliberately
preserves it and cannot turn that state into healthy chassis odometry.

## Two timestamp classes

### UI-only receipt timestamp

Camera or point-cloud previews may be labelled with local receipt time only in
a dedicated `/ui/...` namespace. Such a label must carry provenance
`local_receipt_time_ui_only` and `navigation_eligible=false`. It must not feed
TF, localization, costmaps, Nav2, odometry, or the chassis state watchdog.

### Navigation-safe correction

A navigation stamp is eligible only after one authority has:

1. proven the current RTPS writers and their `.161` source/clock domain;
2. observed all required streams without publishing navigation data;
3. qualified one `local_realtime - source_time` offset;
4. explicitly locked that offset for the session;
5. applied the identical immutable offset to state, IMU, and cloud stamps;
6. re-run the existing per-message freshness checks after correction; and
7. continuously checked stream liveness, cross-stream agreement, drift, clock
   steps, timestamp regression, and source/receipt delta jumps.

Any failure latches the session faulted, stops canonical publication, and lets
the existing 200 ms chassis watchdog fail closed. There is no automatic
re-lock, offset adaptation, or fallback to receipt-time restamping.

Freshness, timestamp progression, delta jumps, and stream deadlines are cheap
per-message checks. Robust rolling drift/cross-stream statistics run at 1 Hz,
not at the 250--300 Hz state/IMU rate. A future wrapper must also call `poll()`
from a monotonic timer so total DDS silence still trips the deadline breaker.

The reusable pure implementation is
`deploy/time-sync/navigation_stamp_discipline.py`; its offline tests are in
`tests/test_navigation_stamp_discipline.py`. It has no ROS, network,
subprocess, publisher, or clock-setting dependency.  A separate explicit
no-motion wrapper embeds it for private corrected topics; the normal startup
and all motion paths remain unwired to timestamp correction.

## Estimator and limits

Receipt age is

```text
receipt_realtime - source_stamp = clock_offset + nonnegative delivery age
```

The authority uses the reference stream's rolling 1st-percentile receipt age
minus a 5 ms guard. A low percentile avoids treating ordinary queueing delay as
clock offset; subtracting the guard keeps corrected samples on the old side of
receipt time. This is still not proof of one-way latency, which is why writer
identity, cross-stream checks, and the hardware acceptance below remain
mandatory.

Default limits are deliberately inside the existing live ceilings:

| Check | Default | Basis |
|---|---:|---|
| Qualification span | 30 s | at least 30 one-second drift bins |
| Offset future guard | 5 ms | far above the measured 17 us clock-read span |
| Corrected state age | 100 ms | half of the existing 200 ms chassis ceiling |
| Corrected IMU age | 100 ms | half of the existing 200 ms IMU ceiling |
| Corrected cloud age | 250 ms | half of the existing 500 ms cloud ceiling |
| Workstation no-motion private lidar-odometry age | 200 ms | bounded startup-load allowance at the existing hard/liveness ceiling; accepted only by the chassis `external_verified` gate before canonical publication |
| State/IMU/cloud relative lag | 10/20/150 ms | cloud has measured acquisition/transport lag around 71--76 ms |
| Absolute robust drift | 50 ppm | measured approximately 9.3/9.9/9.3 ppm |
| Pairwise drift disagreement | 25 ppm | measured spread below 1 ppm in the latest 60 s replay |
| Source/receipt delta jump | 50 ms | strict default retained by every motion-capable profile and by state in the no-motion profile |
| Workstation no-motion sensor callback delta | 150 ms | isolated allowance for private IMU/cloud/lidar-odometry callback scheduling; corrected-age and hard ceilings remain unchanged |
| Local realtime discontinuity | 20 ms | immediate breaker, well below freshness ceilings |
| Locked-offset deviation | 20 ms | preserves at least 80 ms state-age margin |

The stored 60-second capture
`logs/go2-readonly/20260718T030700Z-time-probe/samples.jsonl` passed the offline
discipline with one fixed offset of `748.294606488 s`. Replay metrics were:

| Stream | robust drift | corrected p95 age | relative lower-envelope lag |
|---|---:|---:|---:|
| `sport_primary` | 9.31 ppm | 14.39 ms | reference |
| `mid360_imu` | 9.93 ppm | 14.91 ms | -0.60 ms |
| `mid360_cloud` | 9.26 ppm | 82.25 ms | 70.86 ms |

This replay proves the algorithm accepts the saved evidence; it does not by
itself authorize a live navigation or motion path. At the measured roughly
9 ppm drift, a fixed offset can move about 20 ms in roughly 37 minutes, so the
initial deployment must qualify per mission and bound mission duration. A
supported common-clock solution is preferable for long-running deployment.

The workstation no-motion run also exposed a callback-scheduling failure mode:
under stack-startup CPU load, valid messages can be delivered in a short burst
after the single-threaded timestamp executor has copied and published a large
PointCloud2 message.  This was first observed on the ~15 Hz cloud stream and,
in a subsequent full-stack run, on the ~150 Hz private MID-360 odometry
witness.  A later 271-second full-stack run showed the same recovery pattern
on `mid360_imu`.  Callback receipt periods can temporarily disagree with
source periods by more than the strict 50 ms delta breaker even though the
current corrected age remains within its independent bound.  The no-motion
profile therefore uses a per-stream 150 ms scheduling limit for
`mid360_imu`, `mid360_cloud`, and `mid360_odom` only.  `sport_primary`
continues to use 50 ms, and the normal navigation/motion profile retains 50 ms
for all of its streams.  Under full-stack startup load, a single valid private
lidar-odometry callback was also observed more than 100 ms old while remaining
below its existing 200 ms hard ceiling.
The workstation no-motion profile therefore permits its corrected age up to
that 200 ms ceiling.  Any value above 200 ms, or an odometry receipt outage
longer than 200 ms, still latches the discipline faulted.  All streams retain
the existing regression, relative-lag, drift, clock-step, and immutable-offset
checks. These exceptions apply only to the private timestamp relay. Only the
chassis may validate that relay and create canonical `/odom`; they do not
enable or widen any motion profile.

## Implemented no-motion architecture

The opt-in workstation profile uses exactly one timestamp authority for raw
`.161` state, MID-360 IMU, MID-360 cloud, and MID-360 lidar odometry.  It
subscribes to the original topics and publishes only these private copies:

- `/robonix/time_corrected/raw/sportmodestate`
- `/robonix/time_corrected/raw/utlidar/imu`
- `/robonix/time_corrected/raw/utlidar/cloud`
- `/robonix/time_corrected/raw/utlidar/robot_odom`

It deep-copies each message and changes only the source stamp by the one
explicitly approved immutable offset.  In particular, it does not change
`SportModeState.error_code`. The corrected lidar odometry remains private and
is never remapped directly. `go2_chassis` selects it through
`odom_source=external_verified`, applies the frame/freshness/jump checks, and
alone publishes canonical `/odom` plus `odom -> base_link`.

The no-motion manifest routes corrected MID-360 cloud/IMU through the existing
freshness-checking sensor relay. Mapping consumes
`[lidar, imu, odom]` with `provider_ids.odom: go2_chassis`; because external
odom is present, Mapping does not start internal ICP odometry. Nav2 and the
dashboard use the same chassis-owned canonical stream. The ownership
post-check requires exactly one `/odom` publisher and fails on zero or a
duplicate.

Activation is possible only through
`scripts/start_workstation_full_nomotion_corrected.sh` and a private,
short-lived fixed-offset approval.  Missing/expired approval, offset
disagreement, any stream gap, drift, regression, clock jump, or disconnect
latches a fault and terminates the whole profile.  Normal `start.sh` does not
select it.  See `docs/WORKSTATION_NOMOTION_TIMESTAMP_CORRECTION.md`.

Approval schema v2 binds a distinct exact DDS writer GID to each raw stream.
A single long-lived graph-only monitor checks every raw topic for exactly that
one publisher before and throughout the run.  A missing, duplicate, restarted,
or mismatched writer, an approval-file change, or expiry removes the monitor;
the wrapper then stops the prelayer and all downstream processes.  GIDs and
the fixed offset are generated from a complete same-session time capture and
RTPS DATA-writer/source correlation rather than typed into the template.

Do not independently estimate offsets in the chassis and sensor processes;
small differences would split TF and sensor time domains. Do not write an
offset into a user-editable ROS parameter while motion is possible.

## Remaining hardware acceptance

Before wiring this into Nav2 or enabling motion:

1. Capture at least three cold boots and at least ten minutes per boot. Retain
   RTPS writer GIDs/source IP, raw stamps, clock-pair span, regressions,
   duplicates, lower-envelope offset, robust drift, and pairwise agreement.
2. Prove every required writer belongs to the approved `.161` boot/session.
   Same subnet or a similar numeric timestamp is not sufficient identity proof.
3. Confirm the MID-360 cloud header denotes scan acquisition time and document
   whether usable per-point time exists. Include camera/depth in the disciplined
   set before using vision in localization or a costmap; UI-only camera does not
   need navigation correction.
4. Run the corrected profile with motion disabled. Verify private corrected
   streams, standardized IMU/cloud, one chassis-owned canonical `/odom`, the
   unique `odom -> base_link` TF, mapping output, and zero timestamp/TF errors.
5. In motion-disabled tests, disconnect Ethernet, pause each source, restart a
   writer, inject a timestamp jump, and cause a local realtime discontinuity.
   Canonical output must stop and the fault must remain latched without relock.
6. Verify localization accuracy, odometry direction/scale, cloud geometry,
   sensor extrinsics, watchdog stop, cancel, and single-controller ownership.
7. Resolve the observed `SportModeState error_code=100` and obtain an official
   field interpretation for this exact firmware/IDL. Do not loosen the current
   nonzero guard based only on an informal mode-number explanation. Then, in a
   separate motion-disabled gate, verify advancing chassis `/odom`, body IMU,
   and `odom -> base_link` before Nav2.
8. Only after all gates pass may a separately approved staged physical motion
   test begin at conservative speed with operator, remote, and emergency stop.
