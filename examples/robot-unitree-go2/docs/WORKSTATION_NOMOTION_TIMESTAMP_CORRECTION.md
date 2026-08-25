# Workstation affine timestamp correction (no motion)

## Scope and status

This is an opt-in, motion-disabled commissioning profile for the case where
the reviewed Go2 `.161` DDS writers use one clock domain with a stable offset
and rate relationship to workstation realtime.  The profile began as a
fixed-offset design; the current launcher explicitly selects affine mode and
freezes one source-to-local clock model after live qualification.  The fixed
offset retained in the approval is an evidence-bound intercept check, not the
runtime correction itself.

The profile has now run against ROS and the stationary physical Go2.  That
live run established the fail-closed topology, but it also exposed the
long-duration affine extrapolation fault described below.  It is therefore
still a commissioning result, not a Nav2 or physical-motion acceptance result.

Normal `./start.sh` does not enable this layer.  It can only be selected by
`scripts/start_workstation_full_nomotion_corrected.sh` together with a
short-lived, mode-0600 approval file.  The wrapper reasserts
`GO2_ALLOW_MOTION=false`, clears all motion acknowledgements/modes and routes
any navigation velocity output to `/robonix/nomotion/cmd_vel`.

## Topic contract

The prelayer subscribes to raw topics without modifying or republishing over
them:

| Raw input (read only) | Private corrected output |
|---|---|
| `/sportmodestate` | `/robonix/time_corrected/raw/sportmodestate` |
| `/utlidar/imu` | `/robonix/time_corrected/raw/utlidar/imu` |
| `/utlidar/cloud` | `/robonix/time_corrected/raw/utlidar/cloud` |
| `/utlidar/robot_odom` | `/robonix/time_corrected/raw/utlidar/robot_odom` |

Only the ROS header/stamp is changed.  In the current affine mode the same
frozen model is applied to every stream:

```text
corrected = anchor_local + round((source - anchor_source) * source_to_local_scale)
```

Receipt time is observation evidence only and is never written into a
message.  Every other field, including `SportModeState.error_code`, is
preserved byte-for-field by a deep copy.  No raw-topic fallback exists.  The
older fixed-offset implementation remains covered offline, but the current
workstation launcher does not select it.

The corrected lidar odometry remains a **private adapter input** and is never
remapped directly.  `go2_chassis` runs with `odom_source=external_verified`,
checks its timestamps, frames, finite values, monotonicity, freshness and pose
jumps, and is the sole publisher of canonical `/odom` and
`odom -> base_link`. Mapping consumes `[lidar, imu, odom]` from the pinned Go2
providers and does not start a competing internal ICP odometry source. Nav2
and the dashboard both resolve the chassis-owned `/odom`. The checked-in lidar
deskew settings remain off because per-point time has not been accepted.

Canonical output also requires a continuously fresh, valid SportModeState as
an independent chassis-health witness. Hard timestamp, numeric, frame, replay,
or pose/yaw-continuity faults permanently latch external odometry closed.
Motion-capable processes also latch any liveness loss. The strictly passive
external-odometry profile may reacquire after pure liveness loss without
resetting retained source-stamp history.

The dedicated manual-mapping launcher has one narrower marker-only exception:
marker `1002` was observed on 2026-07-22 while the operator drove with the
paired official remote in the approved motion-disabled mapping session, after
stationary marker `100` had been validated. The launcher still requires fresh
subscriber-only evidence whose current marker is one of exactly `100,1002`,
then renders both values plus
`allow_passive_state_marker_transitions=true`. This keeps the independently
validated external odometry chain available only while those two opaque
markers alternate. The option is rejected for `allow_motion=true`, for
`sport_state` odometry, for fewer than two markers, and when startup evidence is
outside the set. A later outside marker immediately pauses canonical output and
is never accepted; output resumes only after one listed marker remains
continuous for at least `0.50 s` and five samples with no gap over `0.20 s`.
It does not relax timestamp, measurement, freshness, TF, continuity, or
external-odometry integrity latches.

## Evidence-bound approval, affine lock, and fault policy

The local approval validator requires all of the following:

- an absolute, non-symlink, current-user-owned regular file with mode 0600;
- exact no-motion v3 affine schema, `.161` clock-domain/source, one distinct reviewed
  DDS writer GID for each of the four raw streams, and an evidence SHA-256;
- `identity_evidence_verified=true`, `motion_enabled=false`, and the exact
  no-motion acknowledgement;
- a nonzero signed offset no larger than one hour and a validity window no
  longer than one hour that includes current realtime.
- two fixed, non-overlapping 30-second three-core lower-envelope drift
  estimates, their at-most-5 ppm common-rate difference, and their median as
  the approved affine common drift.  Legacy v2 remains fixed-mode-only and is
  rejected by every affine launcher/runtime path.

The template at
`deploy/time-sync/gates/workstation-nomotion-offset-approval.json.template`
is deliberately invalid.  It documents the schema only; do not fill its GIDs,
offset, or hash by hand.  The approval generator accepts only a complete
four-stream time capture and same-session RTPS writer/source evidence, derives
the fixed offset and per-stream GIDs itself, writes a new mode-0600 file, and
then validates that file with the runtime approval loader.

The offline generator accepts output only below this repository's ignored
`logs/` or `rbnx-build/` roots.  It recomputes each `.161` writer/source
correlation from the retained PCAP, reconstructs every raw probe record, and
replays only the four declared qualification streams through the checked-in
discipline.  Each 30-second core window must have all 30 one-second envelope
bins, bounded edge coverage, and no receipt gap beyond the stream's liveness
limit.  It writes a canonical evidence manifest beside the approval; the
approval SHA-256 is the exact digest of that manifest.  `sport_fallback` is a
witness only and never enters the offset calculation.

A separate long-lived graph-only monitor must become ready before the stamp
prelayer starts.  It requires exactly one publisher on every raw input and an
exact match with that stream's approved 24-byte GID.  It continues checking
the graph and approval every 250 ms.  A missing/restarted/duplicate writer,
GID mismatch, approval change, or approval expiry stops the monitor; the
wrapper then terminates the timestamp layer and the whole profile.  The
monitor creates no ROS subscriptions, publishers, clients, services, actions,
or Unitree API objects.

Before publication, all four live streams must cover two fixed,
non-overlapping 30-second windows.  Each window must independently pass the
absolute and pairwise drift gates. The no-motion profile permits a bounded
10 ppm difference between the two common rates; the physical-motion profile
uses 6 ppm, based on retained same-writer hardware evidence where each half
and the complete interval independently remained inside the unchanged
approval comparison. The complete 60-second interval then supplies the frozen
affine model. The approved common drift must agree with that live candidate
within an independent 5 ppm before READY, and the
approved offset must agree with the live lower-envelope intercept within
20 ms.  The no-motion affine anchor is biased 20 ms into
the past, while the legacy fixed-offset guard and every motion-capable profile
remain at 5 ms.  The no-motion candidate must retain at least 5 ms corrected
age during qualification, and every locked sample is checked against that
same past-side floor before it can be published.  This is an early-stop margin,
not additional future tolerance: the unchanged 2 ms hard-future breaker is
still evaluated first.  The locked common-model drift breaker is independently
5 ppm; qualification's pairwise core-stream agreement remains 25 ppm so
short-window transport noise cannot silently redefine the model gate.

Missing/late streams, exhaustion of the 5 ms affine age margin, a corrected
timestamp more than 2 ms in the future, timestamp regression, unexpected
duplicate rate, writer clock-domain mismatch, drift disagreement,
source/receipt jumps, local realtime discontinuity, approval expiry, or
disconnect latches a fault.  The prelayer exits and its private publishers
disappear; the wrapper then terminates the rest of this profile.  There is no
automatic relock, adaptive model, alternate raw input, or receipt-time
fallback.

## 2026-07-20 live result and offline endurance hardening

The motion-disabled affine profile became ready at
`2026-07-20 18:53:08 +08:00` and ran for approximately 2,205 seconds.  At
`19:29:53` the timestamp authority latched
`corrected_timestamp_in_future:mid360_imu`.  The timestamp process stopped,
the launcher observed that exit, and the complete no-motion stack was torn
down.  This is measured evidence that the fail-closed shutdown path works; it
is not evidence that long-duration affine operation is fixed.

The retained fault record from that run contains the reason and bounded phase
latencies, but predates the quantitative future-fault diagnostics and therefore
does not contain the exact triggering corrected age, source timestamp, or
receipt timestamp.  The current implementation now records the trigger stream,
exact corrected age, the unchanged 2 ms future limit, every affine safety
parameter, and the corrected-age snapshot for every stream.  A 5 ms soft-margin
fault records its exact triggering age and limit as well.  These new fields and
the endurance hardening have not yet been exercised by another physical live
run.

The focused affine regression suite now passes 32 of 32 tests, and the stamp
layer suite passes 36 of 36.  The tests retain the deterministic reproduction
in which a frozen model differs from the continuing source rate by only 3 ppm
and reaches the 2 ms future gate after about 2,200 seconds with the former
5 ms affine guard.  A paired regression proves that the no-motion-only 20 ms
affine guard remains above the 5 ms soft floor at the observed 2,205-second
point, then fail-closes at that floor while the timestamp is still in the past.
A separate boundary test proves that an actual hard-future violation keeps
priority.  This explains and mitigates the measured failure without weakening
the gate.

The new no-motion configuration was also replayed, without ROS or a live
robot process, against all 42,511 retained samples in
`logs/go2-readonly/workstation-nomotion-time-liaison-opt-in-20260720T1052Z/samples.jsonl`
(SHA-256 `1ac8b4162a27382fdabe04ecfa530e7362310c274429878d5324661497352449`).
Qualification and affine lock passed; the corrected-age baselines were
20.044154 ms for SportModeState, 19.500575 ms for IMU, 19.652346 ms for odom,
and 93.727311 ms for PointCloud2.  The mode-0600 replay receipt is saved beside
that input as `offline-affine-hardening-replay-20260720T122429Z.json` (SHA-256
`49359dc9e2a64b28208555bc9bace9d13fbcffd51c7bc02ff9d716e9e1d58ae9`).
This validates initial qualification margins only, not post-patch live
endurance.

Do not increase the 2 ms future allowance and do not add an in-process
automatic relock.  Approval validity remains unchanged and bounds each epoch to
at most one hour.  The next hardware check must use a new short-lived approval,
remain motion-disabled, retain the enriched fault evidence, and run a bounded
endurance observation while also recording workstation time-discipline state.
The 20 ms/5 ms margins make a bounded workstation epoch fail earlier and more
predictably; they do not make frozen affine extrapolation safe indefinitely.
The long-term architecture remains a reviewed common-clock deployment on the
NX/Go2 side, with the workstation acting primarily as the UI client.

## 2026-07-21 thermal-rate fault and bounded epoch refresh

A later motion-disabled run locked a common affine drift of
`12.377820560601265 ppm`.  After `2,332.060662988 s` (about 38 minutes 52
seconds), its current 60-second common drift was `17.56221691103554 ppm`.
The `5.184396350434277 ppm` deviation crossed the unchanged `5 ppm` locked
model breaker and the profile stopped.  All three core streams were live, the
local clock-base delta was only 9 ns, and the fault was
`affine_model_drift_deviation_exceeded`.  This is evidence of a changed thermal
rate regime, not justification for more future allowance or an automatic
in-process relock.  A new model is a new, explicitly evidenced process epoch.

`scripts/prepare_hot_stable_nomotion_epoch.py` implements that epoch boundary.
It requires both the existing no-motion acknowledgement and a separate
operator assertion that the stationary hardware has thermally stabilized.  It
also requires an absolute `--identity-evidence-dir`; there is intentionally no
default, newest-directory lookup, or implicit identity capture.  The helper:

1. fully revalidates the retained topic-info/PCAP/source correlations;
2. uses a graph-only ROS node to require three consecutive exact live GID
   matches for all four raw writers;
3. records a fresh 75-second subscription-only time capture;
4. repeats the exact live GID check and requires the two receipts and retained
   identity evidence to be identical; and
5. only then runs the existing offline generator for a 5-to-30-minute,
   motion-disabled affine approval.

It does not start the corrected stack.  The normal startup identity monitor
still rechecks the same GIDs continuously, so a writer restart after preparation
also fails closed.  If either bracketing check differs, discard that epoch and
collect new identity evidence under the separately approved passive-PCAP
procedure; do not point the helper at a different directory automatically.

```bash
IDENTITY_DIR="$PWD/logs/go2-readonly/<explicit-current-writer-identity-dir>"
python3 scripts/prepare_hot_stable_nomotion_epoch.py \
  --identity-evidence-dir "$IDENTITY_DIR" \
  --thermal-stable-ack \
    I_CONFIRM_THE_ROBOT_IS_STATIONARY_AND_TIME_SOURCES_ARE_THERMALLY_STABLE \
  --operator-ack I_APPROVE_THIS_FIXED_OFFSET_FOR_WORKSTATION_NOMOTION \
  --valid-for-seconds 900
```

A 900-second approval is suitable for an initial private ICP stationary test
only when the evaluator leaves startup and shutdown margin.  With the current
90-second minimum margin, use at most a 600-second observation under that
approval.  A complete 900-second observation needs at least a 1,200-second
approval; otherwise expiry can occur during the test and the result is not a
complete 15-minute acceptance.  ICP output must remain on private no-motion
topics/frames with TF publication disabled and cannot replace canonical odom
until its separate drift and ownership gates pass.

## Same-session evidence to no-motion startup

Run these only while the robot remains stationary and the dedicated wired
interface is already configured.  The first command records 75 seconds from
the four qualification streams plus the fallback witness.  The second records
the exact writer GIDs and correlates each one to RTPS DATA packets from
`192.168.123.161`.  It is passive, but packet capture must already be permitted;
if it reports a permission error, stop rather than adding `sudo` implicitly.
Sixty seconds is only the theoretical two-window total, not a reliable capture
duration; the extra 15 seconds provide bounded startup/edge margin, while the
generator still rejects missing bins, edge gaps, and unstable windows.
Do not reboot the robot, restart a writer, or reconnect the data plane between
the two captures and startup; any GID change is rejected by the live monitor.

```bash
TIME_DIR="$PWD/logs/go2-readonly/workstation-nomotion-time-$(date -u +%Y%m%dT%H%M%SZ)"
scripts/probe_go2_time_readonly.sh "$TIME_DIR" 75

IDENTITY_DIR="$PWD/logs/go2-readonly/workstation-nomotion-identity-$(date -u +%Y%m%dT%H%M%SZ)"
GO2_LOCATOR_PACKET_CAPTURE=YES \
  scripts/collect_go2_publisher_locators_readonly.sh \
  "$GO2_NETWORK_INTERFACE" "$IDENTITY_DIR" 30
```

Without typing an offset or GID, derive a new 15-minute approval and its
deterministic evidence manifest.  Both outputs are non-overwriting mode-0600
files under an ignored repository directory.

```bash
SESSION_ID="go2-nomotion-$(date -u +%Y%m%dT%H%M%SZ)"
APPROVAL="$PWD/rbnx-build/run/${SESSION_ID}-approval.json"
python3 scripts/prepare_workstation_nomotion_offset_approval.py \
  --time-evidence-dir "$TIME_DIR" \
  --identity-evidence-dir "$IDENTITY_DIR" \
  --session-id "$SESSION_ID" \
  --operator-ack I_APPROVE_THIS_FIXED_OFFSET_FOR_WORKSTATION_NOMOTION \
  --valid-for-seconds 900 \
  --output "$APPROVAL"
```

Start immediately while that approval remains valid.  This starts only the
motion-disabled corrected profile.  It does not arm the chassis and it routes
Nav2 velocity away from both `/cmd_vel` and the chassis input.

```bash
GO2_TIMESTAMP_APPROVAL_FILE="$APPROVAL" \
GO2_NETWORK_INTERFACE="$GO2_NETWORK_INTERFACE" \
  scripts/start_workstation_full_nomotion_corrected.sh
```

## Offline verification

These commands do not initialize ROS, DDS, the Unitree SDK, networking or the
physical robot:

```bash
python3 -m py_compile \
  deploy/time-sync/navigation_stamp_discipline.py \
  deploy/time-sync/workstation_nomotion_approval.py \
  deploy/time-sync/workstation_nomotion_identity_monitor.py \
  deploy/time-sync/workstation_nomotion_stamp_node.py \
  deploy/time-sync/render_workstation_nomotion_manifest.py \
  scripts/prepare_workstation_nomotion_offset_approval.py \
  tests/test_workstation_nomotion_stamp_layer.py
bash -n scripts/start_workstation_full_nomotion_corrected.sh \
  scripts/check_runtime_ownership.sh start.sh
python3 -m unittest \
  tests.test_affine_navigation_stamp_discipline \
  tests.test_workstation_nomotion_stamp_layer \
  tests.test_workstation_nomotion_approval_generator \
  tests.test_navigation_stamp_discipline \
  tests.test_runtime_ownership -v
```

## Hardware gate that remains closed

This profile remains motion-disabled. The chassis may publish canonical odom
and TF only from the verified private external stream; the ownership post-check
requires exactly one `/odom` publisher and rejects both zero and duplicates.
That establishes one navigation data topology, but does not authorize physical
motion: odometry direction/scale, TF, sensors, localization, Nav2 lifecycle,
watchdog, cancel/stop, command ownership and the staged site gate must still
pass with measured evidence.

Same-session four-stream time and writer-identity evidence was captured and a
real short-lived approval was used for the 2026-07-20 run above.  A subsequent
endurance run still requires fresh evidence and a new approval; the previous
run cannot be treated as reusable authorization.  Physical motion remains a
later, distinct safety gate.
