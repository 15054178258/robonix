# Host-native time-sync templates (inactive)

This directory contains the clock accounting library and an optional
host-native deployment design.  Nothing here is an installer, and every
configuration/unit file is intentionally suffixed `.template`.  There is no
`[Install]` section and no command in this directory enables a service.

The preferred first deployment is `deploy/jetson-time-sync`, which separates
a zero-capability DDS feeder from a networkless `SYS_TIME` chronyd container
and leaves the NX host package/config filesystem unchanged.  Use these host templates only if a later field audit
requires a persistent service and the operator separately approves every
package, file installation, service, and clock-change action.

`go2_clock_ref.py` is subscription-only and defaults to a bounded
`observe-only` run.  Its explicit `feed-chrony` path still cannot start without
an exact enable token, a root-controlled same-session writer approval, a live
GID match against schema-v2 original evidence, and an existing chronyd-owned
SOCK refclock socket.  It never calls
a clock-setting API; chronyd is the only intended clock servo.

## Workstation navigation-stamp correction

`navigation_stamp_discipline.py` contains two fail-closed, process-local
timestamp transforms.  Neither changes a system clock:

- `NavigationStampDiscipline` retains the reviewed fixed-offset behavior.
- `AffineNavigationStampDiscipline` estimates one common clock rate from the
  one-second lower envelopes of SportModeState, MID-360 IMU, and MID-360
  odometry.  It freezes an anchor and scale after qualification and applies
  that exact model to every stream.  PointCloud2 never gets an independent
  fit, so its acquisition lag remains observable.

`workstation_nomotion_stamp_node.py` defaults to `--mode fixed` for backward
compatibility.  The full no-motion/navigation dry-run launcher passes
`--mode affine` explicitly.  The affine ready record has a distinct schema;
motion consumers that only understand the fixed-offset schema therefore fail
closed instead of silently accepting the new mode.

The executable also has two explicit stream profiles. `--profile nomotion`
keeps SportModeState, MID-360 IMU, PointCloud2 and odometry for mapping/UI.
`--profile motion` uses the shared 200 ms motion freshness boundary and creates endpoints only for
SportModeState, IMU and odometry. It does not import PointCloud2 or create a
cloud subscription/publisher, preventing cloud copies from delaying the
single-threaded first-motion liveness path. The motion relay accepts only a
READY record carrying that exact affine/motion/three-stream contract.

`workstation_nomotion_cloud_relay.py` remains an isolated process and defaults
to `--profile nomotion`.  A staged Nav2 launcher may explicitly select
`--profile motion` and bind it to the strict motion stamp READY file and parent
process identities.  That mode applies the same frozen affine model to the raw
MID-360 cloud without creating any state, IMU, odometry, command, service, or
action endpoint.  It publishes at most every 100 ms, drops any cloud older
than 250 ms, and never sets `motion_ready`; downstream scan freshness and the
staged motion guard remain responsible for stopping and disarming motion.

## Persistent full voice/Nav2 profile

`render_workstation_persistent_nav2_manifest.py` minimally migrates the
verified corrected route to normal long-running Robonix operation. It retains
the full official system, Speech, Liaison/Pilot, semantic-navigation skill,
Mapping, Go2 camera and D435i preview. The Navigation provider owns its final
velocity guard and publishes only to the private chassis input; the chassis
adapter and SDK daemon retain independent limits, state freshness, command
watchdog and fail-closed stop behavior. Historical one-goal permit/evidence
tools remain untouched but are not runtime dependencies of this profile.

`start_workstation_persistent_voice_nav2.sh` starts this stack only after an
explicit current on-site acknowledgement, map generation and read-only state
values. It starts disarmed and never sends a goal or calls the arm service.
`persistent_nav2_arm.py --arm` is a separate current-operation gate: it rejects
an active goal, stale localization/odom/scan, map-generation drift, diagnostic
faults or unexpected command-topic owners, and only emits a bounded all-zero
preparation stream. `--disarm` remains available without the arm acknowledgement.
