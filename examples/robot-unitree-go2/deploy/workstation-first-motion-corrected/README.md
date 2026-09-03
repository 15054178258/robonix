# Workstation first-motion corrected profile

`profile.yaml` is a reviewable safety contract and cannot be passed directly to
`rbnx`; it deliberately has no `manifestVersion`. The supervised entry point
is `scripts/start_first_motion_corrected.sh`. It renders a private, minimal
manifest only after all runtime and evidence gates pass. It must never be used
as a teleoperation or navigation launcher.

The first physical probe is a separate route from the no-motion/UI stack:

- corrected state: `/robonix/time_corrected/motion/sportmodestate`;
- strict affine timestamp witnesses: SportModeState, MID-360 IMU and
  MID-360 odometry only, each with the shared 200 ms motion freshness ceiling;
- no PointCloud2 subscription or publisher exists in this profile, so a large
  cloud cannot block the single-threaded Sport/IMU/odometry watchdog path;
- corrected private odometry input:
  `/robonix/time_corrected/raw/utlidar/robot_odom`;
- sole canonical odometry/TF authority: `go2_chassis` on `/odom` and
  `odom -> base_link` after `external_verified` validation;
- sole command source: `/go2_first_motion_probe` on
  `/go2/commissioning/cmd_vel`;
- `/cmd_vel` must have zero publishers;
- adapter and SDK daemon both cap forward speed at 0.05 m/s and yaw at zero;
- the adapter stops at 2.0 seconds or 0.10 m, whichever comes first;
- the daemon independently stops no later than 2.0 seconds;
- a private, short-lived permit is atomically renamed on provider init and
  cannot be replayed;
- explicit disarm and a continuous stationary post-stop observation are part
  of PASS, not cleanup best-effort.

`scripts/prepare_first_motion_permit.py` only creates the permit. It never
starts ROS or sends a command. The launcher independently checks graph
ownership, telemetry freshness, corrected time, evidence-to-permit binding,
physical-site acknowledgement, remote-stop readiness and human presence before
invoking `scripts/first_motion_probe.py`.

The timestamp chain remains non-authoritative for motion: it qualifies only
state, IMU and odometry and creates no command publisher. PointCloud2 remains
available only in the separate no-motion mapping/UI profile. The chassis is
the only canonical odometry authority. Canonical `/odom` and
`odom -> base_link` require both continuously fresh, valid SportModeState and
fresh verified external odometry. A timeout, invalid sample, timestamp fault,
or pose/yaw discontinuity permanently latches that external odometry process
closed; later good samples cannot reopen it and a reviewed process restart is
required. The one-time permit plus the adapter and SDK-daemon runtime gates are
the separate motion authority. Loss of the probe, Robonix process, identity
monitor, timestamp discipline, corrected-state relay, verified odometry,
state freshness or DDS connectivity tears down the command path.
