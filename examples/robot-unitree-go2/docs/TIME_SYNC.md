# Go2, MID-360, NX, and workstation time design

## Decision

Preserve every producer timestamp and align the NX clock domain to the
reviewed Go2 `SportModeState.stamp` source through chrony's SOCK refclock.  Do
not replace sensor/header stamps with callback receipt time.  Run
timestamp-sensitive ROS/Nav2 components on the NX after alignment; keep the
workstation as a thin UI/SSH client where possible.

The preferred implementation is the disposable dual-container
`deploy/jetson-time-sync` profile.  The host-native files under
`deploy/time-sync` are inactive templates for a later persistent deployment,
not installation instructions.

This decision aligns a local clock domain.  It does **not** prove accurate
UTC.  Field samples placed Go2/MID source time about 739 seconds behind the
workstation at that moment.  Synchronizing NX to Go2 would preserve a stable
shared ROS time base while inheriting that absolute UTC error.  Accurate UTC
requires a separately validated onboard GNSS/PPS, RTC, NTP, or vendor clock
mechanism.

## Evidence already established

These are observations, not permanent assumptions:

- `/sportmodestate`, `/utlidar/cloud`, and `/utlidar/imu` source stamps showed
  a similar approximately 739-second offset from workstation receipt time in
  the early read-only sample.
- NX `systemd-timesyncd` reported unsynchronized, with no usable NTP server or
  packet; its RTC reported 1970.  A cold boot therefore cannot rely on the RTC.
- the dedicated workstation Ethernet adapter had no associated PHC.  The only
  enumerated PHC belonged to Wi-Fi, so hardware PTP on that wired link is not
  established.
- source audit found no public Unitree time-discipline API appropriate for
  this integration.
- the current relays/adapters preserve source stamps and fail closed on stale
  or future data.  Restamping would hide the cause and corrupt cross-sensor
  ordering.

One same-session publisher-locator proof was also completed:

- `ros2 topic info --verbose /sportmodestate` reported writer GID
  `01.10.8f.17.35.15.4b.ce.5e.5f.ab.49.00.00.0a.03.00.00.00.00.00.00.00.00`;
- the first 12 bytes yield RTPS participant GUID prefix
  `01108f1735154bce5e5fab49`;
- a DATA packet with that header prefix and writer entity `00000a03`, captured
  only on the dedicated Go2 interface in the same session, had IPv4 source
  `192.168.123.161` and matching `SportModeState` payload evidence.

That proves the writer source for that session.  It does not create a static
allowlist: DDS GIDs commonly change after reboot or publisher restart.  A
fresh capture and approval are mandatory for each formal activation session.

## Read-only measurement

The no-adjust probe subscribes to the primary/fallback chassis timestamps and
MID-360 cloud/IMU timestamps:

```bash
./scripts/probe_go2_time_readonly.sh
```

It records source and receipt clocks, source-minus-receipt offset, drift,
jitter, duplicates, zero/malformed stamps, and backwards jumps.  It does not
create a publisher, service, action, Unitree client, network packet, or clock
request.  Every run is bounded and writes private evidence only under ignored
`logs/`.

Publisher metadata alone is not an IP locator:

```bash
./scripts/collect_go2_publisher_locators_readonly.sh
```

That default only records bounded `topic info --verbose` output.  If the
operator has separately confirmed unprivileged capture permission and the
dedicated interface, enable passive packet capture explicitly:

```bash
GO2_NIC=enp108s0  # replace with the dedicated Go2 interface on this host
GO2_LOCATOR_PACKET_CAPTURE=YES \
  ./scripts/collect_go2_publisher_locators_readonly.sh "$GO2_NIC"
```

The script never invokes sudo and never changes networking.  It accepts only
an interface already configured as `192.168.123.99/24` (workstation) or
`.18/24` (NX), rejects a default route on that interface, bounds every
`ros2 topic echo` with `timeout`, and bounds tcpdump by time and packet count.

The reproducible correlation is:

1. record the writer GID with `ros2 topic info --verbose`;
2. interpret GID bytes 0..11 as the RTPS participant GUID prefix;
3. interpret bytes 12..15 as the RTPS writer entity ID (remaining rmw GID
   bytes are not the DDS GUID);
4. in a same-session PCAP from only the dedicated interface, match both the
   RTPS header prefix and DATA/DATA_FRAG writer entity;
5. take IPv4 source only from those matching writer DATA packets;
6. require exactly one source or conclude that the locator is unproven.

`scripts/correlate_rtps_writer_locator.py` performs steps 2–6 offline.  PCAPs,
topic samples, SSH output, and approvals stay in ignored `logs/`; do not
commit them.

## Formal clock path

```text
Go2 SportModeState.stamp
  -> host-network, zero-capability go2_clock_ref
  -> source-vs-receipt quality and same-session GID gates
  -> group-scoped named-volume Unix SOCK refclock sample
  -> networkless chronyd (the only CAP_SYS_TIME owner/clock servo)
  -> NX CLOCK_REALTIME
  -> original ROS header stamps become comparable to local receipt time
```

The feeder can access only the refclock subdirectory.  Chronyd runs under a
separate UID; its Unix command socket and state live in a private mode-0700
subdirectory.  This prevents a compromised zero-capability feeder from using
`chronyc` as a confused-deputy path into the privileged daemon.

The bridge sends chrony's native `sock_sample`: receipt `timeval` plus
`source - receipt` offset, non-PPS, no leap request, and protocol magic.  This
matches chrony's own `refclock_sock.c`, where offset is defined as true time
minus system time.  The bridge has no direct clock-setting call.

The primary stream is preferred.  A fallback may feed only after its writer
has independent same-session approval, the primary becomes stale, and recent
primary/fallback offsets agree.  Zero, malformed, duplicate, regressing,
implausibly jumping, unstable, or unapproved samples are not fed.  Offset and
drift metrics use a true bounded sliding window, including for runs longer
than two hours.  A detected local realtime step clears quality windows and
forces a new warm-up rather than mixing pre/post-step statistics.

Official protocol references:

- <https://chrony-project.org/doc/4.4/chrony.conf.html#refclock>
- <https://gitlab.com/chrony/chrony/-/blob/4.8/refclock_sock.c>

## Acceptance gates before any formal bootstrap

All must pass without relaxing existing chassis/sensor age guards:

1. one live publisher per approved topic and one same-session RTPS writer
   source; source must be on `192.168.123.0/24` and not NX/workstation itself;
2. at least 10 minutes of short-run evidence and a two-hour stability run;
3. zero zero/malformed/regressing stamps; every duplicate remains visible and
   investigated rather than silently restamped;
4. offset jitter absolute-deviation p95 no more than 20 ms and absolute drift
   estimate no more than 500 ppm for the candidate chassis clock;
5. chassis and MID source offsets agree to an explained bound; a persistent
   mismatch blocks sensor fusion;
6. three cold-boot trials repeat discovery, writer correlation, monotonicity,
   and convergence without a manually seeded NX time;
7. the meaning of current `SportModeState.error_code=100` is resolved before
   any Nav2/motion activation (it does not block passive time measurement);
8. operator console/SSH access is available and no password is saved.

## First step and steady operation

The default sidecar profile has no chronyd and no capability.  A first formal
bootstrap is a separate high-impact event because `CAP_SYS_TIME` changes the
host clock and the offset after a cold boot can be large.

Before that event:

- stop every ROS/Nav2/Robonix/UI/mapping/sensor container; the launcher
  independently requires **zero** running containers;
- prove no competing host time service/process or other `SYS_TIME` container
  is active; stopping one remains a separately approved privileged action;
- keep the Go2 stationary and motion disabled;
- review fresh locator and probe artifacts;
- explicitly set both launcher acknowledgement tokens shown in the sidecar
  README.

After bootstrap, do not start ROS/Nav2 until all are true:

- both containers remain healthy; feeder samples stay fresh and authorized;
- runtime `/proc` checks show all five feeder capability sets zero, with only
  chronyd PID 1 owning `SYS_TIME` and no other capability bit in any set;
  chronyd is launched directly as its dedicated non-root UID, with its own
  healthcheck disabled and seccomp fork/exec blocking so no helper inherits it;
- the bounded chronyd stdout log contains a `Selected source ... GO2` record;
- feeder health reports absolute source-minus-local correction below 50 ms
  and absolute drift below 100 ppm for five continuous minutes (the feeder
  never opens the private chronyd command socket);
- a new no-adjust probe has no discontinuity/regression and source ages pass
  chassis 0.20 s / lidar 0.50 s / future 0.05 s guards;
- TF/localization consumers are started only after that stable epoch.

Use the slew-only profile for later steady restarts.  It contains no
`makestep`; if the offset is again large, stop timestamp-sensitive processes
and repeat the reviewed bootstrap instead of weakening the gate.

## Rollback and limits

Stopping/removing the sidecar rolls back all deployed software and ephemeral
configuration.  There are no host bind mounts, device grants, Docker socket,
installed host packages, or persistent files.  It does not reverse a past
clock step; blindly stepping backwards would create another discontinuity.
Verify time, keep ROS/Nav2 stopped, and restore the chosen host time service
only through a separately approved privileged action.

The optional host-native templates have invalid gate placeholders, `.template`
suffixes, and no systemd `[Install]` sections.  They are not a shortcut around
sidecar acceptance.
