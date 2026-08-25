# Disposable Jetson/NX dual-container time profile

This ARM64 profile is the preferred deployment path for Go2/NX clock-domain
alignment.  It separates chronyd from the subscription-only `go2_clock_ref`,
avoiding a chrony package or persistent chrony configuration on the host.

Nothing in this directory has been run on the physical NX.  Building the
image downloads `chrony` **inside the derivative Docker image**; the build and
every formal `SYS_TIME` run still require separate operator approval.

## Profiles

| Component/profile | Network | Linux capabilities | Clock behavior |
| --- | --- | --- | --- |
| default probe | host | Eff/Prm/Inh/Amb/Bnd all zero | bounded evidence only |
| graph preflight | host | Eff/Prm/Inh/Amb/Bnd all zero | no-adjust live GID check |
| DDS feeder | host | Eff/Prm/Inh/Amb/Bnd all zero | validated Unix-socket samples |
| chronyd | `none` | Eff/Prm/Bnd=`SYS_TIME`, Inh/Amb zero | bootstrap step or steady slew |

Only subscription processes use host networking; chronyd uses `--network
none`.  Components use immutable roots, `cap-drop ALL`, no-new-privileges,
bounded resources and runc.  There is no host bind, device, Docker socket,
host PID or host IPC namespace.  Formal components share only a dedicated
named volume, and `stop.sh` removes it.  DAC-separated subdirectories inside
that volume expose only the SOCK refclock to the feeder: the refclock socket
is owned by chronyd's dedicated UID 10002 and group-accessible to feeder UID
10001, while chronyd's command socket and state are UID/GID 10002 mode 0700.

The feeder entrypoint/healthcheck requires all five capability sets to be
zero.  Docker starts chronyd directly as PID 1 and disables its image
healthcheck so no helper process can inherit `SYS_TIME`; seccomp level 2 also
blocks later fork/exec.  The launcher reads that PID's host `/proc` status,
requires Eff/Prm/Bnd to equal bit 25, and rejects every non-`SYS_TIME` bit in
Inh/Amb (OCI may need that same bit ambient/inheritable for a direct non-root
exec).  The feeder deliberately has no `chronyc` access: otherwise
it could ask the privileged daemon to alter time indirectly.  Readiness
combines fresh authorized feeder samples with the bounded chronyd stdout
selection record for `GO2`.

## Build and no-adjust probe

First build the already-reviewed `deploy/jetson-readonly` image.  On the NX,
after separately approving the Docker build's network/package download:

```bash
./deploy/jetson-time-sync/validate-static.sh
./deploy/jetson-time-sync/build.sh
```

The safe default has no clock capability and does not start chronyd:

```bash
./deploy/jetson-time-sync/run.sh
```

The launcher waits for the bounded probe, copies evidence into ignored
`logs/go2-readonly/` with `docker cp`, and removes the container.  No host path
is mounted into the container.

## Evidence approval

Keep the original PCAP, topic-info, correlation and approval as direct files
in one same-session bundle directory.  Create the inactive schema-v3 approval
only after the directory also contains the completed two-hour probe and three
physically observed cold-boot trials.  A trial is
`TRIAL_ID,CURRENT,ATTESTATION,BOOT_ID,PCAP,TOPIC_INFO,CORRELATION`; exactly one
must use `CURRENT=true`:

```bash
python3 scripts/prepare_go2_time_approval.py \
  --pcap logs/go2-readonly/SESSION/go2-rtps.pcap \
  --topic-info \
  /sportmodestate=logs/go2-readonly/SESSION/sport_primary.topic-info.txt \
  --topic-correlation \
  /sportmodestate=logs/go2-readonly/SESSION/sport_primary.correlation.json \
  --stability-metadata logs/go2-readonly/SESSION/stability-metadata.json \
  --stability-summary logs/go2-readonly/SESSION/stability-summary.json \
  --cold-boot-trial \
  cold-boot-1,false,physical-cold-boot-observed-and-read-only,logs/go2-readonly/SESSION/boot-1.id,logs/go2-readonly/SESSION/boot-1.pcap,logs/go2-readonly/SESSION/boot-1.topic-info.txt,logs/go2-readonly/SESSION/boot-1.correlation.json \
  --cold-boot-trial \
  cold-boot-2,false,physical-cold-boot-observed-and-read-only,logs/go2-readonly/SESSION/boot-2.id,logs/go2-readonly/SESSION/boot-2.pcap,logs/go2-readonly/SESSION/boot-2.topic-info.txt,logs/go2-readonly/SESSION/boot-2.correlation.json \
  --cold-boot-trial \
  cold-boot-current,true,physical-cold-boot-observed-and-read-only,logs/go2-readonly/SESSION/boot-current.id,logs/go2-readonly/SESSION/go2-rtps.pcap,logs/go2-readonly/SESSION/sport_primary.topic-info.txt,logs/go2-readonly/SESSION/sport_primary.correlation.json \
  --output logs/go2-readonly/SESSION/go2-clock-ref-approval.json
```

Preparation and every formal launch reread topic-info and PCAP, recompute the
PCAP hash and RTPS correlation, compare the stored correlation, and bind the
unique source IP.  A zero-capability preflight then requires the live graph
GID to match that raw-PCAP writer.  Fake digests, missing originals, changed
bytes, multiple publishers, or a restarted publisher fail closed.

## Formal bootstrap (future explicit gate)

Do not execute these steps during read-only audit.  Before the first possible
step, a person must verify:

1. the robot remains stationary and no motion path is armed;
2. every ROS, Nav2, Robonix, UI, mapping, sensor, and unrelated Docker
   container is stopped (the launcher independently requires zero running
   containers);
3. competing host time services/processes and other `SYS_TIME` containers are
   absent; stopping one is a separately approved privileged action;
4. current GID-to-IP evidence and the no-adjust quality report pass
   `docs/TIME_SYNC.md`;
5. a console/SSH recovery path is open and the operator accepts that a large
   `CLOCK_REALTIME` step cannot be automatically undone.

Only for that reviewed event, export both exact acknowledgements and pass the
approval under this repository's ignored `logs/` directory:

```bash
export GO2_TIME_ALLOW_SYS_TIME=I_APPROVE_NX_HOST_CLOCK_DISCIPLINE_V1
export GO2_TIME_ALL_ROS_CONTAINERS_STOPPED=YES
./deploy/jetson-time-sync/run.sh \
  --bootstrap-step logs/go2-readonly/SESSION
unset GO2_TIME_ALLOW_SYS_TIME GO2_TIME_ALL_ROS_CONTAINERS_STOPPED
```

Only the canonical payload reconstructed by the launcher is copied through an
environment value.  Original evidence is never mounted.  Chronyd has no
network and the feeder has no clock capability.

Check without issuing a chronyd command or modifying time:

```bash
./deploy/jetson-time-sync/status.sh
```

The launcher waits for fresh authorized feeder samples and chronyd's `GO2`
selection log.  `status.sh` reports the feeder's source-minus-local offset,
jitter and drift measurements instead of opening the private chronyd command
socket.  Do not restart ROS/Nav2 until those measurements meet the gates in
`docs/TIME_SYNC.md` continuously and a new read-only check passes the existing
age guards.  Later restarts use `--steady-slew`, which has no `makestep`
directive.

## Rollback

```bash
./deploy/jetson-time-sync/stop.sh
```

This removes the sidecar and all of its ephemeral chrony files.  No host
package/config cleanup is needed.  A clock value already stepped cannot be
rolled back safely by guessing; verify the time and only then, with separate
approval, restore the site's selected host time service.  Do not restart
timestamp-sensitive ROS/Nav2 processes across an unreviewed discontinuity.
