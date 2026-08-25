# Jetson full-stack no-motion ARM64 blueprint

This directory now contains an auditable, native ARM64 Docker **build
blueprint** for Ubuntu 22.04 / ROS 2 Humble on the Go2 EDU Orin NX.  It is
still intentionally not a runnable full-stack deployment.  The image label,
installed `profile.yaml`, entrypoint, health check, and host launcher agree on
that fact:

- `io.robonix.go2.runtime-complete=false`;
- `runtime.complete=false` and `runtime.launch_allowed=false`;
- `entrypoint.sh` exits with status 78 before starting ROS or Robonix;
- `healthcheck.sh` always exits with status 78;
- `run.sh` accepts only a separately reviewed image whose completion label is
  exactly `true`, so it rejects every image produced by the current
  `build.sh`.

Do not describe this image as installed, deployed, healthy, or runtime-ready.
No script in this directory starts ROS, DDS, Docker, or a container during
static validation.

## What the blueprint does build

`Dockerfile` is a Linux/ARM64 multi-stage build.  It starts from an exact,
locally loaded `jetson-readonly` image, then:

1. verifies ARM64, Jammy, ROS Humble, and the passive adapter baseline;
2. builds only the Robonix Nav2 terminal-controller plugins;
3. installs the ARM64 Humble runtime packages for Nav2, point-cloud projection,
   RTAB-Map, and IMU filtering;
4. copies an allowlisted source snapshot and the explicit Jetson-native
   Mapping/Navigation manifest selections into the blueprint image;
5. runs a root-filesystem verifier before finalizing the image.

The Docker build uses Docker's default isolated build network; it does not use
the host network and therefore does not join the Go2 DDS data plane.  Apt must
reach reviewed mirrors through that isolated network (or a warmed cache).

The Docker build context explicitly excludes `unitree_sdk2`, the Go2 chassis
SDK daemon, Unitree motion examples, generated build trees, VCS metadata,
credentials, and local environment files.  The inherited passive chassis
adapter remains motion-disabled and has no SDK daemon executable in the final
root filesystem.

Navigation is configured to publish only to
`/robonix/nomotion/cmd_vel`.  The passive chassis manifest uses the different,
non-actuating `/robonix/nomotion/chassis_input_disabled` input and
`allow_motion: false`; therefore the Navigation sink cannot become a chassis
command path in this profile.  The physical command endpoints have no
publisher or client here.

## Why runtime completion remains false

The repository does not yet carry all artifacts needed to make a reproducible
NX runtime.  The following are real, unresolved ARM64 gates:

1. a pinned ARM64 Robonix CLI and Robonix system build (the current source is
   outside this repository);
2. ARM64 `rbnx` code-generation outputs and locked Python/Rust dependencies;
3. an aarch64-verified local Chinese ASR backend and complete FunASR model
   bundle with runtime-download fallback disabled;
4. generated Mapping/Navigation Robonix contracts and their native runtime
   builds inside this exact image;
5. a PID-identity supervisor and live freshness checks for state, sensors, TF,
   localization, Nav2 sink output, speech, and loopback UI;
6. completed time-source evidence and cold-boot validation on the physical NX.

The JetPack 6/R36 Scene image is not silently reused on this JetPack 5/R35
device.  Saved semantic landmarks remain the planned first usable route;
real-time visual search is a later optional component.

## Mandatory future launch gates

The host launcher refuses startup unless all of these are true:

1. The host and selected image are Linux ARM64 (`aarch64` / `arm64`).
2. `eth0` has carrier and its sole global IPv4 address is
   `192.168.123.18/24`.
3. `eth0` has no default route or global IPv6 address, and IPv6 is disabled on
   that interface.  These scripts only inspect network state.
4. The reviewed `jetson-time-sync` feeder and networkless chronyd are healthy,
   the proved Go2 source is selected, and the host clock is no longer near the
   1970 factory value.
5. The operator supplies a local image reference and its full `sha256:` image
   ID.  The image must carry exact `jetson-full-nomotion`, `motion=false`, and
   `runtime-complete=true` labels.
6. The image has the reviewed entrypoint and health check.

The future OCI boundary is already fixed: all Linux capabilities are dropped,
`no-new-privileges` is enabled, the root filesystem is read-only, PID namespace
sharing is absent, and neither the Docker socket nor devices are mounted.
Host networking is used only for CycloneDDS on the dedicated robot Ethernet.

## Network topology

Keep data and management planes separate:

- `eth0=192.168.123.18/24`: Go2/MID-360 DDS only; no gateway, DNS, IPv6,
  bridge, or NAT.
- a future management NIC (`wlan0` or USB Ethernet): private operator network
  for SSH and an SSH tunnel to the loopback-only UI.  It must not bridge or NAT
  traffic into `eth0`.

Changing either interface remains a separately approved network operation.
None of these scripts calls NetworkManager, Netplan, DHCP, or an address/link
mutator.

## Offline static validation

From the repository root:

```bash
./deploy/jetson-full-nomotion/validate-static.sh
```

On the NX, the read-only network gate can be run independently:

```bash
./deploy/jetson-full-nomotion/validate-network.sh
```

## Native ARM64 blueprint build (not executed yet)

`build.sh` must run on the NX and requires a content-named local alias for the
already reviewed `jetson-readonly` image plus its exact image ID.  It never
pulls a base image.  The Docker build itself needs the configured Ubuntu/ROS
package mirrors (or a fully warmed BuildKit cache), so this is a later approved
build operation, not part of offline validation.

Example shape only—substitute the measured image ID from the NX audit:

```bash
export JETSON_FULL_NOMOTION_BASE_IMAGE_ID=sha256:<64-hex-id>
export JETSON_FULL_NOMOTION_BASE_IMAGE=robonix-local/jetson-readonly:sha256-<64-hex-id>
./deploy/jetson-full-nomotion/build.sh
```

The resulting tag is a deliberately unhealthy blueprint.  Do **not** invoke
`run.sh` with it; the launcher will reject its completion label.
