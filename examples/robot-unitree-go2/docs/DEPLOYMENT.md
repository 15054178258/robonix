# Deployment

## Prerequisites

- Ubuntu 22.04 x86_64 and ROS 2 Humble.
- Docker Engine for the upstream x86 Scene, Mapping and Navigation package
  targets. Their `jetson-native` manifests are ARM64/JetPack-only and must not
  be selected on this workstation.
- CycloneDDS RMW, Nav2, RTAB-Map, pointcloud-to-laserscan, colcon and CMake.
- `ros-humble-rosidl-generator-dds-idl`, required directly by the pinned
  Unitree ROS 2 message packages in addition to ROS's default generators.
- Robonix `rbnx`, Python build dependencies, Unitree ROS 2 messages and
  Unitree SDK2.
- Rust/Cargo for the Robonix contract generator and Rust system processes.
  `build.sh` compiles `robonix-codegen` plus every Rust system selected by the
  manifest (Atlas, Executor, Pilot, Liaison and Soma; Vitals when configured)
  from the same audited Robonix checkout.
- Python `grpcio-tools` for generated gRPC stubs. The workspace-local
  `.tools/rbnx-python` environment is preferred when present; global user-home
  installation is not required.
- When an operator supplies `HTTP_PROXY`/`HTTPS_PROXY`, `build.sh` explicitly
  enables Scene's build proxy switch for that invocation. No proxy URL is
  stored and no host network setting is changed.
- A dedicated Go2 Ethernet NIC configured according to the approved runbook.

No install command is run automatically. System package installation and
network changes require a separately approved operator step.

## Prepare local configuration

```bash
git clone --recursive https://github.com/syswonder/robot-unitree-go2.git
cd robot-unitree-go2
git submodule update --init --recursive
cp .env.example .env
```

Edit `.env` locally. At minimum set the real `GO2_NETWORK_INTERFACE`, map ID
and Pilot endpoint. Keep `GO2_ALLOW_MOTION=false` through all read-only tests.
In that mode, `build.sh` and `start.sh` forcibly route Navigation output to
`/robonix/nomotion/cmd_vel`; the value cannot be overridden from `.env`.
Canonical `/cmd_vel` is selected only after the complete motion gate is
valid. Build/start also reject any retained rbnx Navigation cache that lacks
this configurable-output contract.
Keep `GO2_RUNTIME_PLACEMENT=workstation-local` unless intentionally using one
of the two split profiles below. The value is an ownership declaration, not a
preference: startup never auto-selects or falls back to another publisher.

Inspect the PointCloud2 field table and record whether a valid per-point time
field exists. The checked-in mapping and scan-projection profiles keep deskew
disabled until that evidence and a bag-replay test exist; a message header
timestamp is not enough.

The public landmark template is deliberately invalid for movement. After the
map is loaded in localization mode, read its latched identity without
publishing anything:

```bash
timeout 5s ros2 topic echo --once \
  /robonix/map/lifecycle map/msg/MapLifecycle
```

Require `mode: localization`, and copy both `map_id` and `generation` from that
same sample. After the approach pose is physically checked, create the ignored
local file:

```bash
python3 scripts/set_landmark.py \
  --map-id lab_go2 \
  --generation <MAP_LIFECYCLE_GENERATION> \
  --x <MAP_X_METRES> --y <MAP_Y_METRES> --yaw <MAP_YAW_RADIANS> \
  --measured-by <OPERATOR_NAME> \
  --confirm-free-space YES_POSE_AND_FOOTPRINT_ARE_CLEAR
```

Then change the local `.env` entry to
`SEMANTIC_LANDMARKS_FILE=config/semantic_landmarks.local.yaml`. Before that,
the deployment loads the public unverified template and safely rejects the
navigation request. A later map load/reset or mode/generation change also
invalidates the saved pose and cancels unfinished semantic navigation.

## Offline validation and build

```bash
./scripts/validate_offline.sh
./build.sh
```

Both build and runtime force `ROBONIX_HOME` to the workspace-local
`.tools/robonix-home`. The config must exist and its `robonix_source_path`
must resolve to the existing audited `upstream/robonix-go2-build` checkout;
there is no `~/.robonix` fallback. The local `.tools/rbnx/bin` is used when
`rbnx` is absent from `PATH`. Runtime also prefers the workspace-local
`.tools/rbnx-python/bin/python3` and verifies that it can import `grpc` before
Soma starts any Python primitive. This keeps provider registration independent
of user-site or system-wide Python packages.

Robonix Rust state is equally workspace-owned. `CARGO_HOME` is fixed to
`.tools/cargo`, `RUSTUP_HOME` to `.tools/rustup`, and `CARGO_TARGET_DIR` to
`.tools/cargo-target/robonix`; inherited or `.env` values cannot redirect a
build into the user home or a system directory. `ROBONIX_BUILD_PROFILE`
accepts only `debug` (default) or `release`. Use the same value for build and
start, for example:

```bash
ROBONIX_BUILD_PROFILE=release ./build.sh
ROBONIX_BUILD_PROFILE=release ./start.sh
```

Before a full Robonix boot, `start.sh` prepends only the selected
`<CARGO_TARGET_DIR>/<profile>` directory to `PATH` and verifies every
manifest-selected system executable resolves back to that exact directory.
Missing, non-executable, redirected, or wrong-profile artifacts fail before
ROS/DDS/network preflight. The UI-client-only placement skips this gate because
it intentionally does not boot any Robonix system process.

Chinese ASR is also offline and fail closed. `start.sh` pins the ModelScope
cache and credentials path below this deployment's ignored `.cache/modelscope`
directory and defaults `GO2_FUNASR_MODEL_PATH` to the cached online Paraformer
model. Both `model.pt` and `config.yaml` must already exist and be non-empty.
Startup never downloads a missing/partial model or writes ModelScope state in
the user home directory.

The validation command parses every YAML/XML file and runs only offline unit
tests. It does not initialize ROS, DDS or Unitree SDK.

`build.sh` compiles only the `unitree_api` and `unitree_go` ROS message
packages from the pinned official submodule. It forces Unitree SDK example
builds off; none of the upstream motion examples are built or run by this
deployment.

Mapping and Navigation are gitlink-pinned submodules while their required
upstream PRs are under review. `scripts/verify_submodule_pins.sh` checks the
exact gitlink commit, clean worktree, configured/origin URL, and every recursive
submodule before tests or builds. This avoids Robonix's mutable URL cache,
which reuses an existing checkout without validating its URL, branch or HEAD.
After the upstream PRs merge, update each submodule URL to `syswonder` and pin
the reviewed merge commit before removing the temporary fork dependency.

The deployment deliberately uses each upstream service's default x86 Docker
manifest. A future Jetson installation needs a separately reviewed deployment
profile; changing the target name alone is not a portable conversion.

After the remote packages are built, a static compatibility gate verifies the
audited Scene lidar/odom/QoS contracts, the Robonix provider bind-host control,
container forwarding of the bind/advertise and CycloneDDS variables, and the
Navigation pending-cancel latch. It writes the actual Robonix, Mapping and
Navigation commits to `rbnx-build/upstream-lock.txt`. Build and start fail
closed until the corresponding upstream changes are present; branch names are
not treated as evidence by themselves.

## Read-only boot

### One-host default

With no NX container publishing the standardized topics, keep:

```dotenv
GO2_RUNTIME_PLACEMENT=workstation-local
GO2_ALLOW_MOTION=false
```

```bash
GO2_ALLOW_MOTION=false ./start.sh
```

The publisher preflight requires zero existing writers for camera image/info,
`/scanner/cloud`, `/scanner/imu`, `/odom`, and `/tf_static` before Robonix is
allowed to create the local owners.

### Workstation full Robonix + NX sensors only

This is the split profile for full Mapping/Nav2/speech/UI on the workstation
while the NX owns only sensor acquisition. Start the NX first:

```bash
./deploy/jetson-readonly/run.sh --sensors-only --camera
```

`--sensors-only` deliberately omits the NX passive chassis and
`robot_state_publisher`, so it creates neither `/odom` nor `/tf_static`. It
still starts the lidar/IMU relay and camera reader. Its `/scanner/imu` receives
the workstation chassis `/imu/data` after the workstation comes online.

On the workstation set the local `.env` entries:

```dotenv
GO2_RUNTIME_PLACEMENT=workstation-full-nx-sensors
GO2_ALLOW_MOTION=false
```

Then start normally:

```bash
./start.sh
```

Before boot, the workstation requires exactly one NX publisher for both camera
topics, `/scanner/cloud`, and `/scanner/imu`, and zero publishers for `/odom`
and `/tf_static`. Its `go2_sensors` provider registers the external topics but
spawns no local relay, VideoClient daemon, or camera bridge. The workstation
remains the sole chassis/odom and description/TF owner.

### NX full read-only + workstation UI/client only

Start the default NX ownership profile with camera:

```bash
./deploy/jetson-readonly/run.sh --camera
```

On the workstation set:

```dotenv
GO2_RUNTIME_PLACEMENT=workstation-ui-nx-full
GO2_ALLOW_MOTION=false
GO2_DASHBOARD_BROWSER_VOICE_ENABLED=0
```

Then run:

```bash
./start.sh
```

This path requires exactly one NX publisher for camera image/info,
`/scanner/cloud`, `/scanner/imu`, `/odom`, and `/tf_static`, then starts only
the loopback Dashboard subscriber at `http://127.0.0.1:8092/`. It does not boot
Robonix Atlas, chassis/sensor/description providers, Mapping, Nav2, speech, or
semantic navigation. Consequently camera/lidar/odometry are visible, while
map and navigation panels remain unavailable. Browser voice is forcibly off.

For all placements, a duplicate publisher fails immediately. A missing
external owner fails after the bounded discovery deadline. Startup holds an
atomic process-lifetime placement lease and repeats the publisher check after
all selected children are visible. The kernel lock recovers automatically
after a crash; audit metadata is never treated as authority. Both checks only
run `ros2 topic info -v` under `timeout`; they never publish a topic, invoke a
service/action, change networking, or open a motion API.

All non-DDS endpoints bind only to `127.0.0.1`: Atlas `50051`, Executor
`50061`, Pilot `50071`, Liaison/client entry `50081`, Soma `50091`, reverse
audio `60002`, capability providers on their allocated ports, Mapping's
operator Save/Load UI on `8091`, and the Go2 read-only dashboard on `8092` by
default. Scene's debug UI remains disabled. Mapping's unauthenticated operator
UI is intentionally loopback-only and is used for supervised map capture; use
an authenticated SSH tunnel if it must be viewed remotely. Mapping
visualization is disabled; the upstream compatibility gate requires its launcher to avoid X11
probing, mounts and `xhost` entirely in that mode. `DISPLAY` is additionally
removed as defense in depth.

For the two full-Robonix workstation placements, `start.sh` launches the local
semantic-intent endpoint before `rbnx boot`. It owns that endpoint with an
exclusive kernel lease and requires three consecutive exact, proxy-free
responses from `http://127.0.0.1:18080/v1/models`. A missing model contract,
duplicate launcher, early exit, or later endpoint crash stops/fails the Robonix
boot rather than silently sending Pilot to an unrelated endpoint. The
UI-client-only placement does not start this endpoint because Pilot and
semantic navigation are intentionally absent there.

For access from another trusted computer, use an authenticated SSH tunnel;
never expose these ports directly:

```bash
ssh -N \
  -L 50081:127.0.0.1:50081 \
  -L 60002:127.0.0.1:60002 \
  -L 8092:127.0.0.1:8092 \
  <user>@<ubuntu-host>
```

Enter the SSH password interactively and do not save it in this repository.

`start.sh` never changes network state. It fails closed unless the selected
interface is a real, non-Wi-Fi physical NIC that is UP with exactly
`192.168.123.99/24`, no other IPv4 address, IPv6 disabled, and no gateway or
DNS entry in either the kernel routing state or NetworkManager. It then
ignores any inherited `CYCLONEDDS_URI` and unconditionally binds this
deployment's CycloneDDS graph to that one NIC. The Go2 link and ROS domain must
be treated as a trusted, isolated control network; do not bridge or route it to
Wi-Fi, a VPN, or an untrusted LAN.

While stationary, stop the lidar replay/input and confirm the local and global
obstacle layers become non-current within the configured 0.5 s expected update
period. Restore input, place then remove a harmless test object, and verify the
projected `+Inf` rays clear it from both costmaps. These are required checks,
not offline-test claims.

Saved maps, scene data, generated bindings, logs and model/build caches are
kept under the ignored `rbnx-build/` or `logs/` directories in this checkout;
the deployment does not write runtime state into another project directory.
Scene receives one audited host mount at `rbnx-build/data/scene` and keeps its
object-memory database, scene-graph cache and annotations below that mount;
none of those paths is left in the disposable container filesystem.

## Stop

Press Ctrl-C in the foreground deployment or run `./stop.sh` from a second
terminal. Shutdown is scoped to this Robonix deployment; scripts never use a
broad process-name kill. The UI-only launcher has its own atomic lock and the
stop path validates PID start-time plus ownership of that exact lock before
signalling it. Stale or malformed optional UI metadata is reported and removed
when safe, but never prevents the independent Robonix shutdown path.
The semantic-intent endpoint has a separate atomic lease; shutdown validates
both launcher and child PID start-time plus ownership of that exact lock before
signalling the child. It never searches process command lines or kills by name.

The full launcher also refuses to overwrite a pre-existing
`rbnx-boot/state.json`. If its own `rbnx boot` child terminates but leaves that
file behind, cleanup verifies both the canonical manifest path and the exact
recorded boot PID before invoking the same selected binary as
`rbnx shutdown -f <manifest>`. A malformed, symlinked, foreign-manifest or
foreign-PID state is retained for inspection and no process is signalled.
