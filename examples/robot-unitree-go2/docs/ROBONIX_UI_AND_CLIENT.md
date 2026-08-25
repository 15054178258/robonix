# Robonix Go2 operator UIs

All four operator surfaces bind to laptop loopback. They are not exposed on
the campus network or the Go2 DDS cable.

| Surface | URL | Purpose |
|---|---|---|
| Official Robonix Client | <http://127.0.0.1:7860/> | Chinese text/voice turns, task state, Stop/steer and RTDL events |
| Scene semantic map | <http://127.0.0.1:50107/> | map-linked objects, rooms, POIs and semantic annotations |
| Mapping UI | <http://127.0.0.1:8091/> | RTABMap state, mapping/localization and map save/load |
| Go2 dashboard | <http://127.0.0.1:8092/> | camera, lidar, map, pose, TF and navigation state |

## One-command no-motion operator profile

The supervisor below starts the existing timestamp-corrected no-motion stack,
waits for Atlas and the three stack UIs, then starts and verifies the official
Client:

```bash
cd /home/zxq/workspace/robonix-go2/packages/robot-unitree-go2
bash scripts/start_operator_ui_nomotion.sh
```

It does not replace or weaken the no-motion launcher's private state evidence,
timestamp approval, wired-interface, DDS writer-identity, publisher-ownership,
canonical odometry, or port preflight. Audio is `auto`: a prepared microphone
is used when `sounddevice` is available, otherwise port 7860 remains a usable
text UI. Ctrl-C sends TERM only to the exact stack and Client child processes
started by this supervisor; it never searches for or kills processes by name.

After the readiness banner, open Client at <http://127.0.0.1:7860/>, Scene at
<http://127.0.0.1:50107/user>, Mapping at <http://127.0.0.1:8091/>, and the
Dashboard at <http://127.0.0.1:8092/>. Atlas is a gRPC endpoint at
`127.0.0.1:50051`, not a browser page.

## Manual start order

1. Start the audited Robonix Go2 profile and wait for its readiness gate. In a
   no-motion session, commands remain preview-only and navigation velocity is
   isolated from `/cmd_vel`.
2. Start the official client in a second terminal:

   ```bash
   cd /home/zxq/workspace/robonix-go2/packages/robot-unitree-go2
   bash scripts/start_robonix_client_local.sh
   ```

3. Open the four URLs above, then run the passive endpoint check:

   ```bash
   bash scripts/check_robonix_ui_stack.sh
   ```

The launcher stores settings and runtime files only under
`rbnx-build/client/`. The tracked template defaults to Atlas
`127.0.0.1:50051`, Chinese (`zh-CN`), and contains no credential.

## Speech backends

`SPEECH_BACKEND=local` is the default. Robonix uses the workspace-cached
Chinese FunASR model; the official client supplies microphone PCM through the
reverse `audio_client_bridge`. If PortAudio is not ready, the launcher starts a
text-only client and explains that condition without changing the host.

Full laptop microphone support needs the separately approved system packages
`libportaudio2` and `portaudio19-dev`, followed by installing the client's
`audio` extra into its workspace venv. No `sudo` or `apt` command is run by the
launcher.

Tencent ASR is optional. If it is selected later, provide its App ID, Secret ID
and Secret Key only as untracked runtime environment variables. Never put
those values in this repository, screenshots, logs, shell history or chat.

## Read-only Hands-free cancellation evidence

With the official Client already running and Hands-free shown as enabled, start
the independent bounded observer in a separate terminal. It never starts the
Client, changes Hands-free state, submits work, or calls a robot interface. The
live flag, duration, and new absolute output directory are all explicit:

```bash
cd /home/zxq/workspace/robonix-go2/packages/robot-unitree-go2
/home/zxq/workspace/robonix-go2/upstream/robonix-client/.venv/bin/python scripts/observe_handsfree_cancel_readonly.py --observe-live --duration-seconds 60 --poll-interval-seconds 0.5 --output-dir "$PWD/logs/handsfree-cancel-$(date -u +%Y%m%dT%H%M%SZ)"
```

Wait for the exact terminal line `READY: event stream accepted, Hands-free
enabled, active plans zero; disable Hands-free once now`. Only then disable
Hands-free once in the Client UI and leave the Client open until the bounded
window ends.
The disable edge must leave at least 25 seconds in that window; a shorter
post-disable observation fails closed even when no late event was seen.
The requested total window is bounded to 30--300 seconds and the status and
active-plan endpoints are polled independently at the fixed 0.5-second
interval; malformed schemas, unavailable results, or a sample gap over one
second fail closed.
The observer detects the first `true -> false` status edge and keeps its own
WebSocket open for the entire window. It reports any later `asr_final`, Pilot,
TTS or `session_done` event, transcript change, unavailable plan result, or
nonzero active-plan result. Because the UI click itself has no timestamp in the
observation API, any target event between the last confirmed `true` sample and
the first confirmed `false` sample also fails closed. Once READY is printed, a
late stale `enabled=true` HTTP response cannot clear an already observed event
from that transition window. The WebSocket uses a
pre-connected IPv4-loopback socket with proxy and redirects disabled; its
settings copy clears microphone and speaker route selections so observing the
event stream cannot start reverse audio I/O. `observations.jsonl` and
`summary.json` are each published only at completion with mode `0600` and
no-overwrite semantics. Exit 0 means the strict acceptance passed; exit 3 means evidence
was captured but one or more acceptance conditions failed. The JSONL can
contain recognized speech, so review it before sharing.

## Acceptance order

1. `/api/defaults` on port 7860 reports the local Atlas endpoint.
2. Client **Connect** shows Atlas/Liaison online; a harmless text request gets
   one structured Pilot/RTDL response.
3. Mapping UI shows a fresh map lifecycle and the same `map_id` as Scene.
4. Scene shows the saved map and an operator-verified vending-machine POI.
5. Dashboard shows fresh camera, lidar, `/odom`, `map -> odom -> base_link`, and
   navigation state.
6. Test one Chinese voice turn in preview/no-motion mode.
7. Only after the independent physical-motion gate passes may the same client
   request a bounded navigation goal.

The official Client never talks directly to the Go2 driver. Its path is
Client -> Atlas/Liaison -> Pilot -> capability/skill -> Executor. Stop and
cancel therefore stay inside the Robonix execution path and the chassis
watchdog remains the final fail-closed layer.
