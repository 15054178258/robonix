---
provider: greet
kind: skill
description: Passerby greeting — watch the front camera, detect people (YOLO) and announce a friendly make-way line (VLM) over the speaker.
---

# greet_rbnx — passerby-greeting skill

Watches the robot's front RGB camera and greets people in its path so they can
make way. Soma gates the loop on a fresh chassis `moving` metric, so the skill
sleeps while the robot is stationary or body state is unavailable. While the
robot is moving, a **local YOLO** person detector runs on the Jetson GPU; only
when it actually sees people does
the skill call an independent **VLM** for one short,
friendly, witty line asking folks to step aside — then speaks it via
`speech/speak`. A cooldown stops it talking over the same crowd.

The robot base is **never driven** — the only outward action is speech.

## MCP tools

- `robonix/skill/greet/greet` — start the greeting watch (returns immediately).
- `robonix/skill/greet/greet/cancel` — stop it (camera stays subscribed).

## Upstream contracts consumed

- `robonix/primitive/camera/rgb` (ros2 topic) — the front RGB image.
- `robonix/service/speech/speak` (mcp) — announce the line aloud.
- `robonix/system/soma/get_health` (grpc) — fresh chassis motion state.

## Config (via Driver CMD_INIT)

| key | meaning | default |
|-----|---------|---------|
| `vlm.upstream` / `vlm.api_key` / `vlm.model` | independent OpenAI-compatible vision endpoint | — / — / `qwen-vl-plus` |
| `camera_provider_id` | front RGB provider selected through Atlas | required |
| `soma_provider_id` | Soma provider selected through Atlas | required |
| `chassis_provider_id` | chassis whose `moving` metric gates detection | required |
| `yolo_conf` | YOLO person confidence threshold | 0.4 |
| `yolo_weights` | path to the YOLO weight file (the robot has no internet) | `weights/yolov8n.pt` |
| `period_s` | seconds between detection cycles | 1.5 |
| `cooldown_s` | min seconds between spoken greetings | 15.0 |
| `speak_target` | speaker provider id (empty = first available) | "" |
