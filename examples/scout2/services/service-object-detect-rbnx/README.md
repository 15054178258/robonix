# service-object-detect-rbnx

Robonix package for **LLM/VLM-based object detection**. It exposes an atlas-routed MCP service and replaces the older `yolo_world_rbnx` path. Owns the `service/perception/object_detect/*` namespace.

Catalog name: `robonix.service.object_detect`.

## Capability surface

| Contract                                                  | Mode | Transport | Source / handler                                          |
| --------------------------------------------------------- | ---- | --------- | --------------------------------------------------------- |
| `robonix/service/perception/object_detect/driver`         | rpc  | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate          |
| `robonix/service/perception/object_detect/detect_object`  | rpc  | MCP       | `DetectObject(object_name) → bbox_2d + object_center_3d`  |

There is no legacy `/yolo/detect_object` ROS-service fallback in this package.

## Compared with the old `yolo_world` path

- no GPU / ultralytics weights;
- RGB → OpenAI-compatible VLM → 2D bbox;
- in vertical-grasp mode the depth stream is completely unused (`skip_depth=true`); `service-grasp-pose-rbnx` maps the bbox center through the calibrated 2D homography and combines it with the configured tabletop z.

## Boot ordering

Must come **after** `primitive-orbbec-dabai_dcw-camera-rbnx` in the deploy manifest — Init resolves the RGB topic (and, when `skip_depth=false`, the depth topic) at atlas-registration time.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge — no ROS spawn. The bridge opens a gRPC server, registers the capability and declares only `object_detect/driver`, then blocks awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init the handler:

1. validates the LLM endpoint / API key / model name;
2. resolves `primitive/camera/rgb` and `primitive/camera/camera_info` via atlas (depth is skipped when `skip_depth=true`);
3. declares `object_detect/detect_object` on atlas.

## Layout

```
service-object-detect-rbnx/
├── package_manifest.yaml
├── capabilities/
│   └── service/perception/object_detect/{driver,detect_object}.v1.toml
├── config/                     example .env / prompt templates
├── llm_detect/                 Python package: atlas bridge + VLM client
├── scripts/
│   ├── build.sh                colcon build + rbnx codegen
│   └── start.sh                source ROS, exec atlas_bridge
└── src/                        vendored dependencies (if any)
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "llm_base_url":       "https://api.ofox.ai/v1",
  "llm_api_key":        "sk-...",
  "llm_model":          "google/gemini-3.1-flash-lite",
  "temperature":        0.0,
  "rotation_cam2arm":   true,
  "skip_depth":         true,
  "rgb_topic":          "",
  "camera_info_topic":  ""
}
```

- `rotation_cam2arm`: rotate the image 180° before sending to the LLM when the camera is mounted opposite to the arm.
- `skip_depth`: vertical-grasp mode — bypass depth back-projection entirely. `env LLM_DETECT_SKIP_DEPTH=1` is the fallback.
- Empty topic strings mean "atlas-resolved"; defaults already match `primitive-orbbec-dabai_dcw-camera-rbnx`.

## Build / run standalone

```bash
bash scripts/build.sh                           # colcon + rbnx codegen
ROBONIX_ATLAS=127.0.0.1:50051 \
    bash scripts/start.sh                       # registers, awaits Init
```

## Verification

```bash
rbnx caps | grep object_detect
# Expected: llm_detect provider with
#   robonix/service/perception/object_detect/{driver, detect_object}

# End-to-end via the atlas MCP surface:
rbnx ask "where is the paper on the desk?"

# The package intentionally has no direct ROS service surface.
```

## Orientation output (branch `feature/orientation-detection`)

In addition to the tight bounding box, the LLM is now asked to also
return a **coarse 4-way orientation** for the primary target, using the
object's principal / long axis:

| label        | meaning                                                | stroke |
| ------------ | ------------------------------------------------------ | ------ |
| `vertical`   | long axis is roughly up-down                           | `|`    |
| `horizontal` | long axis is roughly left-right                        | `--`   |
| `diag_tlbr`  | top-left → bottom-right                                | `\`    |
| `diag_trbl`  | top-right → bottom-left                                | `/`    |
| `unknown`    | round / symmetric / no clear axis, or low confidence   | (none) |

The orientation is:

- constrained via the strict JSON schema in
  `llm_detect/main.py` (`ORIENTATION_LABELS`);
- normalized (`_normalize_orientation`) so common LLM aliases like
  `up-down`, `landscape`, `slash`, `\` all fold onto the 4 canonical
  labels;
- rendered on the debug image (`~/.cache/robonix/object_detect/latest_ok.jpg`)
  as a **magenta stroke inside the bbox** — a different color from the
  green/yellow target box and the orange other-object boxes;
- returned to the caller inside the `detect_object` result as
  `orientation`.

When `rotation_cam2arm` is enabled the image is flipped 180°; since
these labels describe an unsigned axis, they are invariant under 180°
rotation and passed through unchanged.

## Standalone local test

`scripts/test_local.py` lets you run the exact same prompt + schema on
a **single local image**, without ROS or robonix_api, and drops an
annotated JPEG with the bbox + orientation stroke. Useful for
prompt-tuning and CI smoke tests.

```bash
pip install opencv-python openai toml
python scripts/test_local.py \
    --image  /path/to/frame.jpg \
    --object banana \
    --base-url https://api.openai.com/v1 \
    --model    gpt-4o \
    --api-key  sk-... \
    --output   /tmp/detect_out.jpg \
    --dump-json /tmp/detect_out.json
```

Env fallbacks (used when the matching flag is omitted):
`LLM_BASE_URL`, `LLM_MODEL`, `LLM_API_KEY`, `LLM_TEMPERATURE`.

Exit codes: `0` = target found + annotated, `1` = LLM said not found,
`2/3/4/5/64/66/70` = various IO / API / dependency errors (see script
source).

## License

This package: MulanPSL-2.0.
