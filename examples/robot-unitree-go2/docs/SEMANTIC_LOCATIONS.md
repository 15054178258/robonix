# Named map locations

`config/semantic_landmarks.local.yaml` is the ignored, deployment-local source
of named map locations. Every entry is tied to the document's exact
`map_id`, positive `map_generation`, and `map` frame. Do not copy coordinates
between rebuilt map generations.

The public `config/semantic_landmarks.yaml` remains an unverified template.
Setting `verified: true` is a human assertion that the recorded geometry was
measured on the named map generation. A generated zero or guessed coordinate
must never be marked verified.

## Reusable operator UI

Scene at `http://127.0.0.1:50107/user` already provides:

- **Pose estimate**: click the robot's measured initial position to publish
  `/initialpose` through Mapping. This is runtime localization, not a
  navigation destination.
- **Annotate room**: draw, name, rename, persist, and reload room polygons.
  Scene stores them per `map_id` and marks them stale after a generation
  change.
- A stable annotation API. `POST /api/annotations` accepts either a room
  polygon (`{"kind":"room","name":"走廊","points":[[x,y],...]}`) or a POI
  (`{"kind":"poi","name":"充电点","points":[[x,y]],"theta":0.0}`).
  The current `/user` UI draws rooms; POI creation is API-only.

Scene annotations and semantic-navigation YAML are deliberately separate
stores today. Copy only operator-reviewed map-frame measurements between them;
there is no automatic trust promotion from a drawn room to `verified: true`.

## Backward-compatible schema

Existing schema-version-2 entries that omit `kind` remain navigation
destinations. Existing entries that omit `arrival_radius` use a conservative
`0.35` metre semantic default.

```yaml
schema_version: 2
map_id: measured_saved_map_id
map_generation: 1
frame_id: map
landmarks:
  - id: east_meeting_room
    name: 东侧会议室
    aliases: [东会议室, 一号会议室]
    kind: navigation
    verified: false
    arrival_radius: 0.50
    pose: {x: measured_x, y: measured_y, yaw: measured_yaw}
    region:
      # Optional named range. This is the same map-metre point convention as
      # a Scene room annotation.
      points:
        - [measured_x1, measured_y1]
        - [measured_x2, measured_y2]
        - [measured_x3, measured_y3]
    metadata:
      measured_by: ""
      measured_at: ""
      source: unconfigured_template

  - id: initial_reference
    name: 机器人初始位置
    aliases: [初始点, 起始参考点]
    kind: marker
    verified: false
    pose: {x: measured_x, y: measured_y, yaw: measured_yaw}
    metadata:
      measured_by: ""
      measured_at: ""
      source: unconfigured_template
```

A `navigation` entry requires a finite point pose and an arrival radius from
`0.05` through `10.0` metres. Its optional non-zero-area region describes the
named place's extent. A `marker` may contain a point, a region, or both, but is
never returned as a navigation goal. Therefore names such as “机器人初始位置”
or “起点” can be displayed and audited without making “回到起点” move the
robot.

Names and aliases are Unicode-normalized, case-folded, and matched across
common Chinese/English punctuation and ASR spacing. One utterance may resolve
only one navigation destination; two destination names remain an explicit
ambiguity. Mentioning a marker as context, for example “从起点去一号会议室”,
does not hide the unique navigation destination.

`arrival_radius` and `region` are currently semantic/audit metadata. The
Robonix navigation RPC still receives the reviewed `pose` point, while Nav2's
goal checker remains deployment-configured. No per-request tolerance is
silently changed by this file.

## Add or update measured entries

The helper only writes YAML; it starts no ROS process and sends no command:

```bash
python3 scripts/set_landmark.py \
  --id east_meeting_room \
  --name 东侧会议室 \
  --alias 东会议室 \
  --alias 一号会议室 \
  --kind navigation \
  --map-id measured_saved_map_id \
  --generation measured_generation \
  --x measured_x --y measured_y --yaw measured_yaw \
  --arrival-radius 0.50 \
  --region-point measured_x1,measured_y1 \
  --region-point measured_x2,measured_y2 \
  --region-point measured_x3,measured_y3 \
  --measured-by human_operator \
  --confirm-free-space YES_POSE_AND_FOOTPRINT_ARE_CLEAR
```

Use `--kind marker` for a non-navigation start/reference point; omit
`--arrival-radius`. New entries require an explicit stable `--id` and
`--name`. Re-running the command with the same ID updates that entry, allowing
multiple named destinations to coexist in one file.
