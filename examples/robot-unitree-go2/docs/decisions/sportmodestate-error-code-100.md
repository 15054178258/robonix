# Undocumented `SportModeState.error_code=100/1013/2010` handling decision

Status: default fail-closed; explicit opaque-marker compatibility policy
implemented, physical motion still requires the remaining staged gates.

## Decision

The Go2 adapter must not interpret the observed non-zero
`SportModeState.error_code` values, including `100`, `1013`, and `2010`, as a motion
mode, a healthy state, an RPC result, or permission to arm. Zero remains the
only value accepted by default.

For this firmware combination, an operator may explicitly list the exact
current non-zero value in `GO2_ALLOWED_STATE_MARKERS`. That is only a
compatibility statement: the number remains opaque and all independent mode,
timestamp, freshness, measurement, RPC, lease, watchdog, stop, and explicit
arm gates remain mandatory. The first eligible value is bound. Any later
change blocks canonical odometry/TF/IMU and latches motion closed, even when
both values appear in the allowlist. An explicit disarm acknowledgement and a
new arm are required; a value outside the allowlist requires a reviewed
restart. Raw telemetry remains visible regardless of canonical eligibility.

## Source evidence

- Unitree's public `SportModeState.msg` defines `error_code` and `mode` as
  separate fields. The public ROS 2 README documents `mode` values 0 through
  13; it does not assign `100` to a mode.
- `unitree_sdk2/include/unitree/robot/go2/sport/sport_api.hpp` defines 1001,
  1015, and 1016 as Sport RPC API identifiers for Damp, SpeedLevel, and Hello.
  The ROS 2 example writes these values to `RequestIdentity.api_id`; they are
  not `SportModeState.mode` values. They also cannot fit in the message's
  `uint8 mode` field. The same official API table assigns FreeWalk the RPC ID
  2045, not 100.
- The public Go2 Sport error table contains 4101, 4201, and 4205. It does not
  define telemetry value 100. Numeric values used by another Unitree service
  are a separate error namespace and are not transferable to SportModeState.
- A later live sample reported `error_code=1013`. Older API tables used 1013
  for `BodyHeight`, but the pinned current Go2 changelog marks that API as no
  longer supported and the current Go2 `sport_api.hpp` does not define it.
  Seeing the same integer in telemetry therefore does not prove that
  `error_code` contains an API identifier or a healthy controller mode.
- A subsequent bounded, subscriber-only capture recorded 1,477 primary and
  101 fallback samples, all with `error_code=2010`, `mode=0`, and
  `gait_type=0`, while the supervised robot remained stationary. Source
  timestamps had no regressions. The pinned public Go2 API/error headers do
  not define telemetry value 2010. The changing non-zero value therefore
  establishes no semantic mapping. It is also why the compatibility policy
  binds one observed value and treats every later change as a latched event
  rather than silently accepting a second listed value.
- The official ROS 2 `ServiceSwitch` example returns zero after parsing JSON
  without checking `response.header.status.code`. A future controller switch
  implementation must validate the correlated remote response and then
  re-observe controller/state health; wrapper return zero alone is not proof
  of success.

Reviewed sources in this pinned checkout:

- `third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go/msg/SportModeState.msg`
- `third_party/unitree_ros2/README.md`
- `third_party/unitree_sdk2/include/unitree/robot/go2/sport/sport_api.hpp`
- `third_party/unitree_sdk2/include/unitree/robot/go2/sport/sport_error.hpp`
- `third_party/unitree_ros2/example/src/src/common/ros2_sport_client.cpp`

## Compatibility and reconsideration gates

The opaque compatibility exception may be configured only after the current
value is captured on this exact robot/firmware while stationary and the
operator explicitly reviews it. This does not complete the physical motion
gate and must not be presented as Unitree documentation.

The value may be assigned a semantic meaning only when both conditions are
met:

1. Unitree or the supplier provides verifiable documentation for
   `SportModeState.error_code` on this Go2 EDU firmware combination; and
2. the documented state is reproduced in a separately approved stationary
   test with motion transport disabled, retained state/topic evidence, the
   remote/e-stop verified, and no service toggling or mode-switch API call.

An explicitly approved, stationary, single-service App experiment may record
before/after correlation while software motion transport is absent. Such a
correlation is diagnostic evidence only and cannot assign a meaning or itself
authorize motion. Do not call a mode switch merely to clear the field.
