---
description: Resolve a saved Chinese semantic landmark to a verified map pose, then invoke and monitor the standard Robonix navigation service.
---

# Semantic navigation

Use `navigate_landmark` when the user asks to go to a named place such as
“自动售货机”. The skill does not infer coordinates. It accepts only one unique
landmark from the configured active map, requires a physically verified
approach pose, and delegates all planning, obstacle avoidance, status and
cancellation to `robonix/service/navigation/*`.

The landmark file records both `map_id` and `map_generation`. Before accepting
every goal, the skill resolves `robonix/service/map/lifecycle` through Atlas,
requires a latched `map/msg/MapLifecycle` sample in `localization` mode, and
requires the live `(map_id, generation)` pair to match the file. A runtime map
load, reset, mode change, malformed sample, or lifecycle-monitor failure closes
the gate, marks unfinished semantic runs failed, and requests cancellation from
the Robonix navigation service. The lifecycle integration is subscriber-only;
this skill has no ROS publisher or direct chassis API.

Semantic goals are single-flight. A later goal is rejected until the navigation
provider itself reports `SUCCEEDED`, `CANCELED`, or `FAILED` for the earlier
run. Local lifecycle invalidation and `CancelNavigation.accepted` are not proof
that Nav2 has stopped: cancellation is retried and status is polled for a
bounded window. If terminal confirmation times out, the dashboard reports a
high-risk failure and the single-flight latch stays closed until a later
provider status confirms a terminal state.

When configured, the skill also sends `received`, `resolving`, `resolved`,
`navigating`, and terminal display metadata to the Go2 dashboard. The URL must
be a literal loopback HTTP address with the exact `/api/semantic-task` path.
Proxy use and redirects are disabled, delivery is asynchronous/best-effort,
and a dashboard failure never changes navigation behavior.

Do not use this skill for unknown-object visual search or arbitrary coordinates.
