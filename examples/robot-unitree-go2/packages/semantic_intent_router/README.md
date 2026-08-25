# Local semantic intent router

This loopback-only OpenAI-compatible endpoint gives Robonix Pilot a bounded,
credential-free production path for saved semantic landmarks. It is not a
perception model and never calls ROS, Nav2, Unitree APIs, or motion topics.

The server is **preview-only by default**. In preview mode the real
Speech → Liaison → Pilot route may still recognize a Chinese utterance and
match a saved semantic name, including an unverified template such as
`自动售货机`, but the returned RTDL tree is always empty. The response carries
the intended target and explicit blockers for the dashboard; Executor receives
no capability leaf, so Robonix navigation is not called. `live` must be selected
explicitly by the deployment only after the independent motion gate has passed:

```bash
SEMANTIC_INTENT_EXECUTION_MODE=preview \
  bash packages/semantic_intent_router/scripts/start.sh
```

In explicit `live` mode, the router loads the same landmark file as
`semantic_navigation`. It emits a
live `semantic_navigation.semantic_navigation_navigate_landmark` RTDL leaf only
when all of these conditions hold:

- the utterance uniquely matches one saved name or alias;
- the saved approach pose is marked `verified: true`;
- Pilot advertises the exact navigate, status, and cancel capabilities;
- no earlier executor result is missing or ambiguous.

It then polls the semantic status capability by the returned run ID until the
navigation provider reports a terminal state. An exact stop utterance calls the
semantic cancel capability and keeps polling; target changes are serialized so
the next navigation cannot start before the old provider goal is terminal.
Status/cancel failures and malformed feedback never claim that the robot has
stopped. They enter a fail-closed cancel or operator-hold path instead of
reissuing a navigation goal. Unknown text and unverified/multi-target requests
never create a new goal. The server only accepts a literal loopback bind address
and never logs request bodies or authorization headers.

The default two-second status cadence covers roughly 126 seconds within
Pilot's default 64 tool rounds. Every status detail also carries the semantic
run ID, so history compaction cannot silently detach the cancel/status loop from
an accepted Nav2 goal.

Run it from the deployment root:

```bash
bash packages/semantic_intent_router/scripts/start.sh
```

Pilot configuration for this mode is:

```text
VLM_BASE_URL=http://127.0.0.1:18080/v1
VLM_API_KEY=local-no-secret
VLM_MODEL=go2-semantic-router
```

`VLM_API_KEY` is a non-secret protocol placeholder. Do not put real credentials
in the repository.
