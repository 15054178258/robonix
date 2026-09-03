# Motion RPC acknowledgement and controller-isolation contract

Status: corrected locally after a supervised negative trial and revalidated
offline. No physical motion or measured stopping success is claimed yet.

## Upstream audit result

The pinned Unitree ROS 2 Go2 example is not a sufficient acknowledgement
contract for autonomous motion. Its sport wrapper:

- selects a response only by `header.identity.api_id`;
- does not require the response `identity.id` to equal the request identity;
- does not check `header.status.code`; and
- waits without a timeout.

See
`third_party/unitree_ros2/example/src/include/common/ros2_sport_client.h:77`.
The newer generic example client improves identity matching, status checking,
and timeout handling, but it still does not jointly verify both identity fields
and the emitted request lease. See
`third_party/unitree_ros2/example/src/include/common/base_client.hpp:31`.

The pinned SDK2 Go2 `SportClient` constructor defaults to
`enableLease = false`, and its official Go2 examples use that default. The
previous Robonix implementation enabled a positive one-second SDK lease. On the
tested wireless path its renewal produced `3104` timeouts and later safety
calls were rejected with `3205`. The controller therefore uses the official
lease-disabled client path and treats lease ID zero as an exact expected value,
not as an unset value or wildcard.

This upstream default does not prove that zero-lease requests are exclusive,
that they can override a positive lease, or that they are an emergency stop.
Those claims are not part of this contract.

## Local fail-closed contract

Before an arm request can lead to a non-zero command, all of these checks must
pass:

1. `RobotStateClient::ServiceList()` succeeds.
2. Exactly one remote service named `sport_mode` exists and reports active
   state (`status == 1`). No service is switched by this check.
3. Before Robonix boot, a read-only graph capture records the stable set of
   firmware-provided DDS writer GIDs on `/api/sport/request`. During the
   first-motion probe that complete set must remain present and boot must have
   added exactly one stable workstation SDK writer GID.
4. It continuously observes no writer on `/api/sport_lease/request`, exactly
   one probe publisher and one adapter subscriber on the dedicated
   commissioning command topic, and no publisher on canonical `/cmd_vel`.
5. The isolated SDK daemon performs `StopMove` through the primary
   lease-disabled `SportClient`.
6. The observer sees that exact request with API `1003`, lease ID zero, one
   positive request identity, an exact response identity/API match, SDK result
   zero, and remote status zero.

The pre-arm `StopMove` proves only that this exact stop/control RPC was accepted
at that instant. It is not durable ownership and is not a physical stationary
measurement. Fresh state and odometry, operator observation, remote takeover,
and the post-stop measurement remain independent gates.

The pinned SDK2 ABI has two different wire contracts. `Move` uses the
no-output `Client::Call` overload, sets `RequestPolicy.noreply=true`, and
returns after `ClientStub::Send` writes the DDS request. The pinned
`SportClient::Init()` registers `Move` with wire priority `0`. `StopMove` uses
the response-bearing overload, is registered with wire priority `1`, and waits
through `SendRequest`. The x86_64 and arm64 pinned libraries have the same
behavior, and the official ROS 2 Go2 `Move` wrapper likewise only publishes a
request.

Every later `Move` and `StopMove` call independently requires:

- SDK result zero;
- the expected API ID and exact lease ID zero on the emitted request;
- exactly one non-ambiguous matching request identity;
- exact wire `priority` and `noreply` policy matches; and
- an exact serialized request-parameter match. For `Move`, this binds the
  witnessed JSON `x/y/z` values to the velocity passed to the official SDK.

For one-way `Move`, missing response is the expected protocol behavior and is
not called a remote acknowledgement. If a response is nevertheless observed,
its identity/API and status must still match exactly. Physical acceptance
additionally requires at least 0.02 m measured forward displacement, bounded
lateral/yaw deviation, a unique adapter publisher on canonical `/odom`, fresh
`external_verified` odometry diagnostics, a witnessed
`commissioning_motion_active=true` phase, and the continuous state, graph,
watchdog, Stop/Disarm, and post-stop checks. For response-bearing `StopMove`,
an exact response identity/API match and remote status zero remain mandatory.

Missing, ambiguous, mismatched, timed-out, or non-zero required evidence is an
SDK failure to the daemon. A missing response is allowed only when the
independently observed request itself has the exact one-way policy. Failure
latches a fault, prohibits further non-zero commands, attempts Stop/Disarm,
and requires a new process and one-time permit before another motion trial. A
sport writer count/GID change or any positive-lease request writer appearing
during the probe has the same fail-closed result.

The raw RPC observer is subscription-only. It does not construct a raw request
publisher; actual calls remain confined to the official SDK2
`SportClient::Move` and `SportClient::StopMove` methods in the isolated daemon.
The ROS adapter never links SDK2.

## Bounded physical RPC evidence from 2026-07-24

The latest trial observed one exact API `1008`, lease-zero request and SDK
local-send result zero, but no response. The operator observed no chassis
movement, and no qualifying displacement was measured. Pinned SDK inspection
then proved that API `1008` uses `noreply=true`; the old guard had incorrectly
classified this expected absence as response loss. The daemon immediately
faulted and disarmed after the first ramp-limited request. This remains a
failed physical trial, not a motion success.

During the positive-lease implementation:

- SDK error `3104` was observed during lease renewal. The pinned SDK defines it
  as a client API timeout. This does not by itself identify the network or
  server root cause.
- The subsequent API `1003` StopMove returned `3205`, which the pinned SDK
  defines as request denied by lease. That stop was not acknowledged and cannot
  be described as executed.
- Later lease re-application and old-epoch observer mismatches can explain
  subsequent request-missing diagnostics, but cannot turn the earlier `3205`
  rejection into successful evidence.

No trial yet contains both a fully witnessed one-way Move emission,
response-verified Stop sequence, measured physical displacement, and measured
stopping.

## What this does not prove

A complete response-bearing RPC acknowledgement proves that the remote sport
service accepted that request. A one-way Move emission proves only exact local
DDS request publication. Neither alone proves mechanical displacement or
stopping; those require fresh `SportModeState`, odometry, post-stop observation,
and the operator's physical observation.

The DDS graph is eventually consistent and only covers discovered writers in
the active domain. It is a runtime isolation interlock, not authentication and
not proof that every phone, remote-control, firmware-internal, or other-domain
control path is absent. The paired official remote/e-stop and on-site operator
remain mandatory.

`SportModeState.error_code` is not used as an RPC acknowledgement. Motion state
eligibility remains a separate configured runtime gate.

## Verification boundary

- `protocol_guard_test` must cover positive-lease and exact-zero expectations,
  identity, API ID, ambiguity, remote status, SDK error, and failed arm
  preflight.
- `test_static_safety.py` must assert the observer is subscription-only, the
  primary SDK client is explicitly lease-disabled, and ARM does not manufacture
  a false prior-Move state.
- `test_first_motion_probe.py` must preserve separate soft-stop and hard
  envelopes, forward/lateral/yaw physical evidence, unique canonical odometry
  ownership, and the continuous sport-writer/lease-writer graph interlock.
- The isolated SDK daemon must build against the pinned x86_64 and arm64 SDK2
  checkout with Unitree examples disabled.
- Physical acceptance remains pending a new one-time permit and a measured,
  supervised low-speed trial.
