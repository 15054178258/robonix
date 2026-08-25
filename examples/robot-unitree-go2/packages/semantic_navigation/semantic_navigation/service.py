"""Robonix Skill: saved semantic name -> Robonix navigation service."""

from __future__ import annotations

import json
import logging
import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid


def _add_codegen_paths() -> None:
    package_root = Path(__file__).resolve().parent.parent
    for path in (
        package_root / "rbnx-build" / "codegen" / "proto_gen",
        package_root / "rbnx-build" / "codegen" / "robonix_mcp_types",
    ):
        if path.is_dir():
            sys.path.insert(0, str(path))


_add_codegen_paths()

import grpc  # noqa: E402
import navigation_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402
from semantic_navigation_mcp import (  # noqa: E402
    CancelLandmarkNavigation_Request,
    CancelLandmarkNavigation_Response,
    GetLandmarkNavigationStatus_Request,
    GetLandmarkNavigationStatus_Response,
    NavigateLandmark_Request,
    NavigateLandmark_Response,
)
from robonix_api import ATLAS, Err, Ok, Skill  # noqa: E402
from robonix_api.atlas_types import LifecycleState, Transport  # noqa: E402

from .cancel_policy import cancel_until_terminal  # noqa: E402
from .core import Landmark, LandmarkError, LandmarkStore  # noqa: E402
from .map_lifecycle import (  # noqa: E402
    MAP_LIFECYCLE_CONTRACT_ID,
    LifecycleAdmission,
    LifecycleBindingError,
    LifecycleGuard,
    LifecycleTransition,
    RosMapLifecycleSubscriber,
    validate_contract_descriptor,
)
from .run_state import (  # noqa: E402
    LifecycleBoundRunRegistry,
    RunSnapshot,
    SingleFlightError,
)
from .status_notifier import (  # noqa: E402
    LoopbackStatusNotifier,
    StatusEndpointError,
    validate_loopback_endpoint,
)


logging.basicConfig(
    level=os.environ.get("SEMANTIC_NAV_LOG_LEVEL", "INFO").upper(),
    format="[semantic_navigation] %(levelname)s %(message)s",
)
log = logging.getLogger("semantic_navigation")

skill = Skill(id="semantic_navigation", namespace="robonix/skill/semantic_navigation")

_lock = threading.RLock()
_store: LandmarkStore | None = None
_expected_map_id = ""
_navigation_provider_id = "nav2"
_mapping_provider_id = "mapping"
_require_verified = True
_rpc_timeout_s = 10.0
_lifecycle_wait_s = 10.0
_cancel_retry_s = 3.0
_lifecycle_guard: LifecycleGuard | None = None
_lifecycle_subscriber: RosMapLifecycleSubscriber | None = None
_lifecycle_channel = None
_lifecycle_topic = ""
_accepting_goals = False
_runs = LifecycleBoundRunRegistry()
_cancel_threads: set[threading.Thread] = set()
_status_endpoint = ""
_status_timeout_s = 0.5
_status_notifier: LoopbackStatusNotifier | None = None
_nav_status_poll_s = 0.5
_nav_monitor_stop = threading.Event()
_nav_monitor_threads: set[threading.Thread] = set()


def _as_bool(value: object, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().casefold()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value {value!r}")


def _resolve_config_path(value: object) -> Path:
    raw = str(value or "").strip()
    if not raw:
        raise LandmarkError("landmarks_file is required")
    path = Path(raw).expanduser()
    if path.is_absolute():
        return path
    deployment = os.environ.get("ROBONIX_DEPLOY_DIR") or os.environ.get("RBNX_INVOCATION_CWD")
    if not deployment:
        raise LandmarkError("relative landmarks_file requires ROBONIX_DEPLOY_DIR")
    return Path(deployment) / path


def _navigation_stub(contract_id: str, stub_name: str):
    caps = ATLAS.find_capability(
        contract_id=contract_id,
        transport=Transport.GRPC,
        provider_id=_navigation_provider_id,
    )
    if len(caps) != 1:
        raise RuntimeError(
            f"expected one {_navigation_provider_id!r} provider for {contract_id}, found {len(caps)}"
        )
    # These RPC channels are short-lived (status polling can run at 2 Hz), so
    # avoid Capability.connect_capability's lifetime channel list and close the
    # Atlas edge explicitly after every call.
    connection = ATLAS.connect_capability(
        consumer_id=skill.id,
        provider_id=caps[0].provider_id,
        contract_id=contract_id,
        transport=Transport.GRPC,
    )
    endpoint = (connection.endpoint or "").strip()
    if not endpoint:
        connection.close()
        raise RuntimeError(f"navigation endpoint for {contract_id} is empty")
    channel = grpc.insecure_channel(endpoint)
    stub = getattr(contracts_grpc, stub_name)(channel)
    return connection, channel, stub


def _call_navigate(target: Landmark):
    connection, channel, stub = _navigation_stub(
        "robonix/service/navigation/navigate",
        "RobonixServiceNavigationNavigateStub",
    )
    try:
        request = navigation_pb2.Navigate_Request()
        request.goal.header.frame_id = target.frame_id
        request.goal.pose.position.x = target.x
        request.goal.pose.position.y = target.y
        request.goal.pose.position.z = 0.0
        request.goal.pose.orientation.z = math.sin(target.yaw / 2.0)
        request.goal.pose.orientation.w = math.cos(target.yaw / 2.0)
        return stub.Navigate(request, timeout=_rpc_timeout_s)
    finally:
        channel.close()
        connection.close()


def _call_status(nav_run_id: str, *, timeout_s: float | None = None):
    connection, channel, stub = _navigation_stub(
        "robonix/service/navigation/navigate/status",
        "RobonixServiceNavigationNavigateStatusStub",
    )
    try:
        request = navigation_pb2.GetNavigationStatus_Request(run_id=nav_run_id)
        return stub.GetNavigationStatus(
            request,
            timeout=_rpc_timeout_s if timeout_s is None else timeout_s,
        )
    finally:
        channel.close()
        connection.close()


def _call_cancel(nav_run_id: str, *, timeout_s: float | None = None):
    connection, channel, stub = _navigation_stub(
        "robonix/service/navigation/navigate/cancel",
        "RobonixServiceNavigationNavigateCancelStub",
    )
    try:
        request = navigation_pb2.CancelNavigation_Request(run_id=nav_run_id)
        return stub.CancelNavigation(
            request,
            timeout=_rpc_timeout_s if timeout_s is None else timeout_s,
        )
    finally:
        channel.close()
        connection.close()


def _resolve_run_id(run_id: str) -> str:
    try:
        return _runs.resolve(run_id)
    except KeyError as exc:
        raise RuntimeError(str(exc).strip("'")) from exc


def _mapping_lifecycle_capability():
    providers = ATLAS.query_services(
        id=_mapping_provider_id,
        contract_id=MAP_LIFECYCLE_CONTRACT_ID,
        transport=Transport.ROS2,
    )
    if len(providers) != 1:
        raise LifecycleBindingError(
            f"expected one active {_mapping_provider_id!r} mapping provider for "
            f"{MAP_LIFECYCLE_CONTRACT_ID}, found {len(providers)}"
        )
    provider = providers[0]
    if provider.state != LifecycleState.ACTIVE:
        raise LifecycleBindingError(
            f"mapping provider {_mapping_provider_id!r} is {provider.state.name}, not ACTIVE"
        )
    capabilities = [
        cap
        for cap in provider.capabilities
        if cap.contract_id == MAP_LIFECYCLE_CONTRACT_ID
        and cap.transport == Transport.ROS2
    ]
    if len(capabilities) != 1:
        raise LifecycleBindingError(
            f"mapping provider exposes {len(capabilities)} ROS2 lifecycle capabilities"
        )
    return provider, capabilities[0]


def _assert_mapping_provider_live(expected_topic: str) -> None:
    provider, _capability = _mapping_lifecycle_capability()
    # Resolve through Atlas again before every goal.  The latched ROS sample
    # alone could survive a mapping provider crash/restart and is not liveness.
    channel = ATLAS.connect_capability(
        consumer_id=skill.id,
        provider_id=provider.id,
        contract_id=MAP_LIFECYCLE_CONTRACT_ID,
        transport=Transport.ROS2,
    )
    try:
        current_topic = (channel.endpoint or "").strip()
    finally:
        channel.close()
    if not current_topic:
        raise LifecycleBindingError("mapping lifecycle topic is empty")
    if current_topic != expected_topic:
        raise LifecycleBindingError(
            f"mapping lifecycle endpoint changed from {expected_topic!r} to {current_topic!r}"
        )


def _require_lifecycle_ready():
    with _lock:
        guard = _lifecycle_guard
        subscriber = _lifecycle_subscriber
        topic = _lifecycle_topic
        accepting = _accepting_goals
    if not accepting or guard is None or subscriber is None:
        raise LifecycleBindingError("semantic navigation is not active")
    if not subscriber.is_alive:
        guard.fail("map lifecycle subscriber is not running")
    snapshot = guard.require_ready()
    _assert_mapping_provider_live(topic)
    return LifecycleAdmission(
        guard=guard,
        subscriber=subscriber,
        topic=topic,
        snapshot=snapshot,
    )


def _transition_detail(transition: LifecycleTransition) -> str:
    previous = transition.previous
    current = transition.current
    old = (
        "none"
        if previous is None
        else f"{previous.map_id}/{previous.generation}/{previous.mode}"
    )
    new = (
        f"invalid ({transition.error})"
        if current is None
        else f"{current.map_id}/{current.generation}/{current.mode}"
    )
    return f"map lifecycle changed at revision {transition.revision}: {old} -> {new}"


def _on_lifecycle_transition(transition: LifecycleTransition) -> None:
    # The first sample establishes the binding before activation.  Every later
    # identity/mode transition invalidates any run that has not reached a
    # terminal state, even if the map subsequently switches back.
    if transition.previous is None and not transition.error:
        log.info("received initial map lifecycle: %s", _transition_detail(transition))
        return
    reason = _transition_detail(transition)
    log.error("%s; rejecting new semantic goals and cancelling unfinished runs", reason)
    # Serialize transition invalidation with the final admission+reserve block.
    # If the guard changes immediately after validate_current(), this callback
    # runs after the reserved STARTING goal becomes visible and claims it.
    with _lock:
        _invalidate_nonterminal_runs(reason)


def _invalidate_nonterminal_runs(reason: str) -> None:
    for semantic_run_id, nav_run_id, run_reason in _runs.invalidate_nonterminal(reason):
        run = _runs.get(semantic_run_id)
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=run.landmark_name,
            status="failed",
            message=run_reason,
        )
        _schedule_cancel(semantic_run_id, nav_run_id, run_reason)


def _notify_semantic_status(
    *,
    task_id: str,
    target_name: str,
    status: str,
    message: str,
    pose: dict | None = None,
) -> None:
    with _lock:
        notifier = _status_notifier
    if notifier is None:
        return
    payload = {
        "task_id": task_id,
        "target_name": target_name,
        "status": status,
        "message": message,
        "pose": pose,
    }
    notifier.notify(payload)


def _start_status_notifier() -> None:
    global _status_notifier
    notifier = LoopbackStatusNotifier(_status_endpoint, _status_timeout_s)
    notifier.start()
    with _lock:
        _status_notifier = notifier


def _stop_status_notifier() -> None:
    global _status_notifier
    with _lock:
        notifier = _status_notifier
        _status_notifier = None
    if notifier is not None:
        notifier.stop()


def _start_navigation_monitors() -> None:
    global _nav_monitor_stop
    with _lock:
        _nav_monitor_stop = threading.Event()


def _stop_navigation_monitors() -> None:
    with _lock:
        stop_event = _nav_monitor_stop
        threads = tuple(_nav_monitor_threads)
    stop_event.set()
    deadline = time.monotonic() + 1.5
    for thread in threads:
        if thread is not threading.current_thread():
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
            if thread.is_alive():
                log.error("navigation status monitor did not stop cleanly: %s", thread.name)


def _schedule_navigation_monitor(semantic_run_id: str) -> None:
    with _lock:
        stop_event = _nav_monitor_stop

    def worker() -> None:
        try:
            _monitor_navigation_status(semantic_run_id, stop_event)
        finally:
            with _lock:
                _nav_monitor_threads.discard(threading.current_thread())

    thread = threading.Thread(
        target=worker,
        name=f"semantic-status-{semantic_run_id[:8]}",
        daemon=True,
    )
    with _lock:
        _nav_monitor_threads.add(thread)
    thread.start()


def _monitor_navigation_status(
    semantic_run_id: str, stop_event: threading.Event
) -> None:
    last_reported_state: tuple[str, str, bool] | None = None
    while not stop_event.is_set():
        try:
            run = _runs.get(semantic_run_id)
        except KeyError:
            return
        if run.remote_terminal:
            return
        try:
            response = _call_status(
                run.nav_run_id,
                timeout_s=min(_rpc_timeout_s, 0.5),
            )
            if not response.known:
                run = _runs.update_remote(
                    semantic_run_id,
                    "UNKNOWN",
                    f"navigation provider does not currently know run {run.nav_run_id!r}; "
                    "terminal state is unconfirmed",
                )
            else:
                run = _runs.update_remote(
                    semantic_run_id,
                    response.state,
                    response.detail,
                )
            report_key = (run.state, run.remote_state, run.remote_terminal)
            if report_key != last_reported_state:
                _notify_semantic_status(
                    task_id=semantic_run_id,
                    target_name=run.landmark_name,
                    status=_dashboard_status(run.state),
                    message=_run_status_message(run),
                )
                last_reported_state = report_key
            if run.remote_terminal:
                return
        except Exception as exc:  # noqa: BLE001 - observation cannot affect navigation
            log.warning("status monitor for %s failed: %s", semantic_run_id, exc)
        stop_event.wait(_nav_status_poll_s)


def _schedule_cancel(semantic_run_id: str, nav_run_id: str, reason: str) -> None:
    def worker() -> None:
        try:
            _cancel_run_until_terminal(semantic_run_id, nav_run_id, reason)
        finally:
            with _lock:
                _cancel_threads.discard(threading.current_thread())

    thread = threading.Thread(
        target=worker,
        name=f"semantic-cancel-{semantic_run_id[:8]}",
        daemon=True,
    )
    with _lock:
        _cancel_threads.add(thread)
    thread.start()


def _wait_cancel_workers(timeout_s: float) -> None:
    with _lock:
        threads = tuple(_cancel_threads)
    deadline = time.monotonic() + max(0.0, timeout_s)
    for thread in threads:
        if thread is threading.current_thread():
            continue
        thread.join(timeout=max(0.0, deadline - time.monotonic()))
        if thread.is_alive():
            log.error("lifecycle cancellation worker did not stop: %s", thread.name)


def _wait_inflight_dispatches() -> None:
    # A lifecycle transition can race the Navigate RPC before it returns a
    # provider run_id. Wait through that bounded RPC window so attach_navigation
    # can claim and cancel the late id instead of letting shutdown orphan it.
    if not _runs.wait_for_dispatches(_rpc_timeout_s + 0.5):
        log.error("timed out waiting for in-flight navigation dispatches to return")


def _cancel_run_until_terminal(
    semantic_run_id: str, nav_run_id: str, reason: str
) -> None:
    outcome = cancel_until_terminal(
        status_call=lambda: _call_status(
            nav_run_id, timeout_s=min(_rpc_timeout_s, 0.5)
        ),
        cancel_call=lambda: _call_cancel(
            nav_run_id, timeout_s=min(_rpc_timeout_s, 0.5)
        ),
        timeout_s=_cancel_retry_s,
        poll_s=0.10,
    )
    try:
        current = _runs.get(semantic_run_id)
        if outcome.terminal or current.remote_terminal:
            confirmed = (
                _runs.update_remote(
                    semantic_run_id,
                    outcome.terminal_state,
                    outcome.detail,
                )
                if outcome.terminal
                else current
            )
            result_detail = (
                f"{reason}; provider terminal confirmed as {confirmed.remote_state}; "
                f"cancel_attempts={outcome.cancel_attempts}; "
                f"cancel_accepted={outcome.cancel_was_accepted}; "
                f"{confirmed.navigation_detail or outcome.detail}"
            )
            cancel_in_progress = True
            terminal_confirmed = True
        else:
            result_detail = (
                f"HIGH-RISK: {reason}; provider terminal state was not confirmed "
                f"within {_cancel_retry_s:.1f}s; semantic single-flight remains busy; "
                f"cancel_attempts={outcome.cancel_attempts}; "
                f"cancel_accepted={outcome.cancel_was_accepted}; {outcome.detail}"
            )
            # The safety latch remains the unconfirmed remote_terminal=False.
            # Re-open only the cancellation claim so an operator may request
            # another bounded retry window without admitting a second goal.
            cancel_in_progress = False
            terminal_confirmed = False
        run = _runs.mark_cancel_result(
            semantic_run_id,
            result_detail,
            cancel_in_progress=cancel_in_progress,
        )
    except KeyError:
        return
    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=run.landmark_name,
        status=(
            _dashboard_status(run.state)
            if terminal_confirmed
            else "failed"
        ),
        message=run.cancel_detail,
    )
    if outcome.timed_out and not terminal_confirmed:
        log.error("run %s: %s", semantic_run_id, result_detail)
    else:
        log.warning("run %s cancellation completed: %s", semantic_run_id, result_detail)


def _stop_lifecycle_binding(reason: str) -> None:
    global _lifecycle_guard, _lifecycle_subscriber, _lifecycle_channel
    global _lifecycle_topic, _accepting_goals
    with _lock:
        _accepting_goals = False
        subscriber = _lifecycle_subscriber
        channel = _lifecycle_channel
        _lifecycle_subscriber = None
        _lifecycle_channel = None
        _lifecycle_guard = None
        _lifecycle_topic = ""
    _invalidate_nonterminal_runs(reason)
    if subscriber is not None:
        subscriber.stop()
    if channel is not None:
        channel.close()


@skill.on_init
def init(config: dict):
    global _store, _expected_map_id, _navigation_provider_id, _mapping_provider_id
    global _require_verified, _rpc_timeout_s, _lifecycle_wait_s, _cancel_retry_s
    global _status_endpoint, _status_timeout_s, _nav_status_poll_s
    try:
        path = _resolve_config_path(config.get("landmarks_file"))
        store = LandmarkStore.from_path(path)
        expected_map_id = str(config.get("expected_map_id", "")).strip()
        if not expected_map_id:
            raise LandmarkError("expected_map_id is required")
        if store.map_id != expected_map_id:
            raise LandmarkError(
                f"landmark map_id {store.map_id!r} does not match configured map_id "
                f"{expected_map_id!r}"
            )
        require_verified = _as_bool(config.get("require_verified"), True)
        rpc_timeout_s = float(config.get("rpc_timeout_s", 10.0))
        if not math.isfinite(rpc_timeout_s) or not 0.1 <= rpc_timeout_s <= 60.0:
            raise LandmarkError("rpc_timeout_s must be within [0.1, 60]")
        lifecycle_wait_s = float(config.get("lifecycle_wait_s", 10.0))
        if not math.isfinite(lifecycle_wait_s) or not 0.1 <= lifecycle_wait_s <= 60.0:
            raise LandmarkError("lifecycle_wait_s must be within [0.1, 60]")
        cancel_retry_s = float(config.get("cancel_retry_s", 3.0))
        if not math.isfinite(cancel_retry_s) or not 0.1 <= cancel_retry_s <= 15.0:
            raise LandmarkError("cancel_retry_s must be within [0.1, 15]")
        status_endpoint = validate_loopback_endpoint(config.get("status_endpoint", ""))
        status_timeout_s = float(config.get("status_timeout_s", 0.5))
        # Constructor performs the same bound check used at activation without
        # opening a thread or socket.
        LoopbackStatusNotifier(status_endpoint, status_timeout_s)
        nav_status_poll_s = float(config.get("nav_status_poll_s", 0.5))
        if not math.isfinite(nav_status_poll_s) or not 0.1 <= nav_status_poll_s <= 5.0:
            raise LandmarkError("nav_status_poll_s must be within [0.1, 5]")
    except (LandmarkError, StatusEndpointError, TypeError, ValueError) as exc:
        return Err(str(exc))
    with _lock:
        _store = store
        _expected_map_id = expected_map_id
        _navigation_provider_id = str(config.get("navigation_provider_id", "nav2")).strip() or "nav2"
        _mapping_provider_id = str(config.get("mapping_provider_id", "mapping")).strip() or "mapping"
        _require_verified = require_verified
        _rpc_timeout_s = rpc_timeout_s
        _lifecycle_wait_s = lifecycle_wait_s
        _cancel_retry_s = cancel_retry_s
        _status_endpoint = status_endpoint
        _status_timeout_s = status_timeout_s
        _nav_status_poll_s = nav_status_poll_s
    log.info(
        "loaded %d landmarks for map=%s generation=%d from %s (require_verified=%s)",
        len(store.landmarks), store.map_id, store.map_generation, path, require_verified,
    )
    return Ok()


@skill.on_activate
def activate():
    global _lifecycle_guard, _lifecycle_subscriber, _lifecycle_channel
    global _lifecycle_topic, _accepting_goals
    store = _store
    if store is None:
        return Err("semantic landmark store is not initialized")
    try:
        _start_status_notifier()
        _start_navigation_monitors()
        validate_contract_descriptor(ATLAS.query_contract(MAP_LIFECYCLE_CONTRACT_ID))
        _provider, capability = _mapping_lifecycle_capability()
        channel = ATLAS.connect_capability(
            consumer_id=skill.id,
            provider_id=capability.provider_id,
            contract_id=MAP_LIFECYCLE_CONTRACT_ID,
            transport=Transport.ROS2,
        )
        topic = (channel.endpoint or "").strip()
        if not topic:
            channel.close()
            raise LifecycleBindingError("Atlas returned an empty map lifecycle topic")
        guard = LifecycleGuard(
            store.map_id,
            store.map_generation,
            on_transition=_on_lifecycle_transition,
        )
        subscriber = RosMapLifecycleSubscriber(topic, guard)
        with _lock:
            _lifecycle_guard = guard
            _lifecycle_subscriber = subscriber
            _lifecycle_channel = channel
            _lifecycle_topic = topic
            _accepting_goals = False
        subscriber.start()
        if not guard.wait_for_sample(_lifecycle_wait_s):
            raise LifecycleBindingError(
                f"no latched MapLifecycle sample received within {_lifecycle_wait_s:.1f}s"
            )
        snapshot = guard.require_ready()
        _assert_mapping_provider_live(topic)
        with _lock:
            _accepting_goals = True
        state = snapshot.state
        log.info(
            "semantic navigation bound to %s generation=%d mode=%s topic=%s",
            state.map_id,
            state.generation,
            state.mode,
            topic,
        )
        return Ok()
    except Exception as exc:  # noqa: BLE001 - lifecycle activation must fail closed
        _stop_lifecycle_binding(f"activation failed: {exc}")
        _wait_inflight_dispatches()
        _wait_cancel_workers(_cancel_retry_s + 1.5)
        _stop_navigation_monitors()
        _stop_status_notifier()
        return Err(str(exc))


@skill.on_deactivate
def deactivate():
    _stop_lifecycle_binding("semantic navigation deactivated")
    _wait_inflight_dispatches()
    _wait_cancel_workers(_cancel_retry_s + 1.5)
    _stop_navigation_monitors()
    _stop_status_notifier()
    return Ok()


@skill.on_shutdown
def shutdown():
    _stop_lifecycle_binding("semantic navigation shutting down")
    _wait_inflight_dispatches()
    _wait_cancel_workers(_cancel_retry_s + 1.5)
    _stop_navigation_monitors()
    _stop_status_notifier()
    return Ok()


@skill.mcp("robonix/skill/semantic_navigation/navigate_landmark")
def navigate_landmark(req: NavigateLandmark_Request) -> NavigateLandmark_Response:
    """Navigate to one verified landmark from the active saved map. For the Chinese request ‘走到前面自动售货机那里’, pass name='自动售货机' (the full utterance is also accepted). Never use this tool for an unknown object or arbitrary coordinates."""
    semantic_run_id = str(uuid.uuid4())
    requested_name = str(req.name).strip()
    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=requested_name,
        status="received",
        message="已收到语义目标，等待地图生命周期校验",
    )
    store = _store
    if store is None:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=requested_name,
            status="failed",
            message="语义地标未初始化",
        )
        raise RuntimeError("semantic landmark store is not initialized")
    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=requested_name,
        status="resolving",
        message="正在校验 map_id、generation、localization mode 和保存的 Pose",
    )
    try:
        admission = _require_lifecycle_ready()
        active = admission.snapshot.state
        target = store.resolve(
            req.name,
            expected_map_id=active.map_id,
            expected_generation=active.generation,
            require_verified=_require_verified,
        )
    except (LandmarkError, LifecycleBindingError) as exc:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=requested_name,
            status="failed",
            message=str(exc),
        )
        raise RuntimeError(str(exc)) from exc

    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=target.name,
        status="resolved",
        message=(
            f"已匹配保存的地图 Pose；map={active.map_id} generation={active.generation}"
        ),
        pose={
            "frame_id": target.frame_id,
            "x": target.x,
            "y": target.y,
            "yaw": target.yaw,
        },
    )
    try:
        # This lock is shared with _stop_lifecycle_binding(). Shutdown either
        # flips accepting=false before this check (no reserve/no Navigate), or
        # observes the reserved STARTING run and waits for its provider run_id.
        with _lock:
            admission.validate_current(
                accepting=_accepting_goals,
                current_guard=_lifecycle_guard,
                current_subscriber=_lifecycle_subscriber,
                current_topic=_lifecycle_topic,
            )
            _runs.reserve(
                landmark_id=target.id,
                landmark_name=target.name,
                map_id=active.map_id,
                map_generation=active.generation,
                lifecycle_revision=admission.snapshot.revision,
                semantic_run_id=semantic_run_id,
            )
    except (LifecycleBindingError, SingleFlightError) as exc:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=target.name,
            status="failed",
            message=str(exc),
        )
        raise RuntimeError(str(exc)) from exc
    try:
        response = _call_navigate(target)
    except grpc.RpcError as exc:
        detail = (
            f"Robonix navigation RPC failed ambiguously: "
            f"{exc.code().name}: {exc.details()}; provider terminal is unconfirmed, "
            "so semantic single-flight remains busy"
        )
        _runs.update_remote(semantic_run_id, "UNKNOWN", detail)
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=target.name,
            status="failed",
            message=detail,
        )
        raise RuntimeError(detail) from exc
    except Exception as exc:
        detail = (
            f"Robonix navigation call failed ambiguously: {exc}; provider terminal "
            "is unconfirmed, so semantic single-flight remains busy"
        )
        _runs.update_remote(semantic_run_id, "UNKNOWN", detail)
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=target.name,
            status="failed",
            message=detail,
        )
        raise RuntimeError(detail) from exc
    if not response.accepted:
        _runs.remove(semantic_run_id)
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=target.name,
            status="failed",
            message=f"navigation rejected: {response.detail}",
        )
        raise RuntimeError(f"navigation rejected landmark {target.name!r}: {response.detail}")
    if not response.run_id:
        detail = (
            "navigation reported accepted but returned no run_id; provider terminal "
            "is unconfirmed, so semantic single-flight remains busy"
        )
        _runs.update_remote(semantic_run_id, "UNKNOWN", detail)
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=target.name,
            status="failed",
            message=detail,
        )
        raise RuntimeError(detail)

    run, must_cancel = _runs.attach_navigation(
        semantic_run_id, response.run_id, response.detail
    )
    if must_cancel:
        _schedule_cancel(
            semantic_run_id,
            response.run_id,
            run.invalidation_detail or "map lifecycle changed while goal was starting",
        )
    else:
        try:
            latest = _require_lifecycle_ready()
            if latest.snapshot.revision != admission.snapshot.revision:
                raise LifecycleBindingError(
                    "map lifecycle changed while the navigation goal was being accepted"
                )
        except LifecycleBindingError as exc:
            _invalidate_nonterminal_runs(str(exc))
    run = _runs.get(semantic_run_id)
    if run.invalidated:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=run.landmark_name,
            status="failed",
            message=run.invalidation_detail,
        )
    else:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=run.landmark_name,
            status="navigating",
            message=f"Robonix navigation run {run.nav_run_id} 已开始",
            pose={
                "frame_id": target.frame_id,
                "x": target.x,
                "y": target.y,
                "yaw": target.yaw,
            },
        )
    # Monitor both normal and lifecycle-invalidated goals. Cancellation
    # submission is not terminal evidence; only provider status can release the
    # semantic single-flight latch.
    _schedule_navigation_monitor(semantic_run_id)
    return NavigateLandmark_Response(
        accepted=not run.invalidated,
        run_id=semantic_run_id,
        detail=_run_detail(run),
    )


def _run_detail(run: RunSnapshot) -> str:
    return json.dumps(
        {
            # Echo the semantic id inside every navigate/status detail so a
            # bounded Pilot history can recover the active run after its first
            # navigate leaf has been compacted away.
            "semantic_run_id": run.semantic_run_id,
            "state": run.state,
            "landmark": run.landmark_name,
            "map_id": run.map_id,
            "map_generation": run.map_generation,
            "lifecycle_revision": run.lifecycle_revision,
            "navigation_run_id": run.nav_run_id,
            "navigation_detail": run.navigation_detail,
            "lifecycle_invalidated": run.invalidated,
            "invalidation_detail": run.invalidation_detail,
            "cancel_detail": run.cancel_detail,
            "remote_state": run.remote_state,
            "remote_terminal": run.remote_terminal,
            "single_flight_busy": not run.remote_terminal,
        },
        ensure_ascii=False,
    )


def _dashboard_status(state: str) -> str:
    return {
        "SUCCEEDED": "succeeded",
        "CANCELED": "canceled",
        "FAILED": "failed",
    }.get(state.strip().upper(), "navigating")


def _run_status_message(run: RunSnapshot) -> str:
    if run.invalidated:
        remote = run.remote_state or "UNKNOWN"
        terminal = "confirmed terminal" if run.remote_terminal else "not terminal"
        return (
            f"{run.cancel_detail or run.invalidation_detail}; "
            f"provider={remote} ({terminal})"
        )
    return run.navigation_detail or run.state


@skill.mcp("robonix/skill/semantic_navigation/navigate_landmark/status")
def landmark_status(
    req: GetLandmarkNavigationStatus_Request,
) -> GetLandmarkNavigationStatus_Response:
    """Read the Nav2-backed state for a semantic navigation run."""
    semantic_run_id = _resolve_run_id(req.run_id)
    run = _runs.get(semantic_run_id)
    if run.remote_terminal:
        _notify_semantic_status(
            task_id=semantic_run_id,
            target_name=run.landmark_name,
            status=_dashboard_status(run.state),
            message=_run_status_message(run),
        )
        return GetLandmarkNavigationStatus_Response(
            known=True,
            state=run.state,
            detail=_run_detail(run),
        )
    try:
        response = _call_status(run.nav_run_id)
    except grpc.RpcError as exc:
        raise RuntimeError(
            f"Robonix navigation status RPC failed: {exc.code().name}: {exc.details()}"
        ) from exc
    if not response.known:
        _runs.update_remote(
            semantic_run_id,
            "UNKNOWN",
            f"navigation provider does not currently know run {run.nav_run_id!r}; "
            "terminal state is unconfirmed",
        )
        raise RuntimeError(f"navigation provider lost run {run.nav_run_id!r}")
    run = _runs.update_remote(
        semantic_run_id,
        response.state,
        response.detail,
    )
    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=run.landmark_name,
        status=_dashboard_status(run.state),
        message=_run_status_message(run),
    )
    return GetLandmarkNavigationStatus_Response(
        known=True,
        state=run.state,
        detail=_run_detail(run),
    )


@skill.mcp("robonix/skill/semantic_navigation/navigate_landmark/cancel")
def landmark_cancel(
    req: CancelLandmarkNavigation_Request,
) -> CancelLandmarkNavigation_Response:
    """Cancel the active semantic goal through the Robonix navigation service."""
    semantic_run_id = _resolve_run_id(req.run_id)
    run = _runs.get(semantic_run_id)
    if not run.nav_run_id:
        raise RuntimeError("semantic run has no navigation run_id; cancel fails closed")
    if run.remote_terminal:
        if run.remote_state == "CANCELED":
            return CancelLandmarkNavigation_Response(
                accepted=True, detail=run.navigation_detail or "already canceled"
            )
        raise RuntimeError(
            f"cannot cancel provider-terminal semantic run in state {run.remote_state}"
        )
    if run.cancel_started:
        detail = run.cancel_detail or (
            "cancellation is already in progress; provider terminal status has "
            "not yet been confirmed"
        )
        return CancelLandmarkNavigation_Response(accepted=True, detail=detail)
    claim = _runs.claim_cancel(semantic_run_id)
    if claim is None:
        raise RuntimeError("unable to claim semantic cancellation; retry status first")
    try:
        response = _call_cancel(run.nav_run_id)
    except grpc.RpcError as exc:
        _schedule_cancel(
            semantic_run_id,
            run.nav_run_id,
            f"user requested cancellation; initial cancel RPC failed: {exc.code().name}",
        )
        raise RuntimeError(
            f"Robonix navigation cancel RPC failed: {exc.code().name}: {exc.details()}"
        ) from exc
    except Exception as exc:
        _schedule_cancel(
            semantic_run_id,
            run.nav_run_id,
            f"user requested cancellation; initial cancel call failed: {exc}",
        )
        raise RuntimeError(f"Robonix navigation cancel call failed: {exc}") from exc
    if not response.accepted:
        _schedule_cancel(
            semantic_run_id,
            run.nav_run_id,
            "user requested cancellation; initial cancel request was rejected",
        )
        raise RuntimeError(f"navigation cancel rejected: {response.detail}")
    run = _runs.mark_cancel_result(
        semantic_run_id,
        (response.detail or "cancel submitted")
        + "; waiting for provider terminal confirmation",
    )
    _schedule_cancel(
        semantic_run_id,
        run.nav_run_id,
        "user requested cancellation",
    )
    _notify_semantic_status(
        task_id=semantic_run_id,
        target_name=run.landmark_name,
        status="navigating",
        message=run.cancel_detail,
    )
    return CancelLandmarkNavigation_Response(accepted=True, detail=run.cancel_detail)


def main() -> int:
    skill.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
