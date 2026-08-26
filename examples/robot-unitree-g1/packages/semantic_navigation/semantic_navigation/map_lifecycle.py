"""Fail-closed binding to the authoritative Robonix map lifecycle."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import uuid
from typing import Callable


log = logging.getLogger("semantic_navigation.map_lifecycle")

MAP_LIFECYCLE_CONTRACT_ID = "robonix/service/map/lifecycle"
MAP_LIFECYCLE_MESSAGE_TYPE = "map/msg/MapLifecycle"
MAP_LIFECYCLE_FIELDS = {
    "map_id": "string",
    "mode": "string",
    "generation": "uint64",
}
UINT64_MAX = (1 << 64) - 1


class LifecycleBindingError(RuntimeError):
    pass


def parse_generation(value: object, *, label: str = "generation") -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise LifecycleBindingError(f"{label} must be a YAML/integer uint64")
    if value < 0 or value > UINT64_MAX:
        raise LifecycleBindingError(f"{label} must be within [0, {UINT64_MAX}]")
    return value


@dataclass(frozen=True)
class MapLifecycleState:
    map_id: str
    mode: str
    generation: int

    @classmethod
    def from_message(cls, message: object) -> "MapLifecycleState":
        try:
            raw_map_id = getattr(message, "map_id")
            raw_mode = getattr(message, "mode")
            raw_generation = getattr(message, "generation")
        except AttributeError as exc:
            raise LifecycleBindingError(
                "MapLifecycle sample is missing map_id, mode, or generation"
            ) from exc

        map_id = raw_map_id.strip() if isinstance(raw_map_id, str) else ""
        if not map_id:
            raise LifecycleBindingError(
                "mapping broadcasts an ephemeral/empty map_id; saved landmarks are disabled"
            )
        if not isinstance(raw_mode, str):
            raise LifecycleBindingError("MapLifecycle mode must be a string")
        mode = raw_mode.strip().casefold()
        if mode not in {"mapping", "localization"}:
            raise LifecycleBindingError(f"unsupported MapLifecycle mode {raw_mode!r}")
        generation = parse_generation(raw_generation, label="MapLifecycle generation")
        return cls(map_id=map_id, mode=mode, generation=generation)


@dataclass(frozen=True)
class LifecycleSnapshot:
    state: MapLifecycleState | None
    error: str
    revision: int
    sample_seen: bool


@dataclass(frozen=True)
class LifecycleTransition:
    previous: MapLifecycleState | None
    current: MapLifecycleState | None
    error: str
    revision: int


def validate_contract_descriptor(descriptor: object) -> None:
    if descriptor is None:
        raise LifecycleBindingError(
            f"Atlas has no descriptor for {MAP_LIFECYCLE_CONTRACT_ID}"
        )
    if getattr(descriptor, "id", "") != MAP_LIFECYCLE_CONTRACT_ID:
        raise LifecycleBindingError("Atlas returned the wrong map lifecycle contract")
    if str(getattr(descriptor, "version", "")) != "1":
        raise LifecycleBindingError("map lifecycle contract version must be 1")
    if getattr(descriptor, "mode", "") != "topic_out":
        raise LifecycleBindingError("map lifecycle contract must be topic_out")
    if getattr(descriptor, "io_msg_type", "") != MAP_LIFECYCLE_MESSAGE_TYPE:
        raise LifecycleBindingError(
            f"map lifecycle message must be {MAP_LIFECYCLE_MESSAGE_TYPE}"
        )

    fields = {
        str(getattr(field, "name", "")): str(getattr(field, "type_name", ""))
        for field in (getattr(descriptor, "msg_fields", ()) or ())
    }
    if fields != MAP_LIFECYCLE_FIELDS:
        raise LifecycleBindingError(
            f"map lifecycle fields do not match the v1 contract: {fields!r}"
        )


class LifecycleGuard:
    def __init__(
        self,
        expected_map_id: str,
        expected_generation: int,
        on_transition: Callable[[LifecycleTransition], None] | None = None,
    ) -> None:
        expected_map_id = expected_map_id.strip()
        if not expected_map_id:
            raise LifecycleBindingError("expected map_id is empty")
        self.expected_map_id = expected_map_id
        self.expected_generation = parse_generation(
            expected_generation, label="expected map generation"
        )
        self._on_transition = on_transition
        self._lock = threading.RLock()
        self._sample_event = threading.Event()
        self._state: MapLifecycleState | None = None
        self._error = "no MapLifecycle sample has been received"
        self._revision = 0
        self._sample_seen = False

    def observe_message(self, message: object) -> LifecycleSnapshot:
        try:
            state = MapLifecycleState.from_message(message)
        except LifecycleBindingError as exc:
            return self.fail(str(exc), sample_seen=True)
        return self.observe(state)

    def observe(self, state: MapLifecycleState) -> LifecycleSnapshot:
        transition: LifecycleTransition | None = None
        with self._lock:
            previous = self._state
            changed = previous != state or bool(self._error)
            self._state = state
            self._error = ""
            self._sample_seen = True
            self._sample_event.set()
            if changed:
                self._revision += 1
                transition = LifecycleTransition(
                    previous=previous,
                    current=state,
                    error="",
                    revision=self._revision,
                )
            snapshot = self._snapshot_locked()
        self._notify(transition)
        return snapshot

    def fail(self, reason: str, *, sample_seen: bool = False) -> LifecycleSnapshot:
        reason = reason.strip() or "map lifecycle monitor failed"
        transition: LifecycleTransition | None = None
        with self._lock:
            previous = self._state
            changed = self._state is not None or self._error != reason
            self._state = None
            self._error = reason
            self._sample_seen = self._sample_seen or sample_seen
            if sample_seen:
                self._sample_event.set()
            if changed:
                self._revision += 1
                transition = LifecycleTransition(
                    previous=previous,
                    current=None,
                    error=reason,
                    revision=self._revision,
                )
            snapshot = self._snapshot_locked()
        self._notify(transition)
        return snapshot

    def wait_for_sample(self, timeout_s: float) -> bool:
        return self._sample_event.wait(timeout_s)

    def snapshot(self) -> LifecycleSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def require_ready(self) -> LifecycleSnapshot:
        snapshot = self.snapshot()
        if not snapshot.sample_seen:
            raise LifecycleBindingError("no MapLifecycle sample has been received")
        if snapshot.error:
            raise LifecycleBindingError(snapshot.error)
        state = snapshot.state
        if state is None:
            raise LifecycleBindingError("MapLifecycle state is unavailable")
        if state.mode != "localization":
            raise LifecycleBindingError(
                f"active map mode is {state.mode!r}; landmark navigation requires 'localization'"
            )
        if state.map_id != self.expected_map_id:
            raise LifecycleBindingError(
                f"active map_id {state.map_id!r} does not match landmark map_id "
                f"{self.expected_map_id!r}"
            )
        if state.generation != self.expected_generation:
            raise LifecycleBindingError(
                f"active map generation {state.generation} does not match landmark generation "
                f"{self.expected_generation}"
            )
        return snapshot

    def _snapshot_locked(self) -> LifecycleSnapshot:
        return LifecycleSnapshot(
            state=self._state,
            error=self._error,
            revision=self._revision,
            sample_seen=self._sample_seen,
        )

    def _notify(self, transition: LifecycleTransition | None) -> None:
        if transition is None or self._on_transition is None:
            return
        try:
            self._on_transition(transition)
        except Exception:
            log.exception("map lifecycle transition handler failed")


@dataclass(frozen=True)
class LifecycleAdmission:
    guard: LifecycleGuard
    subscriber: object
    topic: str
    snapshot: LifecycleSnapshot

    def validate_current(
        self,
        *,
        accepting: bool,
        current_guard: LifecycleGuard | None,
        current_subscriber: object | None,
        current_topic: str,
    ) -> LifecycleSnapshot:
        if not accepting:
            raise LifecycleBindingError("semantic navigation stopped accepting goals")
        if current_guard is not self.guard or current_subscriber is not self.subscriber:
            raise LifecycleBindingError("map lifecycle binding changed before dispatch")
        if current_topic != self.topic:
            raise LifecycleBindingError("map lifecycle topic changed before dispatch")
        if not bool(getattr(self.subscriber, "is_alive", False)):
            raise LifecycleBindingError("map lifecycle subscriber stopped before dispatch")
        latest = self.guard.require_ready()
        if latest.revision != self.snapshot.revision or latest.state != self.snapshot.state:
            raise LifecycleBindingError("map lifecycle state changed before dispatch")
        return latest


class RosMapLifecycleSubscriber:
    def __init__(self, topic: str, guard: LifecycleGuard) -> None:
        topic = topic.strip()
        if not topic:
            raise LifecycleBindingError("Atlas returned an empty map lifecycle topic")
        self.topic = topic
        self.guard = guard
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._context = None
        self._node = None
        self._executor = None
        self._subscription = None

    @property
    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self._thread is not None:
            raise LifecycleBindingError("map lifecycle subscriber was already started")
        try:
            import rclpy
            from map.msg import MapLifecycle
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )

            context = rclpy.Context()
            rclpy.init(context=context, args=None)
            node = rclpy.create_node(
                f"semantic_map_lifecycle_{uuid.uuid4().hex[:8]}", context=context
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            subscription = node.create_subscription(
                MapLifecycle, self.topic, self.guard.observe_message, qos
            )
        except Exception as exc:
            try:
                if "context" in locals():
                    rclpy.shutdown(context=context)
            except Exception:
                pass
            raise LifecycleBindingError(
                f"cannot create MapLifecycle subscription: {exc}"
            ) from exc

        self._context = context
        self._node = node
        self._executor = executor
        self._subscription = subscription
        self._thread = threading.Thread(
            target=self._spin,
            name="semantic-map-lifecycle",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, timeout_s))
            if thread.is_alive():
                self.guard.fail("map lifecycle subscriber did not stop cleanly")

    def _spin(self) -> None:
        try:
            while not self._stop.is_set():
                self._executor.spin_once(timeout_sec=0.2)
        except Exception as exc:
            self.guard.fail(f"map lifecycle subscriber stopped: {exc}")
        finally:
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
            try:
                self._node.destroy_node()
            except Exception:
                pass
            try:
                import rclpy

                rclpy.shutdown(context=self._context)
            except Exception:
                pass
