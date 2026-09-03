"""Thread-safe semantic-run state with lifecycle invalidation semantics."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import uuid


TERMINAL_STATES = frozenset({"SUCCEEDED", "CANCELED", "FAILED"})


class SingleFlightError(RuntimeError):
    def __init__(self, active_run: "RunSnapshot") -> None:
        self.active_run = active_run
        nav_id = active_run.nav_run_id or "awaiting-provider-run-id"
        super().__init__(
            f"semantic navigation busy: run {active_run.semantic_run_id!r} "
            f"is local={active_run.state} remote={active_run.remote_state or 'UNKNOWN'} "
            f"nav_run_id={nav_id!r}; wait for confirmed remote terminal status"
        )


@dataclass(frozen=True)
class RunSnapshot:
    semantic_run_id: str
    nav_run_id: str
    landmark_id: str
    landmark_name: str
    map_id: str
    map_generation: int
    lifecycle_revision: int
    state: str
    navigation_detail: str
    invalidated: bool
    invalidation_detail: str
    cancel_started: bool
    cancel_detail: str
    remote_state: str
    remote_terminal: bool


@dataclass
class _Run:
    semantic_run_id: str
    nav_run_id: str
    landmark_id: str
    landmark_name: str
    map_id: str
    map_generation: int
    lifecycle_revision: int
    state: str = "STARTING"
    navigation_detail: str = ""
    invalidated: bool = False
    invalidation_detail: str = ""
    cancel_started: bool = False
    cancel_detail: str = ""
    remote_state: str = ""
    remote_terminal: bool = False

    def snapshot(self) -> RunSnapshot:
        return RunSnapshot(**vars(self))


class LifecycleBoundRunRegistry:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._runs: dict[str, _Run] = {}
        self._latest_run_id = ""

    def clear(self) -> None:
        with self._lock:
            self._runs.clear()
            self._latest_run_id = ""
            self._condition.notify_all()

    def reserve(
        self,
        *,
        landmark_id: str,
        landmark_name: str,
        map_id: str,
        map_generation: int,
        lifecycle_revision: int,
        semantic_run_id: str | None = None,
    ) -> str:
        run_id = semantic_run_id or str(uuid.uuid4())
        with self._lock:
            busy = next(
                (run for run in self._runs.values() if not run.remote_terminal),
                None,
            )
            if busy is not None:
                raise SingleFlightError(busy.snapshot())
            if run_id in self._runs:
                raise ValueError(f"duplicate semantic run_id {run_id!r}")
            self._runs[run_id] = _Run(
                semantic_run_id=run_id,
                nav_run_id="",
                landmark_id=landmark_id,
                landmark_name=landmark_name,
                map_id=map_id,
                map_generation=map_generation,
                lifecycle_revision=lifecycle_revision,
            )
            self._latest_run_id = run_id
        return run_id

    def attach_navigation(
        self, semantic_run_id: str, nav_run_id: str, navigation_detail: str
    ) -> tuple[RunSnapshot, bool]:
        if not nav_run_id:
            raise ValueError("nav_run_id is empty")
        with self._lock:
            run = self._require(semantic_run_id)
            run.nav_run_id = nav_run_id
            run.navigation_detail = navigation_detail
            run.remote_state = "PENDING"
            run.remote_terminal = False
            self._latest_run_id = semantic_run_id
            must_cancel = run.invalidated
            if must_cancel:
                run.cancel_started = True
            else:
                run.state = "PENDING"
            self._condition.notify_all()
            return run.snapshot(), must_cancel

    def remove(self, semantic_run_id: str) -> None:
        with self._lock:
            self._runs.pop(semantic_run_id, None)
            self._condition.notify_all()

    def wait_for_dispatches(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while any(not run.nav_run_id for run in self._runs.values()):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return True

    def resolve(self, semantic_run_id: str) -> str:
        with self._lock:
            resolved = semantic_run_id.strip() or self._latest_run_id
            if not resolved:
                raise KeyError("no semantic navigation run exists")
            if resolved not in self._runs:
                raise KeyError(f"unknown semantic navigation run_id {resolved!r}")
            return resolved

    def get(self, semantic_run_id: str) -> RunSnapshot:
        with self._lock:
            return self._require(semantic_run_id).snapshot()

    def update_remote(
        self, semantic_run_id: str, state: str, navigation_detail: str
    ) -> RunSnapshot:
        with self._lock:
            run = self._require(semantic_run_id)
            remote_state = state.strip().upper() or "FAILED"
            if run.remote_terminal:
                return run.snapshot()
            run.remote_state = remote_state
            run.remote_terminal = remote_state in TERMINAL_STATES
            run.navigation_detail = navigation_detail
            if not run.invalidated:
                run.state = remote_state
            return run.snapshot()

    def invalidate_nonterminal(
        self, reason: str
    ) -> tuple[tuple[str, str, str], ...]:
        claimed: list[tuple[str, str, str]] = []
        with self._lock:
            for run in self._runs.values():
                if run.remote_terminal:
                    continue
                run.invalidated = True
                run.invalidation_detail = reason
                run.state = "FAILED"
                if run.nav_run_id and not run.cancel_started:
                    run.cancel_started = True
                    claimed.append((run.semantic_run_id, run.nav_run_id, reason))
        return tuple(claimed)

    def claim_cancel(self, semantic_run_id: str) -> tuple[str, str] | None:
        with self._lock:
            run = self._require(semantic_run_id)
            if not run.nav_run_id or run.cancel_started:
                return None
            run.cancel_started = True
            return run.nav_run_id, run.invalidation_detail

    def mark_cancel_result(
        self,
        semantic_run_id: str,
        detail: str,
        *,
        cancel_in_progress: bool | None = None,
    ) -> RunSnapshot:
        with self._lock:
            run = self._require(semantic_run_id)
            run.cancel_detail = detail
            if cancel_in_progress is not None:
                run.cancel_started = cancel_in_progress
            return run.snapshot()

    def _require(self, semantic_run_id: str) -> _Run:
        run = self._runs.get(semantic_run_id)
        if run is None:
            raise KeyError(f"unknown semantic navigation run_id {semantic_run_id!r}")
        return run
