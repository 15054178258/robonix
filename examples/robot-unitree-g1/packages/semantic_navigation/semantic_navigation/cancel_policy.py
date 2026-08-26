"""Bounded cancellation policy that requires a confirmed provider terminal."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


TERMINAL_STATES = frozenset({"SUCCEEDED", "CANCELED", "FAILED"})


@dataclass(frozen=True)
class CancelOutcome:
    terminal: bool
    terminal_state: str
    detail: str
    cancel_attempts: int
    cancel_was_accepted: bool
    timed_out: bool


def cancel_until_terminal(
    *,
    status_call: Callable[[], object],
    cancel_call: Callable[[], object],
    timeout_s: float,
    poll_s: float = 0.10,
    monotonic: Callable[[], float] = time.monotonic,
    wait: Callable[[float], None] = time.sleep,
) -> CancelOutcome:
    if timeout_s <= 0.0 or poll_s <= 0.0:
        raise ValueError("cancel timeout and poll interval must be positive")
    deadline = monotonic() + timeout_s
    attempts = 0
    accepted = False
    last_detail = "no status or cancel response"

    while True:
        try:
            status = status_call()
            if bool(getattr(status, "known", False)):
                state = str(getattr(status, "state", "")).strip().upper()
                status_detail = str(getattr(status, "detail", "") or "")
                if state in TERMINAL_STATES:
                    return CancelOutcome(
                        terminal=True,
                        terminal_state=state,
                        detail=status_detail or f"navigation confirmed {state}",
                        cancel_attempts=attempts,
                        cancel_was_accepted=accepted,
                        timed_out=False,
                    )
                last_detail = status_detail or f"navigation remains {state or 'UNKNOWN'}"
            else:
                last_detail = "navigation status is unknown/not registered"
        except Exception as exc:
            last_detail = f"navigation status failed: {exc}"

        try:
            response = cancel_call()
            attempts += 1
            response_detail = str(getattr(response, "detail", "") or "")
            if bool(getattr(response, "accepted", False)):
                accepted = True
                last_detail = response_detail or "cancel submitted; terminal not confirmed"
            else:
                last_detail = response_detail or "cancel request rejected"
        except Exception as exc:
            attempts += 1
            last_detail = f"navigation cancel failed: {exc}"

        remaining = deadline - monotonic()
        if remaining <= 0.0:
            return CancelOutcome(
                terminal=False,
                terminal_state="",
                detail=last_detail,
                cancel_attempts=attempts,
                cancel_was_accepted=accepted,
                timed_out=True,
            )
        wait(min(poll_s, remaining))
