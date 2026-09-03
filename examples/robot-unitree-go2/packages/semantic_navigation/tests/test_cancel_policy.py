from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.cancel_policy import cancel_until_terminal


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def wait(self, duration: float) -> None:
        self.now += duration


class CancelPolicyTest(unittest.TestCase):
    def test_accepted_cancel_keeps_polling_and_retries_until_canceled(self) -> None:
        clock = FakeClock()
        statuses = iter(
            (
                SimpleNamespace(known=True, state="RUNNING", detail="executing"),
                SimpleNamespace(known=True, state="RUNNING", detail="cancel pending"),
                SimpleNamespace(known=True, state="CANCELED", detail="goal canceled"),
            )
        )
        cancels = iter(
            (
                SimpleNamespace(accepted=True, detail="cancel submitted"),
                SimpleNamespace(accepted=True, detail="cancel resubmitted"),
            )
        )

        outcome = cancel_until_terminal(
            status_call=lambda: next(statuses),
            cancel_call=lambda: next(cancels),
            timeout_s=1.0,
            poll_s=0.1,
            monotonic=clock.monotonic,
            wait=clock.wait,
        )

        self.assertTrue(outcome.terminal)
        self.assertEqual(outcome.terminal_state, "CANCELED")
        self.assertEqual(outcome.cancel_attempts, 2)
        self.assertTrue(outcome.cancel_was_accepted)
        self.assertFalse(outcome.timed_out)

    def test_never_terminal_times_out_without_releasing_safety_latch(self) -> None:
        clock = FakeClock()
        calls = {"status": 0, "cancel": 0}

        def status_call():
            calls["status"] += 1
            return SimpleNamespace(known=True, state="RUNNING", detail="still moving")

        def cancel_call():
            calls["cancel"] += 1
            return SimpleNamespace(accepted=True, detail="submitted asynchronously")

        outcome = cancel_until_terminal(
            status_call=status_call,
            cancel_call=cancel_call,
            timeout_s=0.25,
            poll_s=0.1,
            monotonic=clock.monotonic,
            wait=clock.wait,
        )

        self.assertFalse(outcome.terminal)
        self.assertEqual(outcome.terminal_state, "")
        self.assertTrue(outcome.cancel_was_accepted)
        self.assertTrue(outcome.timed_out)
        self.assertGreaterEqual(calls["cancel"], 2)
        self.assertEqual(calls["status"], calls["cancel"])


if __name__ == "__main__":
    unittest.main()
