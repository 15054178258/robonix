from __future__ import annotations

from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.status_notifier import (
    LoopbackStatusNotifier,
    StatusEndpointError,
    validate_loopback_endpoint,
)


class StatusEndpointTest(unittest.TestCase):
    def test_only_literal_loopback_status_path_is_allowed(self) -> None:
        valid = (
            "http://127.0.0.1:8092/api/semantic-task",
            "http://[::1]:8092/api/semantic-task",
        )
        for endpoint in valid:
            with self.subTest(endpoint=endpoint):
                self.assertEqual(validate_loopback_endpoint(endpoint), endpoint)

        invalid = (
            "https://127.0.0.1:8092/api/semantic-task",
            "http://localhost:8092/api/semantic-task",
            "http://192.168.123.18:8092/api/semantic-task",
            "http://127.0.0.1:0/api/semantic-task",
            "http://127.0.0.1:8092/",
            "http://127.0.0.1:8092/api/semantic-task?next=http://example.com",
            "http://user:secret@127.0.0.1:8092/api/semantic-task",
        )
        for endpoint in invalid:
            with self.subTest(endpoint=endpoint):
                with self.assertRaises(StatusEndpointError):
                    validate_loopback_endpoint(endpoint)

    def test_empty_endpoint_disables_notifier(self) -> None:
        notifier = LoopbackStatusNotifier("")
        notifier.start()
        notifier.notify({"status": "received"})
        notifier.stop()
        self.assertFalse(notifier.enabled)

    def test_worker_preserves_fifo_and_is_failure_isolated(self) -> None:
        seen = []
        completed = threading.Event()

        def sender(endpoint: str, payload: dict, timeout_s: float) -> None:
            seen.append((endpoint, payload, timeout_s))
            if len(seen) == 1:
                raise OSError("dashboard unavailable")
            completed.set()

        notifier = LoopbackStatusNotifier(
            "http://127.0.0.1:8092/api/semantic-task",
            sender=sender,
        )
        notifier.start()
        notifier.notify({"status": "received"})
        notifier.notify({"status": "resolved"})
        self.assertTrue(completed.wait(1.0))
        notifier.stop()
        self.assertEqual([item[1]["status"] for item in seen], ["received", "resolved"])

    def test_service_covers_display_phases_and_terminal_mapping(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "semantic_navigation"
            / "service.py"
        ).read_text(encoding="utf-8")
        for status in ("received", "resolving", "resolved", "navigating"):
            self.assertIn(f'status="{status}"', source)
        for nav_state, display_state in (
            ("SUCCEEDED", "succeeded"),
            ("CANCELED", "canceled"),
            ("FAILED", "failed"),
        ):
            self.assertIn(f'"{nav_state}": "{display_state}"', source)

    def test_stop_drains_full_queue_and_joins_worker(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        def sender(_endpoint: str, _payload: dict, _timeout_s: float) -> None:
            entered.set()
            release.wait(0.5)

        notifier = LoopbackStatusNotifier(
            "http://127.0.0.1:8092/api/semantic-task",
            sender=sender,
        )
        notifier.start()
        notifier.notify({"status": "received"})
        self.assertTrue(entered.wait(1.0))
        for index in range(64):
            notifier.notify({"status": "navigating", "sequence": index})
        timer = threading.Timer(0.05, release.set)
        timer.start()
        notifier.stop(timeout_s=1.0)
        timer.join()
        self.assertFalse(notifier.is_alive)


if __name__ == "__main__":
    unittest.main()
