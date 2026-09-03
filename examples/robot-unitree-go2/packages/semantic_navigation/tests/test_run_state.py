from __future__ import annotations

from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.run_state import LifecycleBoundRunRegistry, SingleFlightError


class LifecycleBoundRunRegistryTest(unittest.TestCase):
    def reserve(self, registry: LifecycleBoundRunRegistry, run_id: str = "semantic-1") -> str:
        return registry.reserve(
            landmark_id="vending_machine_front",
            landmark_name="自动售货机",
            map_id="lab_go2",
            map_generation=7,
            lifecycle_revision=1,
            semantic_run_id=run_id,
        )

    def test_lifecycle_change_marks_and_claims_attached_unfinished_run(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)
        registry.attach_navigation(run_id, "nav-1", "queued")
        claimed = registry.invalidate_nonterminal("map generation changed")
        self.assertEqual(claimed, ((run_id, "nav-1", "map generation changed"),))
        run = registry.get(run_id)
        self.assertTrue(run.invalidated)
        self.assertEqual(run.state, "FAILED")
        self.assertTrue(run.cancel_started)

    def test_change_during_rpc_is_cancelled_when_nav_id_arrives(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)
        self.assertEqual(registry.invalidate_nonterminal("map switched"), ())
        run, must_cancel = registry.attach_navigation(run_id, "nav-late", "queued")
        self.assertTrue(must_cancel)
        self.assertTrue(run.invalidated)
        self.assertEqual(run.nav_run_id, "nav-late")

    def test_known_terminal_run_is_not_retroactively_invalidated(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)
        registry.attach_navigation(run_id, "nav-1", "queued")
        registry.update_remote(run_id, "SUCCEEDED", "arrived")
        self.assertEqual(registry.invalidate_nonterminal("map switched"), ())
        run = registry.get(run_id)
        self.assertEqual(run.state, "SUCCEEDED")
        self.assertFalse(run.invalidated)

    def test_shutdown_waits_for_late_navigation_id(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)

        def attach_later() -> None:
            time.sleep(0.03)
            registry.attach_navigation(run_id, "nav-late", "queued")

        thread = threading.Thread(target=attach_later)
        thread.start()
        self.assertTrue(registry.wait_for_dispatches(0.5))
        thread.join()
        self.assertEqual(registry.get(run_id).nav_run_id, "nav-late")

    def test_single_flight_rejects_starting_pending_and_running(self) -> None:
        for active_state in ("STARTING", "PENDING", "RUNNING"):
            registry = LifecycleBoundRunRegistry()
            run_id = self.reserve(registry)
            if active_state != "STARTING":
                registry.attach_navigation(run_id, "nav-1", "queued")
            if active_state == "RUNNING":
                registry.update_remote(run_id, "RUNNING", "executing")
            with self.subTest(active_state=active_state):
                with self.assertRaises(SingleFlightError):
                    self.reserve(registry, run_id="semantic-2")

    def test_only_confirmed_remote_terminal_releases_single_flight(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)
        registry.attach_navigation(run_id, "nav-1", "queued")
        registry.invalidate_nonterminal("map switched")

        # Local semantic FAILED and cancel submission are not proof that Nav2
        # stopped; a second goal remains blocked.
        registry.mark_cancel_result(run_id, "cancel accepted")
        self.assertEqual(registry.get(run_id).state, "FAILED")
        with self.assertRaises(SingleFlightError):
            self.reserve(registry, run_id="semantic-2")

        registry.update_remote(run_id, "RUNNING", "cancel not terminal")
        with self.assertRaises(SingleFlightError):
            self.reserve(registry, run_id="semantic-2")

        registry.update_remote(run_id, "CANCELED", "confirmed canceled")
        next_id = self.reserve(registry, run_id="semantic-2")
        self.assertEqual(next_id, "semantic-2")

    def test_concurrent_reservations_admit_exactly_one_goal(self) -> None:
        registry = LifecycleBoundRunRegistry()
        barrier = threading.Barrier(3)
        successes: list[str] = []
        failures: list[Exception] = []
        result_lock = threading.Lock()

        def reserve(run_id: str) -> None:
            barrier.wait()
            try:
                result = self.reserve(registry, run_id=run_id)
                with result_lock:
                    successes.append(result)
            except Exception as exc:  # noqa: BLE001 - asserted below
                with result_lock:
                    failures.append(exc)

        threads = [
            threading.Thread(target=reserve, args=("semantic-a",)),
            threading.Thread(target=reserve, args=("semantic-b",)),
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=1.0)

        self.assertEqual(len(successes), 1)
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(failures[0], SingleFlightError)

    def test_delayed_nonterminal_poll_cannot_erase_terminal_evidence(self) -> None:
        registry = LifecycleBoundRunRegistry()
        run_id = self.reserve(registry)
        registry.attach_navigation(run_id, "nav-1", "queued")
        registry.update_remote(run_id, "CANCELED", "provider confirmed canceled")

        run = registry.update_remote(run_id, "RUNNING", "stale delayed poll")

        self.assertTrue(run.remote_terminal)
        self.assertEqual(run.remote_state, "CANCELED")
        self.assertEqual(run.state, "CANCELED")
        self.assertEqual(run.navigation_detail, "provider confirmed canceled")


if __name__ == "__main__":
    unittest.main()
