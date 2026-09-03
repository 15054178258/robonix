from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from semantic_navigation.map_lifecycle import (
    LifecycleAdmission,
    LifecycleBindingError,
    LifecycleGuard,
    MapLifecycleState,
    validate_contract_descriptor,
)
from semantic_navigation.run_state import LifecycleBoundRunRegistry


def message(map_id: object = "lab_go2", mode: object = "localization", generation: object = 7):
    return SimpleNamespace(map_id=map_id, mode=mode, generation=generation)


def descriptor(**overrides):
    values = {
        "id": "robonix/service/map/lifecycle",
        "version": "1",
        "mode": "topic_out",
        "io_msg_type": "map/msg/MapLifecycle",
        "msg_fields": (
            SimpleNamespace(name="map_id", type_name="string"),
            SimpleNamespace(name="mode", type_name="string"),
            SimpleNamespace(name="generation", type_name="uint64"),
        ),
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class MapLifecycleContractTest(unittest.TestCase):
    def test_real_v1_descriptor_shape_is_required(self) -> None:
        validate_contract_descriptor(descriptor())
        for override in (
            {"version": "2"},
            {"mode": "rpc"},
            {"io_msg_type": "std_msgs/msg/String"},
            {"msg_fields": ()},
        ):
            with self.subTest(override=override):
                with self.assertRaises(LifecycleBindingError):
                    validate_contract_descriptor(descriptor(**override))

    def test_message_parser_rejects_ephemeral_or_malformed_samples(self) -> None:
        invalid = (
            message(map_id=""),
            message(mode="paused"),
            message(generation=True),
            message(generation="7"),
            message(generation=-1),
            SimpleNamespace(map_id="lab_go2", mode="localization"),
        )
        for sample in invalid:
            with self.subTest(sample=sample):
                with self.assertRaises(LifecycleBindingError):
                    MapLifecycleState.from_message(sample)


class LifecycleGuardTest(unittest.TestCase):
    def test_no_sample_fails_closed(self) -> None:
        guard = LifecycleGuard("lab_go2", 7)
        with self.assertRaisesRegex(LifecycleBindingError, "no MapLifecycle"):
            guard.require_ready()

    def test_matching_localization_sample_is_ready(self) -> None:
        guard = LifecycleGuard("lab_go2", 7)
        guard.observe_message(message())
        snapshot = guard.require_ready()
        self.assertTrue(snapshot.sample_seen)
        self.assertEqual(snapshot.state.map_id, "lab_go2")
        self.assertEqual(snapshot.state.generation, 7)

    def test_map_id_generation_and_mode_each_fail_closed(self) -> None:
        samples = (
            message(map_id="other"),
            message(generation=8),
            message(mode="mapping"),
        )
        for sample in samples:
            guard = LifecycleGuard("lab_go2", 7)
            guard.observe_message(sample)
            with self.subTest(sample=sample):
                with self.assertRaises(LifecycleBindingError):
                    guard.require_ready()

    def test_change_notifies_once_and_malformed_sample_invalidates(self) -> None:
        transitions = []
        guard = LifecycleGuard("lab_go2", 7, transitions.append)
        guard.observe_message(message())
        guard.observe_message(message())
        guard.observe_message(message(mode="mapping"))
        self.assertEqual(len(transitions), 2)
        self.assertEqual(transitions[-1].previous.mode, "localization")
        self.assertEqual(transitions[-1].current.mode, "mapping")

        guard.observe_message(message(map_id=""))
        self.assertEqual(len(transitions), 3)
        self.assertIsNone(transitions[-1].current)
        with self.assertRaisesRegex(LifecycleBindingError, "ephemeral"):
            guard.require_ready()

    def test_shutdown_between_first_check_and_reserve_rejects_dispatch(self) -> None:
        """Deterministically exercise the pre-reserve shutdown admission gap."""

        guard = LifecycleGuard("lab_go2", 7)
        guard.observe_message(message())
        subscriber = SimpleNamespace(is_alive=True)
        admission = LifecycleAdmission(
            guard=guard,
            subscriber=subscriber,
            topic="/map/lifecycle",
            snapshot=guard.require_ready(),
        )
        registry = LifecycleBoundRunRegistry()
        dispatch_lock = threading.RLock()
        paused_before_reserve = threading.Event()
        resume_request = threading.Event()
        dispatched = threading.Event()
        errors: list[Exception] = []
        runtime = {
            "accepting": True,
            "guard": guard,
            "subscriber": subscriber,
            "topic": "/map/lifecycle",
        }

        def request_thread() -> None:
            paused_before_reserve.set()
            self.assertTrue(resume_request.wait(1.0))
            try:
                with dispatch_lock:
                    admission.validate_current(
                        accepting=runtime["accepting"],
                        current_guard=runtime["guard"],
                        current_subscriber=runtime["subscriber"],
                        current_topic=runtime["topic"],
                    )
                    registry.reserve(
                        landmark_id="vending_machine_front",
                        landmark_name="自动售货机",
                        map_id="lab_go2",
                        map_generation=7,
                        lifecycle_revision=admission.snapshot.revision,
                        semantic_run_id="must-not-dispatch",
                    )
                dispatched.set()
            except Exception as exc:  # noqa: BLE001 - asserted below
                errors.append(exc)

        request = threading.Thread(target=request_thread)
        request.start()
        self.assertTrue(paused_before_reserve.wait(1.0))

        # Mirrors _stop_lifecycle_binding under the exact same dispatch lock.
        with dispatch_lock:
            runtime.update(
                accepting=False,
                guard=None,
                subscriber=None,
                topic="",
            )
        resume_request.set()
        request.join(timeout=1.0)

        self.assertFalse(request.is_alive())
        self.assertFalse(dispatched.is_set())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], LifecycleBindingError)

        # No hidden reservation was made before the simulated Navigate call.
        probe = registry.reserve(
            landmark_id="probe",
            landmark_name="probe",
            map_id="lab_go2",
            map_generation=7,
            lifecycle_revision=1,
            semantic_run_id="probe",
        )
        self.assertEqual(probe, "probe")


if __name__ == "__main__":
    unittest.main()
