#!/usr/bin/env python3
"""Deterministic voice-to-dashboard proof with no ROS, network, or motion.

This runner deliberately exercises the repository's real pure-Python safety
cores while replacing hardware-facing boundaries with in-memory fixtures. It
is evidence for interface wiring only; it is not a physical navigation test.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re
import socket
import sys
from typing import Any, Mapping

import yaml


ROOT = Path(__file__).resolve().parents[1]
SEMANTIC_PACKAGE = ROOT / "packages" / "semantic_navigation"
DASHBOARD_PACKAGE = ROOT / "packages" / "go2_dashboard"
for package in (SEMANTIC_PACKAGE, DASHBOARD_PACKAGE):
    package_text = str(package)
    if package_text not in sys.path:
        sys.path.insert(0, package_text)

from go2_dashboard.state import DashboardState  # noqa: E402
from semantic_navigation.core import Landmark, LandmarkStore, normalize_text  # noqa: E402
from semantic_navigation.map_lifecycle import (  # noqa: E402
    LifecycleGuard,
    MapLifecycleState,
)
from semantic_navigation.run_state import LifecycleBoundRunRegistry  # noqa: E402


DEFAULT_FIXTURE = ROOT / "tests" / "fixtures" / "offline_voice_e2e.yaml"
SEMANTIC_CONTRACT = (
    SEMANTIC_PACKAGE / "capabilities" / "navigate_landmark.v1.toml"
)
SEMANTIC_IDL = (
    SEMANTIC_PACKAGE
    / "capabilities"
    / "lib"
    / "semantic_navigation"
    / "srv"
    / "NavigateLandmark.srv"
)
NAVIGATE_CONTRACT_ID = "robonix/service/navigation/navigate"
STATUS_CONTRACT_ID = "robonix/service/navigation/navigate/status"
_CONTRACT_ID = re.compile(r'^\s*id\s*=\s*"([^"]+)"\s*$', re.MULTILINE)
_FORBIDDEN_RUNTIME_MODULES = (
    "rclpy",
    "nav2",
    "unitree_sdk2",
    "unitree_sdk2py",
)


class OfflineDemoError(RuntimeError):
    """The fixture or one of the fail-closed offline checks was invalid."""


@dataclass(frozen=True)
class AsrFinal:
    event_type: int
    text: str
    confidence: float
    is_final: bool


@dataclass(frozen=True)
class PilotSelection:
    contract_id: str
    arguments: dict[str, str]
    mode: str = "deterministic-rtdl-equivalent-fixture"


@dataclass(frozen=True)
class NavigationAcceptance:
    accepted: bool
    run_id: str
    detail: str


@dataclass(frozen=True)
class NavigationStatus:
    state: str
    detail: str


class _FixedClock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value


class FixedAsrFixture:
    """Return one final result in the shape produced by Robonix speech ASR."""

    def __init__(self, row: Mapping[str, Any]) -> None:
        self._result = AsrFinal(
            event_type=int(row["event_type"]),
            text=str(row["text"]),
            confidence=float(row["confidence"]),
            is_final=bool(row["is_final"]),
        )

    def recognize_final(self) -> AsrFinal:
        if self._result.event_type != 1 or not self._result.is_final:
            raise OfflineDemoError("ASR fixture must be a final event")
        if not self._result.text.strip():
            raise OfflineDemoError("ASR final transcript is empty")
        return self._result


class OfflineLiaisonFixture:
    """Mirror Liaison's rule that only the accumulated ASR final is submitted."""

    @staticmethod
    def submit_asr_final(event: AsrFinal) -> str:
        if event.event_type != 1 or not event.is_final:
            raise OfflineDemoError("Liaison received a non-final ASR event")
        transcript = event.text.strip()
        if not transcript:
            raise OfflineDemoError("Liaison received an empty transcript")
        return transcript


class DeterministicPilotFixture:
    """Emit the expected RTDL-equivalent contract selection without a model."""

    def __init__(
        self,
        *,
        expected_utterance: str,
        argument_name: str,
        contract_id: str,
    ) -> None:
        self._utterance = normalize_text(expected_utterance)
        self._argument_name = argument_name.strip()
        self._contract_id = contract_id

    def select(self, transcript: str) -> PilotSelection:
        if normalize_text(transcript) != self._utterance:
            raise OfflineDemoError("Pilot fixture has no deterministic plan for transcript")
        if not self._argument_name:
            raise OfflineDemoError("Pilot fixture semantic argument is empty")
        return PilotSelection(
            contract_id=self._contract_id,
            arguments={"name": self._argument_name},
        )


class FakeRobonixNavigation:
    """In-memory implementation of Navigate + Status contract semantics."""

    def __init__(self, row: Mapping[str, Any]) -> None:
        self._run_id = str(row["run_id"]).strip()
        self._detail = str(row["accepted_detail"]).strip()
        self._statuses = tuple(
            NavigationStatus(
                state=str(item["state"]).strip().upper(),
                detail=str(item["detail"]).strip(),
            )
            for item in row["statuses"]
        )
        self._status_index = 0
        self.requests: list[dict[str, Any]] = []
        if not self._run_id or tuple(item.state for item in self._statuses) != (
            "RUNNING",
            "SUCCEEDED",
        ):
            raise OfflineDemoError(
                "fake navigation must provide one deterministic RUNNING -> SUCCEEDED run"
            )

    def navigate(self, target: Landmark) -> NavigationAcceptance:
        self.requests.append(
            {
                "contract_id": NAVIGATE_CONTRACT_ID,
                "frame_id": target.frame_id,
                "x": target.x,
                "y": target.y,
                "yaw": target.yaw,
            }
        )
        return NavigationAcceptance(True, self._run_id, self._detail)

    def next_status(self, run_id: str) -> NavigationStatus:
        if run_id != self._run_id:
            raise OfflineDemoError(f"unknown fake navigation run {run_id!r}")
        if self._status_index >= len(self._statuses):
            raise OfflineDemoError("fake navigation status fixture is exhausted")
        result = self._statuses[self._status_index]
        self._status_index += 1
        return result


class _NoNetworkGuard:
    """Trip immediately if this supposedly offline path opens a socket."""

    def __init__(self) -> None:
        self._original_socket = socket.socket
        self.socket_attempts = 0

    def __enter__(self) -> "_NoNetworkGuard":
        def reject_socket(*_args: Any, **_kwargs: Any) -> Any:
            self.socket_attempts += 1
            raise OfflineDemoError("offline demo attempted to open a network socket")

        socket.socket = reject_socket  # type: ignore[assignment]
        return self

    def __exit__(self, *_exc: object) -> None:
        socket.socket = self._original_socket  # type: ignore[assignment]


def _mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise OfflineDemoError(f"{label} must be a mapping")
    return value


def _load_fixture(path: Path) -> Mapping[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise OfflineDemoError(f"cannot load fixture {path}: {exc}") from exc
    root = _mapping(document, "fixture")
    if root.get("schema_version") != 1:
        raise OfflineDemoError("unsupported offline scenario schema_version")
    return root


def _semantic_contract_id() -> str:
    source = SEMANTIC_CONTRACT.read_text(encoding="utf-8")
    match = _CONTRACT_ID.search(source)
    if match is None:
        raise OfflineDemoError("semantic capability descriptor has no contract id")
    contract_id = match.group(1)
    idl = SEMANTIC_IDL.read_text(encoding="utf-8")
    if "string name" not in idl:
        raise OfflineDemoError("semantic capability no longer accepts string name")
    return contract_id


def _validate_navigation_contracts() -> None:
    soma = (ROOT / "soma.yaml").read_text(encoding="utf-8")
    for contract_id in (NAVIGATE_CONTRACT_ID, STATUS_CONTRACT_ID):
        if contract_id not in soma:
            raise OfflineDemoError(
                f"Soma no longer exports fake-navigation contract {contract_id!r}"
            )


def _dashboard_update(
    dashboard: DashboardState,
    *,
    task_id: str,
    target_name: str,
    status: str,
    message: str,
    pose: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return dashboard.update_semantic_task(
        {
            "task_id": task_id,
            "target_name": target_name,
            "status": status,
            "message": message,
            "pose": pose,
        }
    )


def _assert_runtime_isolation() -> None:
    loaded = tuple(
        sorted(
            module
            for module in sys.modules
            if module.split(".", 1)[0] in _FORBIDDEN_RUNTIME_MODULES
        )
    )
    if loaded:
        raise OfflineDemoError(f"forbidden ROS/motion modules loaded: {loaded}")


def run_scenario(fixture_path: str | Path = DEFAULT_FIXTURE) -> dict[str, Any]:
    """Run and return a deterministic, JSON-serializable offline trace."""

    path = Path(fixture_path).resolve()
    fixture = _load_fixture(path)
    scenario = _mapping(fixture.get("scenario"), "scenario")
    active_map = _mapping(fixture.get("active_map"), "active_map")
    task_id = "semantic-offline-0001"
    timeline: list[dict[str, Any]] = []
    dashboard = DashboardState(
        monotonic_fn=_FixedClock(1000.0),
        wall_fn=_FixedClock(1_700_000_000.0),
    )

    with _NoNetworkGuard() as network_guard:
        _assert_runtime_isolation()
        _validate_navigation_contracts()
        event = FixedAsrFixture(
            _mapping(fixture.get("asr_final"), "asr_final")
        ).recognize_final()
        if normalize_text(event.text) != normalize_text(str(scenario["utterance"])):
            raise OfflineDemoError("ASR final does not match the scenario utterance")
        timeline.append({"stage": "asr_final", "text": event.text})

        transcript = OfflineLiaisonFixture.submit_asr_final(event)
        timeline.append({"stage": "liaison_submit", "text": transcript})
        _dashboard_update(
            dashboard,
            task_id=task_id,
            target_name=transcript,
            status="received",
            message="ASR final received by offline Liaison fixture",
        )

        pilot_row = _mapping(fixture.get("pilot_selection"), "pilot_selection")
        selection = DeterministicPilotFixture(
            expected_utterance=str(scenario["utterance"]),
            argument_name=str(pilot_row["argument_name"]),
            contract_id=_semantic_contract_id(),
        ).select(transcript)
        timeline.append(
            {
                "stage": "pilot_selection",
                "contract_id": selection.contract_id,
                "arguments": selection.arguments,
            }
        )
        _dashboard_update(
            dashboard,
            task_id=task_id,
            target_name=selection.arguments["name"],
            status="resolving",
            message="Pilot selected the saved-landmark capability fixture",
        )

        landmark_document = _mapping(
            fixture.get("landmark_document"), "landmark_document"
        )
        store = LandmarkStore.from_mapping(landmark_document)
        guard = LifecycleGuard(store.map_id, store.map_generation)
        lifecycle = MapLifecycleState(
            map_id=str(active_map["map_id"]),
            generation=int(active_map["generation"]),
            mode=str(active_map["mode"]),
        )
        guard.observe(lifecycle)
        lifecycle_snapshot = guard.require_ready()
        target = store.resolve(
            selection.arguments["name"],
            expected_map_id=lifecycle.map_id,
            expected_generation=lifecycle.generation,
            require_verified=True,
        )
        pose = {
            "frame_id": target.frame_id,
            "x": target.x,
            "y": target.y,
            "yaw": target.yaw,
        }
        timeline.append(
            {
                "stage": "semantic_resolved",
                "landmark_id": target.id,
                "verified": target.verified,
                "map_id": target.map_id,
                "map_generation": target.map_generation,
                "pose": pose,
            }
        )
        _dashboard_update(
            dashboard,
            task_id=task_id,
            target_name=target.name,
            status="resolved",
            message="verified saved Pose resolved against active map generation",
            pose=pose,
        )

        runs = LifecycleBoundRunRegistry()
        runs.reserve(
            semantic_run_id=task_id,
            landmark_id=target.id,
            landmark_name=target.name,
            map_id=target.map_id,
            map_generation=target.map_generation,
            lifecycle_revision=lifecycle_snapshot.revision,
        )
        navigation = FakeRobonixNavigation(
            _mapping(fixture.get("fake_navigation"), "fake_navigation")
        )
        acceptance = navigation.navigate(target)
        if not acceptance.accepted:
            raise OfflineDemoError("fake Robonix navigation rejected the target")
        run, must_cancel = runs.attach_navigation(
            task_id, acceptance.run_id, acceptance.detail
        )
        if must_cancel:
            raise OfflineDemoError("offline run was unexpectedly invalidated")
        _dashboard_update(
            dashboard,
            task_id=task_id,
            target_name=target.name,
            status="navigating",
            message=f"fake Robonix navigation run {run.nav_run_id} started",
            pose=pose,
        )

        remote_states: list[str] = []
        while not runs.get(task_id).remote_terminal:
            status = navigation.next_status(acceptance.run_id)
            remote_states.append(status.state)
            run = runs.update_remote(task_id, status.state, status.detail)
            timeline.append(
                {
                    "stage": "navigation_status",
                    "contract_id": STATUS_CONTRACT_ID,
                    "state": run.remote_state,
                    "detail": run.navigation_detail,
                }
            )
            _dashboard_update(
                dashboard,
                task_id=task_id,
                target_name=target.name,
                status=("succeeded" if run.remote_state == "SUCCEEDED" else "navigating"),
                message=run.navigation_detail,
                pose=pose,
            )

        final_run = runs.get(task_id)
        final_dashboard = dashboard.snapshot()
        _assert_runtime_isolation()

    return {
        "result": "PASS",
        "scope": "offline-fixture-only",
        "scenario_id": str(scenario["id"]),
        "asr_final": asdict(event),
        "liaison_transcript": transcript,
        "pilot_selection": asdict(selection),
        "landmark": {
            "id": target.id,
            "name": target.name,
            "verified": target.verified,
            "map_id": target.map_id,
            "map_generation": target.map_generation,
            "pose": pose,
        },
        "navigation": {
            "contracts": [NAVIGATE_CONTRACT_ID, STATUS_CONTRACT_ID],
            "requests": navigation.requests,
            "states": remote_states,
            "semantic_state": final_run.state,
            "remote_terminal": final_run.remote_terminal,
        },
        "dashboard": {
            "read_only": final_dashboard["read_only"],
            "bridge_connected": final_dashboard["bridge"]["connected"],
            "semantic_task": final_dashboard["semantic_task"],
        },
        "isolation": {
            "network_socket_attempts": network_guard.socket_attempts,
            "ros_graph_started": False,
            "motion_interfaces_loaded": False,
            "publishers_created": 0,
        },
        "timeline": timeline,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args(argv)
    result = run_scenario(args.fixture)
    print(
        json.dumps(
            result,
            ensure_ascii=False,
            sort_keys=True,
            indent=None if args.compact else 2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
