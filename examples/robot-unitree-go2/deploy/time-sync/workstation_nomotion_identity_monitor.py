#!/usr/bin/env python3
"""Continuously bind the no-motion timestamp profile to exact DDS writers.

ROS 2 Humble subscription callbacks do not expose publisher GID to Python.
This graph-only node therefore uses ``get_publishers_info_by_topic()`` and
binds exactly one current-boot endpoint GID for every raw input before READY.
After that point the bound set is immutable for the process lifetime.  Saved
approval GIDs remain historical evidence and never reject a legitimate robot
reboot.  The monitor creates no subscription, publisher, client, service,
action, or Unitree API object.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from workstation_nomotion_approval import (  # noqa: E402
    EXPECTED_RAW_TOPICS,
    ApprovalError,
    FixedOffsetApproval,
    load_approval,
)


READY_SCHEMA = "robonix-go2-workstation-nomotion-writer-identity-ready-v1"
FAULT_SCHEMA = "robonix-go2-workstation-nomotion-writer-identity-fault-v1"
INITIAL_DISCOVERY_SECONDS = 15
STABLE_DISCOVERY_SAMPLES = 3
POLL_INTERVAL_SECONDS = 0.25


def endpoint_gid_hex(value: Any) -> str:
    if not isinstance(value, (list, tuple)) or len(value) != 24:
        raise ValueError("publisher GID is not exactly 24 bytes")
    if any(
        not isinstance(item, int)
        or isinstance(item, bool)
        or not 0 <= item <= 255
        for item in value
    ):
        raise ValueError("publisher GID contains a non-byte value")
    return bytes(value).hex()


def writer_identity_failures(
    expected_writer_gids: Mapping[str, str],
    observations: dict[str, list[Any]],
) -> list[str]:
    """Compare a graph snapshot with one complete, exact writer-GID set.

    This pure helper is shared by the long-lived approval monitor and the
    bounded pre-capture identity verifier.  Keeping the comparison in one
    place prevents the evidence-refresh path from acquiring a weaker identity
    policy than the runtime path.
    """

    failures: list[str] = []
    for stream in EXPECTED_RAW_TOPICS:
        expected_gid = expected_writer_gids.get(stream)
        if not isinstance(expected_gid, str):
            failures.append(f"{stream}:expected_gid_missing")
            continue
        endpoints = observations.get(stream, [])
        if len(endpoints) != 1:
            failures.append(f"{stream}:publisher_count:{len(endpoints)}")
            continue
        try:
            actual_gid = endpoint_gid_hex(endpoints[0].endpoint_gid)
        except (AttributeError, ValueError) as error:
            failures.append(f"{stream}:invalid_gid:{error}")
            continue
        if actual_gid != expected_gid:
            failures.append(f"{stream}:writer_gid_mismatch")
    return failures


def observed_unique_writer_gids(
    observations: dict[str, list[Any]],
) -> tuple[dict[str, str], list[str]]:
    """Return one complete current graph GID set or structural failures.

    This is the startup binding policy.  It deliberately does not compare
    against GIDs captured during an older boot: DDS writer identities are
    expected to change when the robot restarts.  Exactly-one ownership and a
    valid 24-byte GID are still mandatory for every stream.
    """

    writer_gids: dict[str, str] = {}
    failures: list[str] = []
    for stream in EXPECTED_RAW_TOPICS:
        endpoints = observations.get(stream, [])
        if len(endpoints) != 1:
            failures.append(f"{stream}:publisher_count:{len(endpoints)}")
            continue
        try:
            writer_gids[stream] = endpoint_gid_hex(endpoints[0].endpoint_gid)
        except (AttributeError, ValueError) as error:
            failures.append(f"{stream}:invalid_gid:{error}")
    if failures:
        return {}, failures
    return writer_gids, []


def identity_failures(
    approval: FixedOffsetApproval,
    observations: dict[str, list[Any]],
) -> list[str]:
    return writer_identity_failures(dict(approval.writer_gids), observations)


def recoverable_ready_writer_absence(failures: list[str]) -> bool:
    """Return true only when bound writers are temporarily undiscovered.

    After READY, a private-router power interruption can remove otherwise
    unchanged writers from the local graph cache.  Zero writers carry no
    conflicting identity, and the timestamp relay independently stops output
    on receipt liveness.  A mismatched GID, multiple writers, malformed graph
    data, approval change, or any other failure remains terminal.
    """

    if not failures:
        return False
    expected_prefixes = {
        f"{stream}:publisher_count:0" for stream in EXPECTED_RAW_TOPICS
    }
    return all(failure in expected_prefixes for failure in failures)


def load_session_approval(
    approval_file: Path,
    approval: FixedOffsetApproval,
    *,
    ready: bool,
    now_realtime_ns: int | None = None,
) -> FixedOffsetApproval:
    """Reload one immutable approval without turning expiry into a kill timer.

    Before READY the real current time still has to fall inside the approval
    interval. After READY, the interval has already authorized this startup and
    model lock, so use its immutable start solely to revalidate file structure
    and content. Any content, ownership, mode, schema, or identity change still
    fails the equality check.
    """

    current = load_approval(
        approval_file,
        now_realtime_ns=(
            approval.not_before_unix_ns if ready else now_realtime_ns
        ),
    )
    if current != approval:
        raise ApprovalError("approval content changed during the session")
    return current


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, sort_keys=True, separators=(",", ":"))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def run_ros(
    approval_file: Path,
    approval: FixedOffsetApproval,
    ready_file: Path,
    fault_file: Path,
) -> int:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    context = Context()
    rclpy.init(args=[], context=context)
    fault_reason: str | None = None

    class IdentityMonitor(Node):
        def __init__(self) -> None:
            super().__init__(
                "go2_workstation_nomotion_writer_identity", context=context
            )
            self._deadline_ns = (
                time.monotonic_ns() + INITIAL_DISCOVERY_SECONDS * 1_000_000_000
            )
            self._stable = 0
            self._ready = False
            self._candidate_writer_gids: dict[str, str] | None = None
            self._bound_writer_gids: dict[str, str] | None = None
            self._timer = self.create_timer(POLL_INTERVAL_SECONDS, self._check)

        def _fault(self, reason: str) -> None:
            nonlocal fault_reason
            if fault_reason is not None:
                return
            fault_reason = reason[:500] or "unspecified_identity_fault"
            try:
                _atomic_json(
                    fault_file,
                    {
                        "schema": FAULT_SCHEMA,
                        "session_id": approval.session_id,
                        "reason": fault_reason,
                        "motion_ready": False,
                        "canonical_odom_ready": False,
                    },
                )
            finally:
                # Graph identity evidence is mandatory.  A status-file error
                # must still remove this process and stop the wrapper.
                if context.ok():
                    context.shutdown()

        def _check(self) -> None:
            try:
                # Expiry is a startup/lock deadline, not a lifetime limit for a
                # no-motion mapping session which already reached READY. Keep
                # reopening and fully validating the private file so any
                # content/ownership/permission change is still detected, while
                # checking the original interval at its start after READY.
                load_session_approval(
                    approval_file,
                    approval,
                    ready=self._ready,
                )
                observations = {
                    stream: self.get_publishers_info_by_topic(topic)
                    for stream, topic in EXPECTED_RAW_TOPICS.items()
                }
                observed_writer_gids, failures = observed_unique_writer_gids(
                    observations
                )
                if not failures and self._ready:
                    if self._bound_writer_gids is None:
                        failures = ["internal_bound_writer_gids_missing"]
                    else:
                        failures = writer_identity_failures(
                            self._bound_writer_gids,
                            observations,
                        )
            except Exception as error:
                observed_writer_gids = {}
                failures = [f"graph_check_failed:{type(error).__name__}:{error}"]

            if failures:
                self._stable = 0
                self._candidate_writer_gids = None
                if (
                    self._ready
                    and recoverable_ready_writer_absence(failures)
                ):
                    # Remain alive and require the exact bound GIDs again
                    # when discovery returns.  The timestamp process pauses
                    # corrected output independently while receipts are absent.
                    return
                if (
                    self._ready
                    or time.monotonic_ns() >= self._deadline_ns
                ):
                    self._fault(";".join(failures))
                return

            if self._ready:
                return

            if observed_writer_gids != self._candidate_writer_gids:
                self._candidate_writer_gids = dict(observed_writer_gids)
                self._stable = 1
            else:
                self._stable += 1
            if self._stable < STABLE_DISCOVERY_SAMPLES:
                if time.monotonic_ns() >= self._deadline_ns:
                    self._fault("current_writer_gids_not_stable_before_deadline")
                return

            self._bound_writer_gids = dict(observed_writer_gids)
            try:
                _atomic_json(
                    ready_file,
                    {
                        "schema": READY_SCHEMA,
                        "session_id": approval.session_id,
                        "writer_gids": dict(self._bound_writer_gids),
                        "raw_topics": EXPECTED_RAW_TOPICS,
                        "identity_bound": True,
                        "binding_source": "current_unique_graph_publishers",
                        "historical_approval_gids_enforced": False,
                        "motion_ready": False,
                        "canonical_odom_ready": False,
                    },
                )
            except Exception as error:
                self._fault(f"ready_status_write_failed:{type(error).__name__}")
                return
            self._ready = True

    # The node deliberately owns a private Context.  Using rclpy.spin() without
    # an executor would silently select rclpy's global executor, whose guard
    # conditions belong to the default Context.  Humble then fails as soon as
    # the two contexts are mixed.  Keep the executor on the exact same Context
    # and own its complete lifecycle here.
    executor = SingleThreadedExecutor(context=context)
    node = IdentityMonitor()
    added = False
    try:
        added = executor.add_node(node)
        if not added:
            raise RuntimeError("identity monitor could not be added to its executor")
        executor.spin()
    finally:
        try:
            # remove_node() wakes the executor guard condition.  Do that only
            # while its Context is valid; Executor.shutdown() clears its node
            # set itself when a fault/SIGINT already shut the Context down.
            if added and context.ok():
                executor.remove_node(node)
        finally:
            try:
                executor.shutdown()
            finally:
                node.destroy_node()
                if context.ok():
                    context.shutdown()
    return 70 if fault_reason is not None else 0


def _absolute_path(value: str) -> Path:
    path = Path(value)
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("runtime paths must be absolute")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Continuously verify exact raw-topic publisher GIDs (no motion)"
    )
    parser.add_argument("--approval-file", required=True, type=Path)
    parser.add_argument("--ready-file", required=True, type=_absolute_path)
    parser.add_argument("--fault-file", required=True, type=_absolute_path)
    args = parser.parse_args()
    if args.ready_file == args.fault_file:
        parser.error("ready and fault files must differ")
    for path in (args.ready_file, args.fault_file):
        if path.exists() or path.is_symlink():
            parser.error(f"runtime status path already exists: {path}")
    try:
        approval = load_approval(args.approval_file)
    except ApprovalError as error:
        parser.error(str(error))
    return run_ros(args.approval_file, approval, args.ready_file, args.fault_file)


if __name__ == "__main__":
    raise SystemExit(main())
