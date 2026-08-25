#!/usr/bin/env python3
"""Send one absolute map-frame Navigation goal and preserve its outcome.

This tool discovers every Navigation RPC through Atlas, never publishes a
velocity command, and never arms the chassis.  A non-terminal goal is formally
canceled before the tool exits.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Iterator


ROOT = Path(__file__).resolve().parents[1]
PROTO_GEN = ROOT / "packages" / "semantic_navigation" / "rbnx-build" / "codegen" / "proto_gen"
sys.path.insert(0, str(PROTO_GEN))

import grpc  # noqa: E402
import atlas_pb2  # noqa: E402
import atlas_pb2_grpc  # noqa: E402
import navigation_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402


ATLAS_ENDPOINT = "127.0.0.1:50051"
PROVIDER_ID = "nav2"
CONSUMER_ID = "robonix-go2-absolute-goal-runner"
TERMINAL_STATES = {"SUCCEEDED", "FAILED", "CANCELED"}


def finite(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError("must be finite")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", required=True, type=finite)
    parser.add_argument("--y", required=True, type=finite)
    parser.add_argument("--yaw", required=True, type=finite)
    parser.add_argument("--timeout-s", type=finite, default=35.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 5.0 <= args.timeout_s <= 180.0:
        parser.error("--timeout-s must be in the range 5..180")
    return args


def resolve_output(raw: str) -> Path:
    candidate = Path(raw).expanduser()
    if not candidate.is_absolute():
        candidate = ROOT / candidate
    if candidate.exists() and candidate.is_symlink():
        raise ValueError("output must not be a symlink")
    result = candidate.resolve()
    if not result.is_relative_to(ROOT):
        raise ValueError(f"output must remain below {ROOT}")
    return result


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    temporary.replace(path)


@contextmanager
def navigation_rpc(contract_id: str, stub_type: type) -> Iterator[object]:
    atlas_channel = grpc.insecure_channel(ATLAS_ENDPOINT)
    atlas = atlas_pb2_grpc.AtlasStub(atlas_channel)
    connection = None
    rpc_channel = None
    try:
        query = atlas.Query(
            atlas_pb2.QueryRequest(
                kind=atlas_pb2.KIND_SERVICE,
                id=PROVIDER_ID,
                contract_id=contract_id,
                transport=atlas_pb2.TRANSPORT_GRPC,
            ),
            timeout=3.0,
        )
        providers = [
            provider
            for provider in query.providers
            if provider.id == PROVIDER_ID
            and provider.state == atlas_pb2.STATE_ACTIVE
            and sum(
                capability.contract_id == contract_id
                and capability.transport == atlas_pb2.TRANSPORT_GRPC
                for capability in provider.capabilities
            )
            == 1
        ]
        if len(providers) != 1:
            raise RuntimeError(
                f"expected one active {PROVIDER_ID!r} provider for {contract_id}, "
                f"found {len(providers)}"
            )
        connection = atlas.ConnectCapability(
            atlas_pb2.ConnectCapabilityRequest(
                consumer_id=CONSUMER_ID,
                provider_id=PROVIDER_ID,
                contract_id=contract_id,
                transport=atlas_pb2.TRANSPORT_GRPC,
            ),
            timeout=3.0,
        )
        endpoint = connection.endpoint.strip()
        if not endpoint:
            raise RuntimeError(f"Atlas returned an empty endpoint for {contract_id}")
        rpc_channel = grpc.insecure_channel(endpoint)
        grpc.channel_ready_future(rpc_channel).result(timeout=3.0)
        yield stub_type(rpc_channel)
    finally:
        if rpc_channel is not None:
            rpc_channel.close()
        if connection is not None and connection.channel_id:
            try:
                atlas.DisconnectCapability(
                    atlas_pb2.DisconnectCapabilityRequest(
                        channel_id=connection.channel_id
                    ),
                    timeout=3.0,
                )
            except grpc.RpcError:
                pass
        atlas_channel.close()


def get_status(run_id: str):
    with navigation_rpc(
        "robonix/service/navigation/navigate/status",
        contracts_grpc.RobonixServiceNavigationNavigateStatusStub,
    ) as stub:
        return stub.GetNavigationStatus(
            navigation_pb2.GetNavigationStatus_Request(run_id=run_id), timeout=3.0
        )


def cancel(run_id: str) -> str:
    with navigation_rpc(
        "robonix/service/navigation/navigate/cancel",
        contracts_grpc.RobonixServiceNavigationNavigateCancelStub,
    ) as stub:
        response = stub.CancelNavigation(
            navigation_pb2.CancelNavigation_Request(run_id=run_id), timeout=3.0
        )
    return f"accepted={response.accepted}; {response.detail}"


def wait_terminal(run_id: str, deadline: float, history: list[dict[str, object]]):
    last = None
    while time.monotonic() < deadline:
        response = get_status(run_id)
        sample = {
            "unix_ns": time.time_ns(),
            "known": bool(response.known),
            "state": str(response.state),
            "detail": str(response.detail),
        }
        if sample != last:
            history.append(sample)
            print(f"{sample['state']}: {sample['detail']}", flush=True)
            last = sample
        if response.known and response.state in TERMINAL_STATES:
            return response
        time.sleep(0.25)
    return None


def main() -> int:
    args = parse_args()
    output = resolve_output(args.output)
    evidence: dict[str, object] = {
        "schema_version": 1,
        "operation": "absolute_map_navigation_once",
        "goal": {"frame_id": "map", "x": args.x, "y": args.y, "yaw": args.yaw},
        "started_unix_ns": time.time_ns(),
        "timeout_s": args.timeout_s,
        "status": "error",
        "history": [],
    }
    run_id = ""
    terminal = None
    try:
        request = navigation_pb2.Navigate_Request()
        request.goal.header.frame_id = "map"
        request.goal.pose.position.x = args.x
        request.goal.pose.position.y = args.y
        request.goal.pose.orientation.z = math.sin(args.yaw / 2.0)
        request.goal.pose.orientation.w = math.cos(args.yaw / 2.0)
        with navigation_rpc(
            "robonix/service/navigation/navigate",
            contracts_grpc.RobonixServiceNavigationNavigateStub,
        ) as stub:
            accepted = stub.Navigate(request, timeout=5.0)
        evidence["acceptance"] = {
            "accepted": bool(accepted.accepted),
            "run_id": str(accepted.run_id),
            "detail": str(accepted.detail),
        }
        if not accepted.accepted or not accepted.run_id:
            raise RuntimeError(f"navigation goal rejected: {accepted.detail}")
        run_id = str(accepted.run_id)
        terminal = wait_terminal(
            run_id, time.monotonic() + args.timeout_s, evidence["history"]
        )
        if terminal is None:
            evidence["cancel"] = cancel(run_id)
            terminal = wait_terminal(
                run_id, time.monotonic() + 10.0, evidence["history"]
            )
            if terminal is None:
                raise RuntimeError("navigation remained non-terminal after cancel")
        evidence["terminal"] = {
            "known": bool(terminal.known),
            "state": str(terminal.state),
            "detail": str(terminal.detail),
        }
        evidence["status"] = "pass" if terminal.state == "SUCCEEDED" else "failed"
        return 0 if terminal.state == "SUCCEEDED" else 1
    except BaseException as error:
        evidence["error"] = f"{type(error).__name__}: {error}"
        if run_id and terminal is None:
            try:
                evidence["cancel"] = cancel(run_id)
                terminal = wait_terminal(
                    run_id, time.monotonic() + 10.0, evidence["history"]
                )
            except Exception as cancel_error:
                evidence["cancel_error"] = (
                    f"{type(cancel_error).__name__}: {cancel_error}"
                )
        if isinstance(error, KeyboardInterrupt):
            return_code = 130
        else:
            return_code = 1
        return return_code
    finally:
        evidence["finished_unix_ns"] = time.time_ns()
        write_json(output, evidence)
        print(f"evidence={output}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
