#!/usr/bin/env python3
"""Capture a stable, read-only DDS writer baseline before motion boot.

The Go2 firmware exposes several built-in writers on /api/sport/request.  This
script records those pre-existing GIDs so the one-shot motion probe can require
exactly one additional, stable workstation SDK writer after Robonix boots.
It creates no publisher, subscription, client, service, or Unitree API object.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
from typing import Sequence


SCHEMA = "robonix-go2-motion-rpc-graph-baseline-v1"
SPORT_REQUEST_TOPIC = "/api/sport/request"
SPORT_LEASE_REQUEST_TOPIC = "/api/sport_lease/request"
DDS_GID_SIZE = 24
STABLE_SAMPLES = 3
SAMPLE_PERIOD_S = 0.10
CAPTURE_TIMEOUT_S = 5.0


def normalized_gids(endpoints: Sequence[object]) -> tuple[str, ...]:
    result: list[str] = []
    for endpoint in endpoints:
        try:
            gid = bytes(endpoint.endpoint_gid)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError("DDS endpoint has no usable GID") from exc
        if len(gid) != DDS_GID_SIZE or not any(gid):
            raise ValueError("DDS endpoint GID is invalid")
        result.append(gid.hex())
    if len(result) != len(set(result)):
        raise ValueError("DDS graph contains duplicate writer GIDs")
    return tuple(sorted(result))


def write_payload(output: Path, payload: dict[str, object]) -> None:
    if not output.is_absolute():
        raise ValueError("--output must be an absolute path")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.chmod(temporary, 0o600)
        os.replace(temporary, output)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
    except ImportError as exc:
        print(f"ROS 2 Python runtime is unavailable: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("go2_motion_rpc_graph_baseline")
    stable = 0
    previous: tuple[str, ...] | None = None
    deadline = time.monotonic() + CAPTURE_TIMEOUT_S
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=SAMPLE_PERIOD_S)
            lease_publishers = node.get_publishers_info_by_topic(
                SPORT_LEASE_REQUEST_TOPIC
            )
            if lease_publishers:
                raise RuntimeError(
                    "positive-lease request writer exists before motion boot"
                )
            current = normalized_gids(
                node.get_publishers_info_by_topic(SPORT_REQUEST_TOPIC)
            )
            stable = stable + 1 if current == previous else 1
            previous = current
            if stable >= STABLE_SAMPLES:
                payload: dict[str, object] = {
                    "schema": SCHEMA,
                    "captured_unix_ns": time.time_ns(),
                    "sport_request_topic": SPORT_REQUEST_TOPIC,
                    "sport_request_writer_count": len(current),
                    "sport_request_writer_gids": list(current),
                    "sport_lease_request_topic": SPORT_LEASE_REQUEST_TOPIC,
                    "sport_lease_request_writer_count": 0,
                    "stable_samples": stable,
                }
                write_payload(args.output, payload)
                print(args.output)
                return 0
        print("DDS writer baseline did not become stable", file=sys.stderr)
        return 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"could not capture motion RPC graph baseline: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
