#!/usr/bin/env python3
"""Bounded publisher-ownership gate with no ROS communication endpoints.

ROS 2 Humble's high-level ``Node`` (and therefore ``ros2 topic info``) creates
an implicit ``/parameter_events`` publisher.  This gate deliberately uses only
the installed low-level node handle and graph-count API, so the audit process
owns no publisher, subscription, service, client, action, timer, or Unitree
SDK object.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Callable, Mapping, Sequence


TOPICS = (
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/scanner/cloud",
    "/scanner/imu",
    "/odom",
    "/tf_static",
)

PROFILE_COUNTS: dict[tuple[str, str], tuple[int, ...]] = {
    ("workstation-local", "pre"): (0, 0, 0, 0, 0, 0),
    ("workstation-full-nomotion-corrected", "pre"): (0, 0, 0, 0, 0, 0),
    ("nx-full", "pre"): (0, 0, 0, 0, 0, 0),
    ("nx-sensors-only", "pre"): (0, 0, 0, 0, 0, 0),
    ("workstation-full-nx-sensors", "pre"): (1, 1, 1, 1, 0, 0),
    ("workstation-ui-nx-full", "pre"): (1, 1, 1, 1, 1, 1),
    ("workstation-local", "post"): (1, 1, 1, 1, 1, 1),
    ("workstation-full-nomotion-corrected", "post"): (1, 1, 1, 1, 1, 1),
    ("workstation-full-nx-sensors", "post"): (1, 1, 1, 1, 1, 1),
    ("workstation-ui-nx-full", "post"): (1, 1, 1, 1, 1, 1),
    ("nx-full", "post"): (1, 1, 1, 1, 1, 1),
    ("nx-sensors-only", "post"): (1, 1, 1, 1, 0, 0),
}


class OwnershipError(RuntimeError):
    """A graph query or ownership invariant could not be proven."""


def _bounded_environment_int(
    name: str, default: int, maximum: int, environment: Mapping[str, str]
) -> int:
    raw = environment.get(name, str(default))
    try:
        value = int(raw)
    except ValueError as error:
        raise OwnershipError(f"{name} must be a positive integer") from error
    if value < 1:
        raise OwnershipError(f"{name} must be a positive integer")
    if value > maximum:
        raise OwnershipError(f"{name} exceeds its audited maximum {maximum}")
    return value


def expected_counts(profile: str, phase: str) -> tuple[int, ...]:
    if phase not in {"pre", "post"}:
        raise OwnershipError("ownership phase must be exactly pre or post")
    try:
        return PROFILE_COUNTS[(profile, phase)]
    except KeyError as error:
        raise OwnershipError(f"unknown runtime ownership profile: {profile}") from error


def evaluate_counts(
    observed: Sequence[int],
    expected: Sequence[int],
) -> tuple[bool, str | None]:
    if len(observed) != len(TOPICS) or len(expected) != len(TOPICS):
        raise OwnershipError("publisher ownership count vector is malformed")
    missing = False
    for topic, actual, wanted in zip(TOPICS, observed, expected, strict=True):
        if isinstance(actual, bool) or not isinstance(actual, int) or actual < 0:
            raise OwnershipError(f"publisher count is invalid for {topic}")
        if actual > wanted:
            return (
                False,
                f"publisher ownership violation: {topic} "
                f"expected {wanted}, found {actual}",
            )
        if actual < wanted:
            missing = True
    return not missing, None


def observe_live_counts() -> tuple[Callable[[], tuple[int, ...]], Callable[[], None]]:
    """Create only a low-level graph node and return query/cleanup callables."""

    try:
        from rclpy.context import Context
        from rclpy.impl.implementation_singleton import (
            rclpy_implementation as _rclpy,
        )
    except ImportError as error:
        raise OwnershipError(f"ROS 2 graph dependency is unavailable: {error}") from error

    context = Context()
    node_handle = None
    try:
        context.init(args=[], initialize_logging=False)
        with context.handle:
            node_handle = _rclpy.Node(
                "go2_runtime_ownership_readonly",
                "",
                context.handle,
                None,
                False,
                False,
            )
    except Exception:
        if node_handle is not None:
            try:
                node_handle.destroy_when_not_in_use()
            except Exception:
                pass
        try:
            context.try_shutdown()
            context.destroy()
        except Exception:
            pass
        raise

    def query() -> tuple[int, ...]:
        if not context.ok():
            raise OwnershipError("ROS context shut down during ownership audit")
        try:
            with node_handle:
                return tuple(
                    int(node_handle.get_count_publishers(topic)) for topic in TOPICS
                )
        except Exception as error:
            raise OwnershipError(
                f"could not query publisher ownership: {type(error).__name__}: {error}"
            ) from error

    def cleanup() -> None:
        errors: list[str] = []
        try:
            node_handle.destroy_when_not_in_use()
        except Exception as error:
            errors.append(f"node: {type(error).__name__}: {error}")
        try:
            context.try_shutdown()
            context.destroy()
        except Exception as error:
            errors.append(f"context: {type(error).__name__}: {error}")
        if errors:
            raise OwnershipError(
                "publisher ownership cleanup failed: " + "; ".join(errors)
            )

    return query, cleanup


def run(
    profile: str,
    phase: str,
    *,
    environment: Mapping[str, str] | None = None,
    query_factory: Callable[
        [], tuple[Callable[[], tuple[int, ...]], Callable[[], None]]
    ] = observe_live_counts,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    environment = os.environ if environment is None else environment
    try:
        expected = expected_counts(profile, phase)
        discovery_timeout = _bounded_environment_int(
            "GO2_OWNERSHIP_DISCOVERY_TIMEOUT_S", 20, 60, environment
        )
        stable_samples = _bounded_environment_int(
            "GO2_OWNERSHIP_STABILITY_SAMPLES", 3, 10, environment
        )
        _bounded_environment_int(
            "GO2_OWNERSHIP_QUERY_TIMEOUT_S", 4, 15, environment
        )
    except OwnershipError as error:
        print(error, file=sys.stderr)
        return 2

    print("================================================================")
    print(f" READ-ONLY ROS PUBLISHER OWNERSHIP {phase.upper()}CHECK: {profile}")
    print(" Low-level graph discovery only: zero ROS communication endpoints.")
    print(" No topic is published and no service, action or motion API is called.")
    print("================================================================")

    query: Callable[[], tuple[int, ...]]
    cleanup: Callable[[], None]
    try:
        query, cleanup = query_factory()
    except Exception as error:
        print(f"could not initialize publisher ownership audit: {error}", file=sys.stderr)
        return 4

    deadline = monotonic() + discovery_timeout
    stable = 0
    last_counts = (0,) * len(TOPICS)
    status = 6
    try:
        while monotonic() <= deadline:
            try:
                last_counts = query()
                exact, violation = evaluate_counts(last_counts, expected)
            except OwnershipError as error:
                print(error, file=sys.stderr)
                status = 4
                break
            if violation is not None:
                print(violation, file=sys.stderr)
                status = 5
                break
            if exact:
                stable += 1
                if stable >= stable_samples:
                    for topic, count in zip(TOPICS, last_counts, strict=True):
                        print(
                            f"  {topic} publishers={count} owner-profile={profile}"
                        )
                    print("READ-ONLY publisher ownership "
                          f"{phase}check passed")
                    status = 0
                    break
            else:
                stable = 0
            sleep(1.0)
        else:
            status = 6
    finally:
        try:
            cleanup()
        except OwnershipError as error:
            print(error, file=sys.stderr)
            if status == 0:
                status = 4

    if status == 6:
        for topic, wanted, actual in zip(
            TOPICS, expected, last_counts, strict=True
        ):
            print(
                f"  {topic} expected={wanted} observed={actual}",
                file=sys.stderr,
            )
        print(
            f"publisher ownership discovery deadline expired for profile: {profile}",
            file=sys.stderr,
        )
    return status


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    parser.add_argument("phase", nargs="?", default="pre")
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    return run(arguments.profile, arguments.phase)


if __name__ == "__main__":
    raise SystemExit(main())
