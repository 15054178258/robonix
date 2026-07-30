# SPDX-License-Identifier: MulanPSL-2.0
"""Timestamp contract tests for Scene's tf2 adapter."""

from types import SimpleNamespace

import numpy as np

from scene_service.ingest.ros_subscribers import SubscribersHub


class _Time:
    def __init__(self, *, seconds=0, nanoseconds=0):
        self.seconds = seconds
        self.nanoseconds = nanoseconds


class _Duration:
    def __init__(self, *, seconds):
        self.seconds = seconds


class _Buffer:
    def __init__(self):
        self.calls = []

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append((target, source, stamp, timeout))
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


def _hub():
    hub = SubscribersHub([])
    hub._ros = {"Time": _Time, "Duration": _Duration}
    hub._tf_buffer = _Buffer()
    return hub


def test_lookup_transform_uses_message_acquisition_time():
    hub = _hub()

    matrix = hub.lookup_transform_4x4(
        "camera",
        "map",
        stamp=SimpleNamespace(sec=123, nanosec=456),
    )

    np.testing.assert_allclose(matrix[:3, 3], [1.0, 2.0, 3.0])
    target, source, stamp, timeout = hub._tf_buffer.calls[0]
    assert (target, source) == ("map", "camera")
    assert (stamp.seconds, stamp.nanoseconds) == (123, 456)
    assert timeout.seconds == 1.0


def test_lookup_transform_keeps_latest_semantics_for_unstamped_callers():
    hub = _hub()

    matrix = hub.lookup_transform_4x4("camera", "map")

    assert matrix is not None
    stamp = hub._tf_buffer.calls[0][2]
    assert (stamp.seconds, stamp.nanoseconds) == (0, 0)
