# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import os
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from mapping_rbnx import map_ops


class LoadMapTransactionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        map_dir = os.path.join(self.tmp.name, "target")
        os.makedirs(map_dir)
        self.saved_db = os.path.join(map_dir, "rtabmap.db")
        open(self.saved_db, "wb").close()

    def test_localization_precedes_database_load_and_publish_is_verified(self):
        order = []
        barrier = {"subscription": object()}

        with (
            patch.object(map_ops, "MAPS_DIR", self.tmp.name),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(
                map_ops,
                "_runtime_db_copy",
                side_effect=lambda *_: order.append("copy") or "/runtime/target.db",
            ),
            patch.object(
                map_ops, "_set_mode", side_effect=lambda *_: order.append("mode") or (True, "ok")
            ),
            patch.object(map_ops, "set_current_mode"),
            patch.object(
                map_ops,
                "_load_database",
                side_effect=lambda *_: order.append("load") or (True, "ok"),
            ),
            patch.object(
                map_ops,
                "_publish_full_map",
                side_effect=lambda *_args, **_kwargs: order.append("publish")
                or (True, "published optimized global map"),
            ),
            patch.object(
                map_ops,
                "_begin_target_map_wait",
                side_effect=lambda *_: order.append("subscribe") or barrier,
            ),
            patch.object(
                map_ops,
                "_finish_target_map_wait",
                side_effect=lambda *_: order.append("verify")
                or (True, "verified fresh occupancy"),
            ),
            patch.object(
                map_ops.lifecycle,
                "set_state",
                side_effect=lambda *_args, **_kwargs: order.append("identity"),
            ),
        ):
            result = map_ops.load_map_impl("target")

        self.assertTrue(result["ok"], result)
        self.assertEqual(
            order,
            ["copy", "mode", "load", "identity", "publish", "subscribe", "verify"],
        )
        self.assertIn("verified fresh occupancy", result["detail"])

    def test_mode_switch_failure_never_loads_database(self):
        with (
            patch.object(map_ops, "MAPS_DIR", self.tmp.name),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(map_ops, "_runtime_db_copy", return_value="/runtime/target.db"),
            patch.object(map_ops, "_set_mode", return_value=(False, "mode failed")),
            patch.object(map_ops, "_load_database") as load,
        ):
            result = map_ops.load_map_impl("target")

        self.assertFalse(result["ok"])
        self.assertIn("before load", result["detail"])
        load.assert_not_called()

    @staticmethod
    def _occupancy(width, height, resolution, data):
        return SimpleNamespace(
            info=SimpleNamespace(
                width=width,
                height=height,
                resolution=resolution,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            ),
            data=data,
        )

    def test_occupancy_readiness_accepts_optimized_non_empty_map(self):
        msg = self._occupancy(3, 2, 0.05, [-1, 0, 0, 100, -1, 0])

        ready, summary = map_ops._occupancy_sample_ready(msg)

        self.assertTrue(ready)
        self.assertIn("3x2@0.050000", summary)
        self.assertIn("known=4", summary)

    def test_occupancy_readiness_rejects_unknown_or_malformed_grid(self):
        unknown = self._occupancy(3, 2, 0.05, [-1] * 6)
        malformed = self._occupancy(3, 2, 0.05, [0] * 5)
        no_resolution = self._occupancy(3, 2, 0.0, [0] * 6)

        self.assertFalse(map_ops._occupancy_sample_ready(unknown)[0])
        self.assertFalse(map_ops._occupancy_sample_ready(malformed)[0])
        self.assertFalse(map_ops._occupancy_sample_ready(no_resolution)[0])


if __name__ == "__main__":
    unittest.main()
