# SPDX-License-Identifier: MulanPSL-2.0
"""Checks for Scene's build and source-level capability contracts."""

import ast
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class SceneBuildConfigTest(unittest.TestCase):
    def test_every_scene_manifest_uses_canonical_implicit_shared_driver(self):
        for filename in (
            "package_manifest.yaml",
            "package_manifest.jetson-native.yaml",
            "package_manifest.jetson-docker.yaml",
        ):
            manifest = yaml.safe_load((ROOT / filename).read_text())
            capability_names = {
                capability["name"] for capability in manifest["capabilities"]
            }
            self.assertNotIn("robonix/lifecycle/driver", capability_names, filename)
            self.assertNotIn("robonix/system/scene/driver", capability_names, filename)

    def test_scene_wires_explicit_lifecycle_handlers(self):
        tree = ast.parse((ROOT / "scene_service" / "service.py").read_text())
        decorators = {
            decorator.attr
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            for decorator in node.decorator_list
            if isinstance(decorator, ast.Attribute)
            and isinstance(decorator.value, ast.Name)
            and decorator.value.id == "scene"
        }

        self.assertTrue(
            {"on_init", "on_activate", "on_deactivate", "on_shutdown"}.issubset(
                decorators
            )
        )

    def test_scene_container_forwards_runtime_selection_and_frame_overrides(self):
        start_script = (ROOT / "scripts" / "start.sh").read_text()
        self.assertIn('-e SCENE_CAMERA_FRAME="${SCENE_CAMERA_FRAME:-}"', start_script)
        self.assertIn('-e SCENE_BASE_FRAME="${SCENE_BASE_FRAME:-}"', start_script)
        for name in (
            "SCENE_CG_MERGE_THRESHOLD",
            "SCENE_CG_MAX_MERGE_DIST_M",
            "SCENE_CG_ONE_TO_ONE_ASSOCIATION",
            "SCENE_CG_ADAPTIVE_MERGE_DISTANCE",
            "SCENE_CG_ADAPTIVE_MERGE_MIN_DIST_M",
            "SCENE_CG_ADAPTIVE_MERGE_EXTENT_SCALE",
        ):
            self.assertIn(
                f'-e {name}="${{{name}:-}}"',
                start_script,
            )

    def test_ingest_uses_the_canonical_lidar3d_contract(self):
        tree = ast.parse((ROOT / "scene_service" / "service.py").read_text())
        contracts = None
        for node in ast.walk(tree):
            if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                if node.target.id == "_SCENE_CONTRACTS":
                    contracts = ast.literal_eval(node.value)
                    break
        self.assertIsNotNone(contracts)
        by_kind = {
            kind: (contract_id, msg_type) for kind, contract_id, msg_type in contracts
        }
        self.assertEqual(
            by_kind["lidar3d"],
            ("robonix/primitive/lidar/lidar3d", "PointCloud2"),
        )

    def test_every_scene_mcp_tool_is_declared(self):
        tree = ast.parse((ROOT / "scene_service" / "service.py").read_text())
        tool_names = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Assign):
                continue
            if not any(
                isinstance(target, ast.Name) and target.id == "scene_tools"
                for target in node.targets
            ):
                continue
            tool_names = [
                item.attr
                for item in node.value.elts
                if isinstance(item, ast.Attribute)
                and isinstance(item.value, ast.Name)
                and item.value.id == "mcp_tools"
            ]
            break
        self.assertEqual(
            tool_names,
            [
                "list_objects",
                "goal_near",
                "goal_room",
                "get_scene_graph",
                "get_object_context",
                "get_robot_context",
                "list_relations",
                "update_object_label",
                "update_object_geometry",
                "delete_object",
                "flush_objects",
            ],
        )



if __name__ == "__main__":
    unittest.main()
