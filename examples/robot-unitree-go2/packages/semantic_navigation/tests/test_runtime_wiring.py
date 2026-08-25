from __future__ import annotations

import ast
from pathlib import Path
import unittest


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "semantic_navigation"
    / "service.py"
)


def function_source(name: str) -> str:
    source = SOURCE_PATH.read_text(encoding="utf-8")
    module = ast.parse(source)
    function = next(
        node
        for node in module.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    )
    return ast.get_source_segment(source, function) or ""


class RuntimeWiringTest(unittest.TestCase):
    def test_activate_starts_latched_lifecycle_and_status_workers(self) -> None:
        source = function_source("activate")
        for required in (
            "_start_status_notifier()",
            "_start_navigation_monitors()",
            "subscriber.start()",
            "guard.wait_for_sample",
            "guard.require_ready()",
            "_assert_mapping_provider_live(topic)",
        ):
            self.assertIn(required, source)

    def test_deactivate_and_shutdown_stop_and_join_all_workers(self) -> None:
        for function_name in ("deactivate", "shutdown"):
            source = function_source(function_name)
            with self.subTest(function=function_name):
                for required in (
                    "_stop_lifecycle_binding",
                    "_wait_inflight_dispatches",
                    "_wait_cancel_workers",
                    "_stop_navigation_monitors",
                    "_stop_status_notifier",
                ):
                    self.assertIn(required, source)

    def test_every_goal_rechecks_lifecycle_and_gets_terminal_monitor(self) -> None:
        source = function_source("navigate_landmark")
        self.assertIn("_require_lifecycle_ready()", source)
        self.assertIn("expected_generation=active.generation", source)
        self.assertIn("_schedule_navigation_monitor(semantic_run_id)", source)

        # The second lifecycle check and single-flight reservation must share
        # the same global lock used by shutdown.
        locked_blocks = []
        module = ast.parse(source)
        for node in ast.walk(module):
            if not isinstance(node, ast.With):
                continue
            if any(
                isinstance(item.context_expr, ast.Name)
                and item.context_expr.id == "_lock"
                for item in node.items
            ):
                locked_blocks.append(ast.get_source_segment(source, node) or "")
        self.assertTrue(
            any(
                "admission.validate_current" in block and "_runs.reserve" in block
                for block in locked_blocks
            )
        )

    def test_cancel_requires_provider_terminal_confirmation(self) -> None:
        source = function_source("landmark_cancel")
        self.assertIn("if not run.nav_run_id", source)
        self.assertIn("if run.remote_terminal", source)
        self.assertIn("if run.cancel_started", source)
        self.assertIn("_schedule_cancel", source)

        monitor = function_source("_monitor_navigation_status")
        self.assertIn("if run.remote_terminal", monitor)
        self.assertNotIn("if run.invalidated:\n            return", monitor)

    def test_every_status_detail_echoes_durable_semantic_run_id(self) -> None:
        source = function_source("_run_detail")
        self.assertIn('"semantic_run_id": run.semantic_run_id', source)

    def test_lifecycle_transition_serializes_with_dispatch_reservation(self) -> None:
        source = function_source("_on_lifecycle_transition")
        self.assertIn("with _lock", source)
        self.assertIn("_invalidate_nonterminal_runs(reason)", source)


if __name__ == "__main__":
    unittest.main()
