from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages" / "semantic_intent_router"))
sys.path.insert(0, str(ROOT / "packages" / "semantic_navigation"))

from semantic_intent_router.server import (  # noqa: E402
    CANCEL_CAPABILITY,
    CANCEL_CONTRACT,
    FEEDBACK_PREFIX,
    NAV_CAPABILITY,
    NAV_CONTRACT,
    STATUS_CAPABILITY,
    STATUS_CONTRACT,
    build_server,
    decide,
)
from semantic_navigation.core import LandmarkStore  # noqa: E402


def store(*, verified: bool = True, two_landmarks: bool = False) -> LandmarkStore:
    landmarks = [
        {
            "id": "vending_machine_front",
            "name": "自动售货机",
            "aliases": ["售货机", "自动贩卖机"],
            "verified": verified,
            "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
        }
    ]
    if two_landmarks:
        landmarks.append(
            {
                "id": "lab_door_front",
                "name": "实验室大门",
                "aliases": ["大门"],
                "verified": True,
                "pose": {"x": -1.0, "y": 0.5, "yaw": 3.0},
            }
        )
    return LandmarkStore.from_mapping(
        {
            "schema_version": 2,
            "map_id": "lab_go2",
            "map_generation": 7,
            "frame_id": "map",
            "landmarks": landmarks,
        }
    )


def system_message(*caps: str) -> dict:
    lines = "\n".join(f"- capability_name: {cap}" for cap in caps)
    return {"role": "system", "content": lines}


def leaf(contract: str, output: dict, *, success: bool = True) -> dict:
    payload = {
        "leaf_result": {
            "contract_id": contract,
            "success": success,
            "output": output,
        }
    }
    return {
        "role": "user",
        "content": FEEDBACK_PREFIX + json.dumps(payload, ensure_ascii=False),
    }


def nav_output(
    run_id: str = "semantic-run-1", target: str = "自动售货机"
) -> dict:
    return {
        "accepted": True,
        "run_id": run_id,
        "detail": json.dumps(
            {"semantic_run_id": run_id, "landmark": target},
            ensure_ascii=False,
        ),
    }


def status_output(
    state: str,
    run_id: str = "semantic-run-1",
    target: str = "自动售货机",
) -> dict:
    return {
        "known": True,
        "state": state,
        "detail": json.dumps(
            {"semantic_run_id": run_id, "landmark": target},
            ensure_ascii=False,
        ),
    }


def provider_terminal_output(
    state: str,
    run_id: str = "semantic-run-1",
    target: str = "自动售货机",
) -> dict:
    return {
        "semantic_run_id": run_id,
        "state": state,
        "landmark": target,
        "remote_state": state,
        "remote_terminal": True,
    }


def call(result: dict) -> dict:
    return result["rtdl"]["children"][0]


class DecisionTests(unittest.TestCase):
    def system(self, *caps: str) -> dict:
        return system_message(
            *(caps or (NAV_CAPABILITY, STATUS_CAPABILITY, CANCEL_CAPABILITY))
        )

    def base(self) -> list[dict]:
        return [
            self.system(),
            {"role": "user", "content": "走到前面自动售货机那里"},
        ]

    def test_verified_landmark_dispatches_exact_live_capability(self) -> None:
        result = decide(self.base(), store()).envelope
        self.assertEqual(call(result)["cap"], NAV_CAPABILITY)
        self.assertEqual(call(result)["args"], {"name": "自动售货机"})
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_preview_recognizes_unverified_target_without_any_capability_leaf(self) -> None:
        result = decide(
            self.base(),
            store(verified=False),
            execution_mode="preview",
        ).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertEqual(result["task_update"]["goal"], "语义导航预览：自动售货机")
        self.assertEqual(result["task_update"]["status"], "done")
        self.assertIn("GO2_ALLOW_MOTION=false", result["content"])
        self.assertIn("尚无物理验证 approach Pose", result["content"])
        self.assertIn("未调用 Robonix navigation", result["content"])

    def test_preview_rejects_unknown_target_without_any_capability_leaf(self) -> None:
        messages = [self.system(), {"role": "user", "content": "去窗边"}]
        result = decide(
            messages,
            store(),
            execution_mode="preview",
        ).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertIsNone(result["task_update"])
        self.assertIn("未找到唯一语义目标", result["content"])

    def test_unknown_execution_mode_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "preview or live"):
            decide(self.base(), store(), execution_mode="maybe")

    def test_unverified_landmark_never_dispatches(self) -> None:
        result = decide(self.base(), store(verified=False)).envelope
        self.assertEqual(result["rtdl"]["children"], [])

    def test_unknown_landmark_never_dispatches(self) -> None:
        messages = [self.system(), {"role": "user", "content": "去窗边"}]
        result = decide(messages, store()).envelope
        self.assertEqual(result["rtdl"]["children"], [])

    def test_initial_navigation_requires_nav_status_and_cancel(self) -> None:
        for missing in (NAV_CAPABILITY, STATUS_CAPABILITY, CANCEL_CAPABILITY):
            caps = {
                NAV_CAPABILITY,
                STATUS_CAPABILITY,
                CANCEL_CAPABILITY,
            } - {missing}
            messages = [self.system(*sorted(caps)), self.base()[1]]
            with self.subTest(missing=missing):
                result = decide(messages, store()).envelope
                self.assertEqual(result["rtdl"]["children"], [])

    def test_accepted_navigation_polls_status_by_returned_id(self) -> None:
        result = decide(
            self.base() + [leaf(NAV_CONTRACT, nav_output())], store()
        ).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertEqual(call(result)["args"], {"run_id": "semantic-run-1"})

    def test_running_navigate_completion_polls_status_by_detail_id(self) -> None:
        result = decide(
            self.base() + [leaf(NAV_CONTRACT, status_output("RUNNING"))],
            store(),
        ).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertEqual(call(result)["args"], {"run_id": "semantic-run-1"})

    def test_succeeded_navigate_completion_is_terminal_success(self) -> None:
        result = decide(
            self.base() + [leaf(NAV_CONTRACT, status_output("SUCCEEDED"))],
            store(),
        ).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertEqual(result["task_update"]["status"], "done")
        self.assertIn("SUCCEEDED", result["content"])
        self.assertIn("已到达目标并停止", result["content"])
        self.assertNotIn("拒绝", result["content"])

    def test_provider_terminal_failed_navigate_leaf_is_done_without_hold(self) -> None:
        for state in ("FAILED", "CANCELED"):
            with self.subTest(state=state):
                result = decide(
                    self.base()
                    + [
                        leaf(
                            NAV_CONTRACT,
                            provider_terminal_output(state),
                            success=False,
                        )
                    ],
                    store(),
                ).envelope
                self.assertEqual(result["rtdl"]["children"], [])
                self.assertEqual(result["task_update"]["status"], "done")
                self.assertIn(state, result["content"])
                self.assertIn("任务已终止", result["content"])
                self.assertNotIn("safety hold", result["rtdl_description"])

    def test_only_explicit_false_navigation_ack_is_called_rejected(self) -> None:
        rejected = decide(
            self.base()
            + [
                leaf(
                    NAV_CONTRACT,
                    {"accepted": False, "run_id": "", "detail": "rejected"},
                )
            ],
            store(),
        ).envelope
        self.assertIn("明确拒绝", rejected["content"])

        missing = decide(
            self.base() + [leaf(NAV_CONTRACT, {"detail": "opaque"})],
            store(),
        ).envelope
        self.assertNotIn("拒绝", missing["content"])
        self.assertIn("缺少可验证", missing["content"])
        self.assertEqual(missing["task_update"]["status"], "in_progress")

    def test_running_status_recovers_run_id_and_polls_again(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            leaf(STATUS_CONTRACT, status_output("RUNNING")),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertEqual(call(result)["args"]["run_id"], "semantic-run-1")

    def test_status_detail_alone_survives_compacted_navigate_leaf(self) -> None:
        messages = self.base() + [
            leaf(STATUS_CONTRACT, status_output("RUNNING", "durable-run"))
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertEqual(call(result)["args"], {"run_id": "durable-run"})

    def test_succeeded_status_is_terminal(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            leaf(STATUS_CONTRACT, status_output("SUCCEEDED")),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertEqual(result["task_update"]["status"], "done")
        self.assertIn("SUCCEEDED", result["content"])

    def test_second_same_command_after_old_success_starts_new_run(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            leaf(STATUS_CONTRACT, status_output("SUCCEEDED")),
            {"role": "user", "content": "再去一次自动售货机"},
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], NAV_CAPABILITY)

    def test_stop_active_run_emits_cancel(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "停止"},
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], CANCEL_CAPABILITY)
        self.assertEqual(call(result)["args"], {"run_id": "semantic-run-1"})
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_cancel_accepted_polls_until_provider_terminal(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "停止"},
            leaf(CANCEL_CONTRACT, {"accepted": True, "detail": "submitted"}),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_cancel_is_done_only_after_provider_terminal(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "停止"},
            leaf(CANCEL_CONTRACT, {"accepted": True, "detail": "submitted"}),
            leaf(STATUS_CONTRACT, status_output("CANCELED")),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertEqual(result["task_update"]["status"], "done")
        self.assertIn("CANCELED", result["content"])

    def test_status_rpc_failure_while_active_emits_cancel_not_done(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            leaf(STATUS_CONTRACT, {}, success=False),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], CANCEL_CAPABILITY)
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_malformed_status_while_active_emits_cancel_not_done(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            leaf(STATUS_CONTRACT, {"known": True, "state": "ALIEN"}),
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], CANCEL_CAPABILITY)
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_unparseable_feedback_with_known_run_emits_cancel(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": FEEDBACK_PREFIX + "not-json"},
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], CANCEL_CAPABILITY)
        self.assertEqual(result["task_update"]["status"], "in_progress")

    def test_active_same_target_new_command_polls_existing_run(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "继续去自动售货机"},
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)

    def test_unknown_steer_keeps_polling_old_run(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "看看窗外"},
        ]
        result = decide(messages, store()).envelope
        self.assertEqual(call(result)["cap"], STATUS_CAPABILITY)
        self.assertIn("不下发新目标", result["content"])

    def test_target_switch_cancels_then_waits_then_starts_new_target(self) -> None:
        messages = self.base() + [
            leaf(NAV_CONTRACT, nav_output()),
            {"role": "user", "content": "去实验室大门"},
        ]
        first = decide(messages, store(two_landmarks=True)).envelope
        self.assertEqual(call(first)["cap"], CANCEL_CAPABILITY)

        messages.append(
            leaf(CANCEL_CONTRACT, {"accepted": True, "detail": "submitted"})
        )
        waiting = decide(messages, store(two_landmarks=True)).envelope
        self.assertEqual(call(waiting)["cap"], STATUS_CAPABILITY)

        messages.append(leaf(STATUS_CONTRACT, status_output("CANCELED")))
        switched = decide(messages, store(two_landmarks=True)).envelope
        self.assertEqual(call(switched)["cap"], NAV_CAPABILITY)
        self.assertEqual(call(switched)["args"], {"name": "实验室大门"})

    def test_multi_target_command_is_rejected(self) -> None:
        messages = [
            self.system(),
            {"role": "user", "content": "去自动售货机再去实验室大门"},
        ]
        result = decide(messages, store(two_landmarks=True)).envelope
        self.assertEqual(result["rtdl"]["children"], [])
        self.assertIn("ambiguous", result["content"])

    def test_non_loopback_bind_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "landmarks.yaml"
            path.write_text(
                "schema_version: 2\nmap_id: lab\nmap_generation: 1\nframe_id: map\n"
                "landmarks:\n  - id: x\n    name: X\n    verified: true\n"
                "    pose: {x: 0, y: 0, yaw: 0}\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "loopback"):
                build_server("0.0.0.0", 0, path)


if __name__ == "__main__":
    unittest.main()
