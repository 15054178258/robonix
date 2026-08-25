from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
HEALTH = ROOT / "scripts" / "check_semantic_intent_health.py"
SPEC = importlib.util.spec_from_file_location("check_semantic_intent_health", HEALTH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class _Response:
    def __init__(self, payload: object, status: int = 200) -> None:
        self.status = status
        self.body = json.dumps(payload).encode("utf-8")

    def __enter__(self) -> "_Response":
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def read(self, limit: int) -> bytes:
        return self.body[:limit]


class _Opener:
    def __init__(self, response: _Response) -> None:
        self.response = response
        self.request = None
        self.timeout = None

    def open(self, request: object, timeout: float) -> _Response:
        self.request = request
        self.timeout = timeout
        return self.response


class SemanticHealthTest(unittest.TestCase):
    valid_payload = {
        "object": "list",
        "data": [{"id": "go2-semantic-router", "object": "model"}],
    }

    def run_health(self, url: str, payload: object) -> tuple[int, _Opener]:
        opener = _Opener(_Response(payload))
        with mock.patch.object(sys, "argv", [str(HEALTH), url]), mock.patch.object(
            MODULE, "build_opener", return_value=opener
        ) as build:
            status = MODULE.main()
        build.assert_called_once()
        proxy_handler = build.call_args.args[0]
        self.assertEqual(proxy_handler.proxies, {})
        return status, opener

    def test_exact_loopback_models_contract_passes(self) -> None:
        status, opener = self.run_health(
            "http://127.0.0.1:18080/v1/models", self.valid_payload
        )
        self.assertEqual(status, 0)
        self.assertEqual(opener.timeout, 0.6)

    def test_wrong_model_contract_fails_closed(self) -> None:
        status, _opener = self.run_health(
            "http://127.0.0.1:18080/v1/models", {"object": "list", "data": []}
        )
        self.assertEqual(status, 1)

    def test_noncanonical_host_or_path_is_rejected_without_io(self) -> None:
        for url in (
            "http://localhost:18080/v1/models",
            "http://127.0.0.1:18080/models",
        ):
            with self.subTest(url=url), mock.patch.object(
                sys, "argv", [str(HEALTH), url]
            ), mock.patch.object(MODULE, "build_opener") as build:
                self.assertEqual(MODULE.main(), 2)
                build.assert_not_called()


if __name__ == "__main__":
    unittest.main()
