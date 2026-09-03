#!/usr/bin/env python3
"""Bounded, proxy-free health check for the local semantic intent endpoint."""

from __future__ import annotations

import json
import sys
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener


def main() -> int:
    if len(sys.argv) != 2:
        return 2
    url = sys.argv[1]
    parsed = urlsplit(url)
    if (
        parsed.scheme != "http"
        or parsed.hostname != "127.0.0.1"
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path != "/v1/models"
    ):
        return 2
    try:
        request = Request(url, method="GET", headers={"Accept": "application/json"})
        with build_opener(ProxyHandler({})).open(request, timeout=0.6) as response:
            if response.status != 200:
                return 1
            body = response.read(16_384)
        payload = json.loads(body.decode("utf-8"))
    except Exception:
        return 1
    if payload != {
        "object": "list",
        "data": [{"id": "go2-semantic-router", "object": "model"}],
    }:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
