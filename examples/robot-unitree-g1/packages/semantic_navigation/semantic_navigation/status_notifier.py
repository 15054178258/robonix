"""Optional, loopback-only semantic task display notifications."""

from __future__ import annotations

import ipaddress
import json
import logging
import queue
import threading
from typing import Callable
from urllib import error, parse, request


log = logging.getLogger("semantic_navigation.status_notifier")

_EXPECTED_PATH = "/api/semantic-task"
_STOP = object()


class StatusEndpointError(ValueError):
    pass


def validate_loopback_endpoint(value: object) -> str:
    endpoint = str(value or "").strip()
    if not endpoint:
        return ""
    try:
        parsed = parse.urlsplit(endpoint)
        port = parsed.port
    except ValueError as exc:
        raise StatusEndpointError(f"invalid semantic status endpoint: {exc}") from exc
    if parsed.scheme != "http":
        raise StatusEndpointError("semantic status endpoint must use http")
    if parsed.username is not None or parsed.password is not None:
        raise StatusEndpointError("semantic status endpoint must not contain credentials")
    if parsed.hostname is None or port is None:
        raise StatusEndpointError("semantic status endpoint requires a literal host and port")
    if port == 0:
        raise StatusEndpointError("semantic status endpoint port must be within [1, 65535]")
    try:
        host = ipaddress.ip_address(parsed.hostname)
    except ValueError as exc:
        raise StatusEndpointError(
            "semantic status endpoint host must be a literal loopback IP"
        ) from exc
    if not host.is_loopback:
        raise StatusEndpointError("semantic status endpoint must be loopback-only")
    if parsed.path != _EXPECTED_PATH or parsed.query or parsed.fragment:
        raise StatusEndpointError(
            f"semantic status endpoint path must be exactly {_EXPECTED_PATH}"
        )
    return endpoint


class _NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def _post_json(endpoint: str, payload: dict, timeout_s: float) -> None:
    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    req = request.Request(
        endpoint,
        data=body,
        method="POST",
        headers={"Content-Type": "application/json", "Accept": "application/json"},
    )
    opener = request.build_opener(request.ProxyHandler({}), _NoRedirect())
    with opener.open(req, timeout=timeout_s) as response:
        if not 200 <= int(response.status) < 300:
            raise error.HTTPError(
                endpoint,
                int(response.status),
                "dashboard status update rejected",
                response.headers,
                None,
            )


class LoopbackStatusNotifier:
    def __init__(
        self,
        endpoint: object,
        timeout_s: float = 0.5,
        *,
        sender: Callable[[str, dict, float], None] = _post_json,
    ) -> None:
        self.endpoint = validate_loopback_endpoint(endpoint)
        timeout_s = float(timeout_s)
        if not 0.05 <= timeout_s <= 2.0:
            raise StatusEndpointError("semantic status timeout must be within [0.05, 2.0]")
        self.timeout_s = timeout_s
        self._sender = sender
        self._queue: queue.Queue = queue.Queue(maxsize=64)
        self._thread: threading.Thread | None = None
        self._state_lock = threading.Lock()
        self._accepting = False

    @property
    def enabled(self) -> bool:
        return bool(self.endpoint)

    @property
    def is_alive(self) -> bool:
        with self._state_lock:
            return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        with self._state_lock:
            if not self.enabled or self._thread is not None:
                return
            self._accepting = True
            self._thread = threading.Thread(
                target=self._run,
                name="semantic-status-notifier",
                daemon=True,
            )
            self._thread.start()

    def notify(self, payload: dict) -> None:
        with self._state_lock:
            if (
                not self.enabled
                or not self._accepting
                or self._thread is None
                or not self._thread.is_alive()
            ):
                return
            try:
                self._queue.put_nowait(dict(payload))
            except queue.Full:
                log.warning("semantic status queue full; dropping display update")

    def stop(self, timeout_s: float = 2.0) -> None:
        with self._state_lock:
            thread = self._thread
            self._accepting = False
            if thread is None:
                return
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                break
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            log.error("cannot enqueue semantic status shutdown sentinel")
        if thread is not threading.current_thread():
            thread.join(timeout=max(timeout_s, self.timeout_s + 0.5))
            if thread.is_alive():
                log.error("semantic status worker did not stop cleanly")
        with self._state_lock:
            self._thread = None

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _STOP:
                    return
                try:
                    self._sender(self.endpoint, item, self.timeout_s)
                except Exception as exc:
                    log.warning("dashboard semantic status update failed: %s", exc)
            finally:
                self._queue.task_done()
