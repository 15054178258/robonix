# SPDX-License-Identifier: Apache-2.0
"""Greeting loop controller.

Owns the runtime of the passerby-greeting skill: subscribe to the robot's
front RGB camera, run the cheap local YOLO detector on every cycle, and ONLY
when it sees people escalate to the VLM for a customised line, then speak it
through the speech service. A cooldown stops the robot from talking over
itself while the same people stay in frame.

The base is never touched — the only outward action is `speech/speak` over MCP.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

import numpy as np

from .detector import PersonDetector
from .vlm import VlmGreeter, fallback_greeting

log = logging.getLogger("greet_skill")


def _image_to_rgb(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image → HxWx3 uint8 RGB. Handles rgb8/bgr8; returns None
    for anything else so the loop just skips the frame."""
    enc = (msg.encoding or "").lower()
    if enc not in ("rgb8", "bgr8"):
        return None
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    try:
        arr = arr.reshape(msg.height, msg.width, 3)
    except ValueError:
        return None
    if enc == "bgr8":
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr)


def _rgb_to_jpeg(rgb: np.ndarray) -> Optional[bytes]:
    import cv2  # ships with ultralytics (opencv-python)
    ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return buf.tobytes() if ok else None


class GreetController:
    def __init__(self, *, camera_topic: str, speak_endpoint: str,
                 detector: PersonDetector, greeter: VlmGreeter,
                 motion_monitor,
                 speak_target: str = "", period_s: float = 1.5,
                 cooldown_s: float = 15.0):
        self.camera_topic = camera_topic
        self.speak_endpoint = speak_endpoint
        self.detector = detector
        self.greeter = greeter
        self.motion_monitor = motion_monitor
        self.speak_target = speak_target
        self.period_s = period_s
        self.cooldown_s = cooldown_s

        self._ros = None
        self._node = None
        self._sub = None
        self._spin_thread: Optional[threading.Thread] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest = None  # type: Optional[np.ndarray]
        self._mcp_client: Optional[Any] = None
        self._last_spoke = 0.0
        self._state = "idle"
        self._last_line = ""
        self._run_id = ""
        self._run_generation = 0

    # ── runtime ──────────────────────────────────────────────────────────────
    def start_runtime(self) -> None:
        """Bring up the rclpy node + camera subscription in a spin thread."""
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init(args=None)
        self._ros = rclpy
        self._node = Node("greet_skill_camera")
        self._sub = self._node.create_subscription(
            Image, self.camera_topic, self._on_image, qos_profile_sensor_data
        )
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        log.info("camera subscription up on %s", self.camera_topic)
        # Requirement: ALWAYS list_speakers first and pick a real on-robot
        # audio primitive — never the macOS bridge (a dev relay).
        self.pick_speaker()

    def _spin(self) -> None:
        while not self._stop.is_set() and self._ros.ok():
            self._ros.spin_once(self._node, timeout_sec=0.1)

    def _on_image(self, msg) -> None:
        rgb = _image_to_rgb(msg)
        if rgb is not None:
            with self._frame_lock:
                self._latest = rgb

    # ── greeting loop ────────────────────────────────────────────────────────
    def start(self) -> str:
        """Start the greeting loop (idempotent) and return a run_id for the
        async status/cancel handshake the executor requires."""
        import uuid
        if self._state == "running" and self._loop_thread and self._loop_thread.is_alive():
            return self._run_id
        self._run_generation += 1
        generation = self._run_generation
        self._state = "running"
        self._run_id = uuid.uuid4().hex[:8]
        self._loop_thread = threading.Thread(
            target=self._loop, args=(generation,), daemon=True
        )
        self._loop_thread.start()
        return self._run_id

    def _loop(self, generation: int) -> None:
        while (self._state == "running"
               and generation == self._run_generation
               and not self._stop.is_set()):
            time.sleep(self.period_s)
            if not self.motion_monitor.is_moving():
                continue
            with self._frame_lock:
                frame = None if self._latest is None else self._latest.copy()
            if frame is None:
                continue
            # Stage 1 — cheap local YOLO. No people → skip, no paid call.
            count = self.detector.count_people(frame)
            if count <= 0:
                continue
            # Cooldown so we don't talk over the same crowd every cycle.
            if time.time() - self._last_spoke < self.cooldown_s:
                continue
            # Stage 2 — VLM customises the line; falls back to a template.
            jpeg = _rgb_to_jpeg(frame)
            line = (self.greeter.greet(jpeg, count) if jpeg
                    else fallback_greeting(count))
            self._last_line = line
            self._speak(line)
            self._last_spoke = time.time()
            log.info("greeted %d people: %s", count, line)

    # ── speech MCP: list_speakers + speak (never touches the base) ───────────
    async def _mcp_call(self, tool: str, args: dict) -> dict:
        from fastmcp import Client
        import json
        async with Client(self.speak_endpoint) as c:
            result = await c.call_tool(tool, args)
            if not result.content:
                return {}
            txt = result.content[0].text
            try:
                return json.loads(txt)
            except Exception:  # noqa: BLE001
                return {"raw": txt}

    def _mcp_call_sync(self, tool: str, args: dict) -> dict:
        import asyncio
        try:
            return asyncio.run(self._mcp_call(tool, args))
        except Exception as e:  # noqa: BLE001
            log.warning("mcp %s failed: %s", tool, e)
            return {}

    def pick_speaker(self) -> None:
        """Always call list_speakers first, then pick a real on-robot audio
        primitive — explicitly skipping the macOS bridge (a dev relay). Falls
        back to '' (first available) only if nothing else is registered."""
        import json
        resp = self._mcp_call_sync("list_speakers", {"namespace_prefix": ""})
        raw = resp.get("speakers_json", "[]")
        try:
            speakers = json.loads(raw) if isinstance(raw, str) else (raw or [])
        except Exception:  # noqa: BLE001
            speakers = []
        log.info("list_speakers → %s", [s.get("provider_id") for s in speakers])
        chosen = ""
        for s in speakers:
            blob = " ".join(str(s.get(k, "")) for k in
                            ("provider_id", "namespace", "description")).lower()
            if "macos" in blob or "bridge" in blob:
                continue
            chosen = str(s.get("provider_id", ""))
            break
        self.speak_target = chosen
        log.info("speak_target = %r", chosen or "(first available)")

    def _speak(self, text: str) -> None:
        self._mcp_call_sync("speak", {"target": self.speak_target, "text": text})

    # ── status / teardown ────────────────────────────────────────────────────
    def status(self, run_id=None):
        """Canonical async status of a LONG-LIVED task. While the watch is up it
        reports RUNNING — never SUCCEEDED — so the executor keeps monitoring it
        and the greet tree stays live in the forest, letting other RTDL branches
        run in parallel. It only reaches a terminal state (CANCELED) on cancel.
        Returns None for an unknown run_id so the caller can report 'no such run'."""
        if run_id and run_id != self._run_id:
            return None
        if self._state == "running":
            return {"state": "RUNNING",
                    "detail": f"greeting watch active; last: {self._last_line}"}
        if self._state == "canceled":
            return {"state": "CANCELED", "detail": "greeting watch stopped"}
        return {"state": "PENDING", "detail": self._state}

    def cancel(self, run_id=None):
        """Stop the greeting loop (camera stays subscribed; greet() resumes).
        Idempotent."""
        if run_id and run_id != self._run_id:
            return False, "no such run"
        self._state = "canceled"
        self._run_generation += 1
        self._loop_thread = None
        return True, "greeting watch stopped"

    def stop_runtime(self) -> None:
        self._stop.set()
        self._run_generation += 1
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        self.motion_monitor.close()
