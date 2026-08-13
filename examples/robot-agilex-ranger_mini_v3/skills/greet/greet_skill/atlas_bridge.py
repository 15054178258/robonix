# SPDX-License-Identifier: Apache-2.0
"""greet_rbnx — passerby-greeting skill (atlas bridge).

A user-invocable skill: once started it watches the front RGB camera, runs a
cheap local YOLO person detector every cycle, and only when people appear asks
a VLM (ofox + a cheap Qwen vision model) for ONE friendly, witty line asking
folks to make way — then speaks it through speech/speak. A cooldown avoids
talking over the same crowd. The robot base is never driven; the only outward
action is speech.

Lifecycle mirrors explore: on_init is light; on_activate (fired on the first
MCP call) resolves camera + speak from atlas and starts the loop; on_deactivate
tears it down.
"""
from __future__ import annotations

import logging
import time

from robonix_api import Skill, Ok, Err, ATLAS  # noqa: E402

# No basicConfig: robonix_api configures the logging handlers (scribe) on
# import, exactly like explore/mapping. We just take a named child logger.
log = logging.getLogger("greet_skill")

greet_skill = Skill(id="greet", namespace="robonix/skill/greet")

# Upstream contracts this skill consumes (resolved from atlas — no hardcoded
# topics). camera/rgb is a ros2 image topic; speech/speak is an MCP tool.
REQUIRED_INPUTS = {
    "camera": ("robonix/primitive/camera/rgb", "ros2"),
    "speak":  ("robonix/service/speech/speak", "mcp"),
    "soma":   ("robonix/system/soma/get_health", "grpc"),
}

ctrl = None      # GreetController
_cfg: dict = {}  # snapshot of the deploy config from on_init


def resolve_inputs(*, camera_provider_id: str, soma_provider_id: str,
                   deadline_s: float = 5.0) -> dict[str, str]:
    """Resolve the exact configured camera and speech providers.

    Camera contracts are intentionally not resolved by contract alone: a robot
    may expose several RGB cameras with the same contract.  Keeping the
    provider selector in deploy config makes the physical sensor choice
    explicit and stable.
    """
    camera_provider_id = camera_provider_id.strip()
    if not camera_provider_id:
        raise RuntimeError(
            "greet skill requires config.camera_provider_id; refusing to pick "
            "an arbitrary camera when multiple providers may be present"
        )
    soma_provider_id = soma_provider_id.strip()
    if not soma_provider_id:
        raise RuntimeError("greet skill requires config.soma_provider_id")

    resolved: dict[str, str] = {}
    last_errors: dict[str, str] = {}
    deadline = time.monotonic() + max(0.0, deadline_s)
    first_attempt = True
    while first_attempt or time.monotonic() < deadline:
        first_attempt = False
        for key, (cid, transport) in REQUIRED_INPUTS.items():
            if key in resolved:
                continue
            provider_id = {
                "camera": camera_provider_id,
                "soma": soma_provider_id,
            }.get(key, "")
            try:
                cap = ATLAS.find_unique_capability(
                    contract_id=cid,
                    transport=transport,
                    provider_id=provider_id,
                )
                ch = greet_skill.connect_capability(cap, cid, transport)
            except Exception as exc:  # noqa: BLE001
                last_errors[key] = str(exc)
                continue
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
                selected = f" provider={provider_id}" if provider_id else ""
                log.info("resolved %s [%s]%s → %s", cid, transport, selected, ep)
            else:
                last_errors[key] = "Atlas returned an empty endpoint"
        if len(resolved) == len(REQUIRED_INPUTS):
            return resolved
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(0.5, remaining))

    failures = []
    for key, (cid, transport) in REQUIRED_INPUTS.items():
        if key in resolved:
            continue
        provider_id = {
            "camera": camera_provider_id,
            "soma": soma_provider_id,
        }.get(key, "<unique>")
        reason = last_errors.get(key, "not found")
        failures.append(
            f"{key}(provider={provider_id}, contract={cid}, "
            f"transport={transport}): {reason}"
        )
    raise RuntimeError("greet skill could not resolve atlas inputs: " + "; ".join(failures))


from greet_mcp import (  # noqa: E402
    Greet_Request, Greet_Response,
    GetGreetStatus_Request, GetGreetStatus_Response,
    CancelGreet_Request, CancelGreet_Response,
)


@greet_skill.mcp("robonix/skill/greet/greet")
def greet(req: Greet_Request) -> Greet_Response:
    """Start the passerby-greeting watch (async). Returns a run_id. The watch is
    LONG-LIVED: status() stays RUNNING while it watches (never SUCCEEDED), so the
    executor keeps monitoring it and the tree stays live in the forest — other
    RTDL branches run in parallel. Only cancel() ends it."""
    if ctrl is None:
        return Greet_Response(accepted=False, run_id="", message="controller not initialized")
    rid = ctrl.start()
    return Greet_Response(accepted=True, run_id=rid, message="greeting watch started")


@greet_skill.mcp("robonix/skill/greet/greet/status")
def status(req: GetGreetStatus_Request) -> GetGreetStatus_Response:
    """Poll the watch. Empty run_id = most recent. Long-lived task: stays RUNNING
    while active so the executor keeps monitoring it; terminal only on cancel."""
    if ctrl is None:
        return GetGreetStatus_Response(known=False, state="PENDING", detail="not initialized")
    s = ctrl.status(req.run_id or None)
    if s is None:
        return GetGreetStatus_Response(known=False, state="PENDING", detail="no such run")
    return GetGreetStatus_Response(known=True, state=s["state"], detail=s["detail"])


@greet_skill.mcp("robonix/skill/greet/greet/cancel")
def cancel(req: CancelGreet_Request) -> CancelGreet_Response:
    """Stop greeting (the camera stays subscribed; another greet() resumes)."""
    if ctrl is None:
        return CancelGreet_Response(ok=False, message="not initialized")
    ok, msg = ctrl.cancel(req.run_id or None)
    return CancelGreet_Response(ok=ok, message=msg)


@greet_skill.on_init
def init(cfg: dict):
    """CMD_INIT: just stash config. No atlas queries / no model load yet —
    those happen on_activate when there's actually a request to serve."""
    global _cfg
    _cfg = dict(cfg or {})
    log.info("CMD_INIT ok")
    return Ok()


@greet_skill.on_activate
def activate():
    """CMD_ACTIVATE: resolve camera + speak, load YOLO + the VLM client, bring
    up the camera subscription and start the greeting loop. Idempotent."""
    global ctrl
    if ctrl is not None:
        return Ok()

    vlm = _cfg.get("vlm", {}) or {}
    base_url = vlm.get("upstream") or vlm.get("base_url")
    api_key = vlm.get("api_key")
    model = vlm.get("model", "qwen-vl-plus")
    if not base_url or not api_key:
        return Err("greet skill: vlm.upstream / vlm.api_key missing in config")

    try:
        inputs = resolve_inputs(
            camera_provider_id=str(_cfg.get("camera_provider_id", "")),
            soma_provider_id=str(_cfg.get("soma_provider_id", "")),
            deadline_s=float(_cfg.get("dependency_timeout_s", 5.0)),
        )
        from .detector import PersonDetector
        from .vlm import VlmGreeter
        from .controller import GreetController
        from .body_state import SomaMotionMonitor

        detector = PersonDetector(
            weights=_cfg.get("yolo_weights"),
            conf=float(_cfg.get("yolo_conf", 0.4)),
            # skip people too far/small to be in the robot's way
            min_box_frac=float(_cfg.get("yolo_min_box_frac", 0.30)),
        )
        greeter = VlmGreeter(base_url=base_url, api_key=api_key, model=model)
        motion_monitor = SomaMotionMonitor(
            endpoint=inputs["soma"],
            chassis_provider_id=str(_cfg.get("chassis_provider_id", "")),
            timeout_s=float(_cfg.get("soma_timeout_s", 0.5)),
        )
        ctrl = GreetController(
            camera_topic=inputs["camera"],
            speak_endpoint=inputs["speak"],
            detector=detector,
            greeter=greeter,
            motion_monitor=motion_monitor,
            speak_target=_cfg.get("speak_target", ""),
            period_s=float(_cfg.get("period_s", 1.5)),
            cooldown_s=float(_cfg.get("cooldown_s", 15.0)),
        )
        ctrl.start_runtime()
    except Exception as e:  # noqa: BLE001
        return Err(f"greet skill activate failed: {e}")
    log.info("CMD_ACTIVATE ok — greeting watch running")
    return Ok()


@greet_skill.on_deactivate
def deactivate():
    global ctrl
    if ctrl is None:
        return Ok()
    try:
        ctrl.stop_runtime()
    finally:
        ctrl = None
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    greet_skill.run()
    if ctrl is not None:
        ctrl.stop_runtime()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
