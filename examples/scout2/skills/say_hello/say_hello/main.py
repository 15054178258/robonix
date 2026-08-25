from robonix_api import ATLAS, Skill, Ok, Err
from say_hello_mcp import SayHello_Request, SayHello_Response
from fastmcp import Client
import asyncio
import json
import logging
import time
from concurrent.futures import ThreadPoolExecutor

log = logging.getLogger("say_hello")
skill = Skill(id="say_hello", namespace="robonix/skill/say_hello")

_speak_endpoint: str = ""


def resolve_speak_endpoint(deadline_s: float = 30.0) -> str:
    """通过 Atlas 发现 speech/speak MCP 端点"""
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        try:
            cap = ATLAS.find_unique_capability(
                contract_id="robonix/service/speech/speak",
                transport="mcp",
            )
            ch = skill.connect_capability(
                cap, "robonix/service/speech/speak", "mcp"
            )
            ep = ch.endpoint
            ch.close()
            if ep:
                log.info("resolved speech/speak -> %s", ep)
                return ep
        except Exception:
            pass
        time.sleep(2.0)
    raise RuntimeError(
        "say_hello: could not find robonix/service/speech/speak on Atlas. "
        "Is the speech service running?"
    )


async def _speak(text: str) -> dict:
    async with Client(_speak_endpoint) as c:
        result = await c.call_tool("speak", {"target": "", "text": text})
        if not result.content:
            return {}
        return json.loads(result.content[0].text)


def speak_sync(text: str) -> dict:
    """Run async _speak in a separate thread to avoid event-loop conflict."""
    # Auto-resolve endpoint if not yet initialized
    global _speak_endpoint
    if not _speak_endpoint:
        try:
            _speak_endpoint = resolve_speak_endpoint()
        except RuntimeError as e:
            log.error("failed to resolve speech endpoint: %s", e)
            return {"ok": False, "detail": str(e)}
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(asyncio.run, _speak(text))
        return future.result()


@skill.on_init
def init(cfg: dict):
    return Ok()


@skill.on_activate
def activate():
    global _speak_endpoint
    _speak_endpoint = resolve_speak_endpoint()
    log.info("say_hello activated, speak endpoint ready")
    return Ok()


@skill.on_deactivate
def deactivate():
    global _speak_endpoint
    _speak_endpoint = ""
    return Ok()


@skill.mcp("robonix/skill/say_hello/say")
def say(req: SayHello_Request) -> SayHello_Response:
    """Speak text through the robot speaker.

    Uses 'text' if provided, otherwise falls back to greeting with 'name'.
    """
    text = (req.text or "").strip()
    if not text and (req.name or "").strip():
        text = f"你好，{req.name.strip()}！"
    if not text:
        text = "你好！"

    result = speak_sync(text)
    ok = result.get("ok", False)
    if not ok:
        log.warning("speak failed: %s", result.get("detail", "unknown"))

    return SayHello_Response(greeting=text)


if __name__ == "__main__":
    skill.run()
