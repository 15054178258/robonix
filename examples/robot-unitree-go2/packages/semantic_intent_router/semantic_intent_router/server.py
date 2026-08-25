"""Loopback OpenAI-compatible semantic intent endpoint for Robonix Pilot.

This is deliberately not a general-purpose language model.  It turns a user
utterance into one live Robonix semantic-navigation capability call only when
the utterance resolves to one unique, physically verified saved landmark.
Every other input produces an empty RTDL tree.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import re
import time
from typing import Any, Iterable

from semantic_navigation.core import LandmarkError, LandmarkStore, normalize_text


NAV_CONTRACT = "robonix/skill/semantic_navigation/navigate_landmark"
STATUS_CONTRACT = f"{NAV_CONTRACT}/status"
CANCEL_CONTRACT = f"{NAV_CONTRACT}/cancel"
NAV_CAPABILITY = "semantic_navigation.semantic_navigation_navigate_landmark"
STATUS_CAPABILITY = "semantic_navigation.navigate_landmark_status"
CANCEL_CAPABILITY = "semantic_navigation.navigate_landmark_cancel"
FEEDBACK_PREFIX = (
    "Executor feedback for the current RTDL leaf (not a new user request): "
)
_CAP_LINE = re.compile(r"^\s*-\s*capability_name:\s*(\S+)\s*$", re.MULTILINE)
_MAX_REQUEST_BYTES = 2 * 1024 * 1024
_TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELED"})
_NONTERMINAL_STATES = frozenset({"STARTING", "PENDING", "RUNNING"})
_KNOWN_STATES = _TERMINAL_STATES | _NONTERMINAL_STATES
_EXECUTION_MODES = frozenset({"preview", "live"})
_PREVIEW_GOAL_PREFIX = "语义导航预览："
_STOP_UTTERANCES = frozenset(
    normalize_text(value)
    for value in (
        "停",
        "停止",
        "停下",
        "停下来",
        "别走了",
        "不要走了",
        "取消",
        "取消任务",
        "取消导航",
        "停止任务",
        "停止导航",
    )
)


@dataclass(frozen=True)
class Decision:
    """One OpenAI response envelope plus an HTTP status."""

    envelope: dict[str, Any]
    status: int = 200


@dataclass(frozen=True)
class ActiveRun:
    run_id: str
    target: str
    state: str = "RUNNING"

    @property
    def terminal(self) -> bool:
        return self.state in _TERMINAL_STATES


@dataclass(frozen=True)
class Turn:
    command: str
    boundary: int
    prior_messages: list[dict[str, Any]]
    feedback_messages: list[dict[str, Any]]


def _empty_tree(description: str = "no safe semantic action") -> dict[str, Any]:
    return {
        "op": "sequence",
        "op_id": 0,
        "description": description,
        "children": [],
    }


def _task(target: str, status: str) -> dict[str, str]:
    return {
        "goal": f"导航到已保存地标：{target}",
        "success_criterion": "语义导航状态返回 SUCCEEDED，机器人已停止",
        "status": status,
    }


def _envelope(
    *,
    content: str,
    description: str,
    tree: dict[str, Any] | None = None,
    task_update: dict[str, str] | None = None,
) -> dict[str, Any]:
    return {
        "content": content,
        "rtdl_description": description,
        "rtdl": tree if tree is not None else _empty_tree(description),
        "task_update": task_update,
    }


def _message_text(message: dict[str, Any]) -> str:
    content = message.get("content")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(
            part.get("text", "")
            for part in content
            if isinstance(part, dict)
            and part.get("type") == "text"
            and isinstance(part.get("text"), str)
        )
    return ""


def advertised_capabilities(messages: Iterable[dict[str, Any]]) -> set[str]:
    """Read capability names only from trusted Pilot system messages."""

    capabilities: set[str] = set()
    for message in messages:
        if message.get("role") == "system":
            capabilities.update(_CAP_LINE.findall(_message_text(message)))
    return capabilities


def _is_feedback(message: dict[str, Any]) -> bool:
    return (
        message.get("role") == "user"
        and _message_text(message).startswith(FEEDBACK_PREFIX)
    )


def current_turn(messages: list[dict[str, Any]]) -> Turn:
    """Split history at the last real user command.

    Robonix Pilot intentionally stores executor results as user-role messages,
    but prefixes them with a fixed sentence.  Treating every user-role message
    as a fresh command lets an old result contaminate a later navigation turn.
    """

    boundary = -1
    command = ""
    for index, message in enumerate(messages):
        if message.get("role") == "user" and not _is_feedback(message):
            boundary = index
            command = _message_text(message).strip()
    if boundary < 0:
        return Turn("", len(messages), list(messages), [])
    feedback = [message for message in messages[boundary + 1 :] if _is_feedback(message)]
    return Turn(command, boundary, list(messages[:boundary]), feedback)


def _json_values(text: str) -> Iterable[Any]:
    decoder = json.JSONDecoder()
    index = 0
    while index < len(text):
        if text[index] not in "[{":
            index += 1
            continue
        try:
            value, consumed = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            index += 1
            continue
        yield value
        index += max(1, consumed)


def _walk(value: Any) -> Iterable[Any]:
    yield value
    if isinstance(value, dict):
        for child in value.values():
            if isinstance(child, str) and child.lstrip().startswith(("{", "[")):
                try:
                    child = json.loads(child)
                except json.JSONDecodeError:
                    pass
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def leaf_results(messages: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Recover executor leaf results from Pilot's user-role history."""

    results: list[dict[str, Any]] = []
    for message in messages:
        if not _is_feedback(message):
            continue
        payload = _message_text(message)[len(FEEDBACK_PREFIX) :]
        for value in _json_values(payload):
            for node in _walk(value):
                if not isinstance(node, dict):
                    continue
                leaf = node.get("leaf_result")
                if isinstance(leaf, dict):
                    results.append(leaf)
                elif {"contract_id", "success", "output"}.issubset(node):
                    results.append(node)
    return results


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}
    return {}


def _semantic_leaves(messages: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    contracts = {NAV_CONTRACT, STATUS_CONTRACT, CANCEL_CONTRACT}
    return [leaf for leaf in leaf_results(messages) if leaf.get("contract_id") in contracts]


def _detail_mapping(output: dict[str, Any]) -> dict[str, Any]:
    return _mapping(output.get("detail"))


def _target_from_output(output: dict[str, Any]) -> str:
    direct = output.get("landmark")
    if direct is not None and str(direct).strip():
        return str(direct).strip()
    detail = _detail_mapping(output)
    value = detail.get("landmark")
    return str(value).strip() if value is not None else ""


def _run_id_from_output(output: dict[str, Any]) -> str:
    """Recover the semantic run id from a response or its durable detail."""

    for key in ("run_id", "semantic_run_id"):
        direct = str(output.get(key, "")).strip()
        if direct:
            return direct
    detail = _detail_mapping(output)
    return str(detail.get("semantic_run_id", "")).strip()


def _state_from_output(output: dict[str, Any]) -> str:
    direct = str(output.get("state", "")).strip().upper()
    if direct:
        return direct
    detail = _detail_mapping(output)
    return str(detail.get("state", "")).strip().upper()


def _navigation_completion_state(output: dict[str, Any]) -> str:
    """Normalize the completion shape returned by a Robonix skill leaf.

    Executor may keep a skill leaf open until its provider task is terminal. In
    that case the navigate contract produces the provider's status-shaped
    result instead of its initial ``accepted/run_id`` response. A failed leaf
    may likewise carry a provider-confirmed terminal snapshot directly.
    """

    state = _state_from_output(output)
    if output.get("known") is True and state in _KNOWN_STATES:
        return state
    detail = _detail_mapping(output)
    remote_terminal = (
        output.get("remote_terminal") is True
        or detail.get("remote_terminal") is True
    )
    if remote_terminal and state in _TERMINAL_STATES:
        return state
    return ""


def _advance_active(
    active: ActiveRun | None,
    leaves: Iterable[dict[str, Any]],
) -> ActiveRun | None:
    """Reduce semantic leaf history to the latest provider-confirmed run.

    Terminal runs are deliberately retained.  A later user command may need to
    distinguish "the previous goal has now stopped" from "there was never a
    goal" before it is allowed to start another one.
    """

    for leaf in leaves:
        contract = str(leaf.get("contract_id", ""))
        output = _mapping(leaf.get("output"))
        if contract == NAV_CONTRACT:
            run_id = _run_id_from_output(output)
            completion_state = _navigation_completion_state(output)
            if completion_state and run_id:
                active = ActiveRun(
                    run_id=run_id,
                    target=_target_from_output(output),
                    state=completion_state,
                )
            elif (
                leaf.get("success") is True
                and output.get("accepted") is True
                and run_id
            ):
                active = ActiveRun(
                    run_id=run_id,
                    target=_target_from_output(output),
                    state="RUNNING",
                )
        elif contract == STATUS_CONTRACT:
            if leaf.get("success") is not True:
                continue
            state = _state_from_output(output)
            run_id = _run_id_from_output(output)
            target = _target_from_output(output)
            if output.get("known") is not True or state not in _KNOWN_STATES:
                continue
            if active is None:
                if run_id:
                    active = ActiveRun(run_id, target, state)
                continue
            if run_id and run_id != active.run_id:
                continue
            active = ActiveRun(active.run_id, target or active.target, state)
    return active


def _latest_semantic_leaf(messages: Iterable[dict[str, Any]]) -> dict[str, Any] | None:
    leaves = _semantic_leaves(messages)
    return leaves[-1] if leaves else None


def _do(capability: str, args: dict[str, Any], description: str) -> dict[str, Any]:
    return {
        "op": "sequence",
        "op_id": 0,
        "description": description,
        "children": [
            {
                "op": "do",
                "op_id": 0,
                "description": description,
                "cap": capability,
                "args": args,
            }
        ],
    }


def _terminal(content: str, target: str, *, success: bool) -> Decision:
    state = "已到达目标并停止" if success else "任务已终止，未继续下发导航"
    return Decision(
        _envelope(
            content=f"{content}；{state}",
            description="semantic navigation terminal",
            task_update=_task(target, "done"),
        )
    )


def _terminal_state(content: str, run: ActiveRun) -> Decision:
    return _terminal(content, run.target or "当前目标", success=run.state == "SUCCEEDED")


def _hold(content: str, target: str) -> Decision:
    """Keep Pilot alive without claiming that a possibly moving goal stopped."""

    return Decision(
        _envelope(
            content=content,
            description="semantic navigation safety hold",
            task_update=_task(target or "当前目标", "in_progress"),
        )
    )


def _poll(run: ActiveRun, content: str | None = None) -> Decision:
    return Decision(
        _envelope(
            content=content
            or f"正在前往“{run.target or '已保存目标'}”，当前状态 {run.state}",
            description="poll semantic navigation status",
            tree=_do(
                STATUS_CAPABILITY,
                {"run_id": run.run_id},
                "read semantic navigation terminal state",
            ),
            task_update=_task(run.target or "当前目标", "in_progress"),
        )
    )


def _cancel(run: ActiveRun, content: str) -> Decision:
    return Decision(
        _envelope(
            content=content,
            description="cancel semantic navigation and await provider terminal state",
            tree=_do(
                CANCEL_CAPABILITY,
                {"run_id": run.run_id},
                "cancel semantic navigation",
            ),
            task_update={
                "goal": f"安全停止当前导航：{run.target or '当前目标'}",
                "success_criterion": "导航提供方确认 SUCCEEDED、CANCELED 或 FAILED，机器人已停止",
                "status": "in_progress",
            },
        )
    )


def _start_navigation(target: str) -> Decision:
    return Decision(
        _envelope(
            content=f"已匹配已验证地标“{target}”，交给 Robonix 导航服务",
            description=f"navigate to saved landmark {target}",
            tree=_do(
                NAV_CAPABILITY,
                {"name": target},
                f"navigate to verified landmark {target}",
            ),
            task_update=_task(target, "in_progress"),
        )
    )


def _execution_mode(value: Any) -> str:
    normalized = str(value or "preview").strip().lower()
    if normalized not in _EXECUTION_MODES:
        raise ValueError("semantic intent execution mode must be preview or live")
    return normalized


def _preview(messages: list[dict[str, Any]], store: LandmarkStore) -> Decision:
    """Resolve one intent for display while emitting no capability leaf.

    Liaison always submits an ASR final to Pilot.  In a motion-disabled boot,
    this model-side policy is therefore the boundary that lets the real
    Speech/Liaison/Pilot path demonstrate intent parsing without allowing
    Executor to receive a semantic-navigation, Nav2, ROS, or Unitree call.
    The saved name may be recognised even when its approach pose is still an
    unverified template; that condition is reported as a blocker, never
    relaxed into an executable target.
    """

    turn = current_turn(messages)
    normalized = normalize_text(turn.command)
    if not normalized:
        return Decision(
            _envelope(
                content="未识别到可预览的语义导航指令；未调用任何能力",
                description="motion-disabled semantic preview: empty command",
            )
        )
    if normalized in _STOP_UTTERANCES:
        return Decision(
            _envelope(
                content="当前是运动禁用预览模式，没有启动导航任务",
                description="motion-disabled semantic preview: no active run",
                task_update={
                    "goal": f"{_PREVIEW_GOAL_PREFIX}停止导航",
                    "success_criterion": (
                        "阻塞原因：GO2_ALLOW_MOTION=false；预览模式未调用 "
                        "Robonix navigation、Nav2 或 Unitree API"
                    ),
                    "status": "done",
                },
            )
        )

    try:
        landmark = store.resolve(
            turn.command,
            expected_map_id=store.map_id,
            expected_generation=store.map_generation,
            require_verified=False,
        )
    except LandmarkError as error:
        return Decision(
            _envelope(
                content=f"未找到唯一语义目标：{error}；未调用任何能力",
                description="motion-disabled semantic preview: target rejected",
            )
        )

    blockers = [
        "GO2_ALLOW_MOTION=false",
        "预览模式禁止执行",
        "map/localization/Nav2 就绪状态未通过",
    ]
    if not landmark.verified:
        blockers.insert(2, f"地标“{landmark.name}”尚无物理验证 approach Pose")
    blocked_reason = "；".join(blockers)
    return Decision(
        _envelope(
            content=(
                f"已识别导航意图：{landmark.name}；{blocked_reason}；"
                "未调用 Robonix navigation、Nav2 或 Unitree API"
            ),
            description="motion-disabled semantic preview: no capability calls",
            task_update={
                "goal": f"{_PREVIEW_GOAL_PREFIX}{landmark.name}",
                "success_criterion": (
                    f"阻塞原因：{blocked_reason}；未调用 Robonix navigation、"
                    "Nav2 或 Unitree API"
                ),
                "status": "done",
            },
        )
    )


def decide(
    messages: list[dict[str, Any]],
    store: LandmarkStore,
    *,
    execution_mode: str = "live",
) -> Decision:
    """Build one fail-closed RTDL response from Pilot history."""

    if _execution_mode(execution_mode) == "preview":
        return _preview(messages, store)

    caps = advertised_capabilities(messages)
    turn = current_turn(messages)
    prior_run = _advance_active(None, _semantic_leaves(turn.prior_messages))
    prior_active = prior_run if prior_run is not None and not prior_run.terminal else None
    current_leaves = _semantic_leaves(turn.feedback_messages)
    run = _advance_active(prior_active, current_leaves)
    active = run if run is not None and not run.terminal else None
    terminal = run if run is not None and run.terminal else None
    latest = current_leaves[-1] if current_leaves else None

    normalized_command = normalize_text(turn.command)
    stop_requested = normalized_command in _STOP_UTTERANCES
    landmark = None
    landmark_error: LandmarkError | None = None
    if not stop_requested:
        try:
            landmark = store.resolve(
                turn.command,
                expected_map_id=store.map_id,
                expected_generation=store.map_generation,
                require_verified=True,
            )
        except LandmarkError as exc:
            landmark_error = exc

    # A Pilot feedback message that cannot be decoded might represent an
    # accepted motion call. Never issue a second navigate call or claim done.
    malformed_feedback = any(
        not leaf_results([message]) for message in turn.feedback_messages
    )
    if malformed_feedback:
        candidate = active or prior_active
        if candidate is not None and CANCEL_CAPABILITY in caps:
            return _cancel(
                candidate,
                "执行反馈无法可靠解析；正在取消已知活动导航并等待终态",
            )
        return _hold(
            "执行反馈无法可靠解析；无法证明机器人已经停止，请人工接管并检查导航状态",
            landmark.name if landmark is not None else (candidate.target if candidate else "当前目标"),
        )

    if latest is not None:
        contract = str(latest.get("contract_id", ""))
        output = _mapping(latest.get("output"))
        navigation_completion = (
            _navigation_completion_state(output) if contract == NAV_CONTRACT else ""
        )
        provider_terminal = (
            navigation_completion in _TERMINAL_STATES
            and terminal is not None
        )

        # Executor/RPC failures are not evidence that an already submitted goal
        # stopped. A navigate leaf carrying an explicit provider-terminal
        # snapshot is the exception; it is already measured terminal evidence.
        # Otherwise, if its run id is known, immediately enter the cancel loop.
        if latest.get("success") is not True and not provider_terminal:
            candidate = active or prior_active
            if candidate is not None and CANCEL_CAPABILITY in caps:
                return _cancel(
                    candidate,
                    "Robonix 能力调用失败；正在重试取消活动导航并等待终态",
                )
            return _hold(
                "Robonix 能力调用失败且无法证明没有活动目标；未宣告任务完成",
                landmark.name if landmark is not None else "当前目标",
            )

        if contract == NAV_CONTRACT:
            if navigation_completion:
                pass
            elif output.get("accepted") is True and not _run_id_from_output(output):
                return _hold(
                    "导航可能已接受但未返回可取消的 run_id；请人工接管，系统不会重复下发",
                    landmark.name if landmark is not None else "当前目标",
                )
            elif output.get("accepted") is False:
                if prior_active is not None and CANCEL_CAPABILITY in caps:
                    return _cancel(prior_active, "新导航未被接受；先安全取消原活动导航")
                return _terminal(
                    "语义导航明确拒绝了该请求",
                    landmark.name if landmark is not None else "当前目标",
                    success=False,
                )
            elif output.get("accepted") is not True:
                return _hold(
                    "导航反馈缺少可验证的接受状态或提供方终态；未宣告任务完成",
                    landmark.name if landmark is not None else "当前目标",
                )

        if contract == STATUS_CONTRACT:
            state = str(output.get("state", "")).strip().upper()
            if output.get("known") is not True or state not in _KNOWN_STATES:
                candidate = active or prior_active
                if candidate is not None and CANCEL_CAPABILITY in caps:
                    return _cancel(
                        candidate,
                        "导航状态未知或格式无效；正在取消活动导航并等待提供方终态",
                    )
                return _hold(
                    "导航状态未知且没有可验证的停止证据；未宣告任务完成",
                    landmark.name if landmark is not None else "当前目标",
                )

        if contract == CANCEL_CONTRACT:
            if output.get("accepted") is not True:
                candidate = active or prior_active
                if candidate is not None and CANCEL_CAPABILITY in caps:
                    return _cancel(candidate, "取消请求未被接受；正在重试取消")
                return _hold(
                    "取消请求未被接受且无法确认终态；请保持人工接管",
                    landmark.name if landmark is not None else "当前目标",
                )

    required_live_caps = {NAV_CAPABILITY, STATUS_CAPABILITY, CANCEL_CAPABILITY}
    monitoring_caps = {STATUS_CAPABILITY, CANCEL_CAPABILITY}

    if stop_requested:
        if terminal is not None:
            return _terminal_state(
                f"导航提供方已确认 {terminal.state}",
                terminal,
            )
        if active is None:
            return Decision(
                _envelope(
                    content="当前没有可识别的活动语义导航任务",
                    description="no active semantic navigation to cancel",
                    task_update={
                        "goal": "停止当前导航",
                        "success_criterion": "不存在活动语义导航任务",
                        "status": "done",
                    },
                )
            )
        if not monitoring_caps.issubset(caps):
            return _hold(
                "Pilot 缺少状态或取消能力，无法证明机器人停止；请立即人工接管",
                active.target,
            )
        if latest is not None and latest.get("contract_id") == CANCEL_CONTRACT:
            return _poll(active, "取消已提交，正在等待导航提供方确认终态")
        return _cancel(active, "收到停止指令，正在取消活动导航并等待终态")

    if landmark is None:
        if active is not None:
            if not monitoring_caps.issubset(caps):
                return _hold(
                    "新指令未匹配已验证地标，且 Pilot 缺少活动导航的安全跟踪能力",
                    active.target,
                )
            return _poll(
                active,
                f"新指令未匹配唯一已验证地标（{landmark_error}）；继续跟踪原导航，不下发新目标",
            )
        if terminal is not None:
            return _terminal_state(
                f"原导航已确认 {terminal.state}；新指令未匹配已验证地标",
                terminal,
            )
        return Decision(
            _envelope(
                content=f"未找到唯一且已验证的语义地标：{landmark_error}",
                description="semantic target rejected",
            )
        )

    target = landmark.name

    if terminal is not None:
        # A target switch is serialized: only after the old provider goal is
        # terminal may the new navigation be issued.
        if (
            prior_active is not None
            and terminal.run_id == prior_active.run_id
            and target != prior_active.target
        ):
            if not required_live_caps.issubset(caps):
                return _hold(
                    "原导航已停止，但 Pilot 未公布完整导航/状态/取消能力，未启动新目标",
                    target,
                )
            return _start_navigation(target)
        return _terminal_state(
            f"Robonix navigation 已确认 {terminal.state}",
            terminal,
        )

    if active is not None:
        if not monitoring_caps.issubset(caps):
            return _hold(
                "存在活动导航，但 Pilot 未公布完整状态/取消能力；请人工接管",
                active.target or target,
            )
        if prior_active is not None and active.run_id == prior_active.run_id:
            if target != prior_active.target:
                if latest is not None and latest.get("contract_id") == CANCEL_CONTRACT:
                    return _poll(active, "目标切换的取消已提交，等待原导航确认终态")
                return _cancel(
                    active,
                    f"切换到“{target}”前，先取消原目标“{active.target}”并等待终态",
                )
        return _poll(
            active,
            f"已启动前往“{active.target or target}”的真实导航，正在等待到达",
        )

    # No accepted run exists and there is no ambiguous executor feedback. A
    # first navigate call is allowed only when its status and cancel controls
    # are already discoverable in the same Pilot round.
    if not required_live_caps.issubset(caps):
        missing = sorted(required_live_caps - caps)
        return Decision(
            _envelope(
                content=f"Pilot 未公布完整语义导航能力，未发起运动；缺少：{missing}",
                description="semantic capability unavailable",
            )
        )
    return _start_navigation(target)


class Handler(BaseHTTPRequestHandler):
    store: LandmarkStore
    poll_delay_s: float = 2.0
    execution_mode: str = "preview"

    def log_message(self, _format: str, *_args: Any) -> None:
        """Avoid logging utterances, headers, or query payloads."""

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.rstrip("/").endswith("/models"):
            self._json(
                200,
                {"object": "list", "data": [{"id": "go2-semantic-router", "object": "model"}]},
            )
            return
        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400, "invalid content length")
            return
        if length <= 0 or length > _MAX_REQUEST_BYTES:
            self.send_error(413, "request body outside allowed size")
            return
        try:
            request = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.send_error(400, "invalid JSON")
            return
        messages = request.get("messages")
        if not isinstance(messages, list) or not all(isinstance(item, dict) for item in messages):
            self.send_error(400, "messages must be a list")
            return
        decision = decide(
            messages,
            self.store,
            execution_mode=self.execution_mode,
        )
        envelope = decision.envelope
        model = str(request.get("model") or "go2-semantic-router")
        if not request.get("stream", False):
            self._json(
                decision.status,
                {
                    "id": "go2-semantic-router",
                    "object": "chat.completion",
                    "created": int(time.time()),
                    "model": model,
                    "choices": [
                        {
                            "index": 0,
                            "message": {
                                "role": "assistant",
                                "content": json.dumps(envelope, ensure_ascii=False),
                            },
                            "finish_reason": "stop",
                        }
                    ],
                },
            )
            return
        children = envelope["rtdl"].get("children") or []
        capability = children[0].get("cap") if children else ""
        if capability in {STATUS_CAPABILITY, CANCEL_CAPABILITY}:
            time.sleep(max(0.0, min(self.poll_delay_s, 2.0)))
        self._stream(model, envelope)

    def _stream(self, model: str, envelope: dict[str, Any]) -> None:
        payload = json.dumps(envelope, ensure_ascii=False)
        base = {
            "id": "go2-semantic-router",
            "object": "chat.completion.chunk",
            "created": int(time.time()),
            "model": model,
        }
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        for delta, finish in (({"role": "assistant", "content": payload}, None), ({}, "stop")):
            chunk = dict(base, choices=[{"index": 0, "delta": delta, "finish_reason": finish}])
            self.wfile.write(f"data: {json.dumps(chunk, ensure_ascii=False)}\n\n".encode("utf-8"))
            self.wfile.flush()
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def build_server(
    host: str,
    port: int,
    landmarks: str | Path,
    *,
    poll_delay_s: float = 2.0,
    execution_mode: str = "preview",
) -> ThreadingHTTPServer:
    address = ipaddress.ip_address(host)
    if not address.is_loopback:
        raise ValueError("semantic intent endpoint must bind to a literal loopback address")
    store = LandmarkStore.from_path(landmarks)
    selected_mode = _execution_mode(execution_mode)
    handler = type(
        "ConfiguredSemanticIntentHandler",
        (Handler,),
        {
            "store": store,
            "poll_delay_s": poll_delay_s,
            "execution_mode": selected_mode,
        },
    )
    return ThreadingHTTPServer((host, port), handler)


def main() -> None:
    parser = argparse.ArgumentParser(description="Bounded local semantic intent endpoint")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--landmarks", type=Path, required=True)
    parser.add_argument("--poll-delay-s", type=float, default=2.0)
    parser.add_argument(
        "--execution-mode",
        choices=sorted(_EXECUTION_MODES),
        default="preview",
        help="preview emits no capability leaves; live may emit semantic navigation",
    )
    args = parser.parse_args()
    server = build_server(
        args.host,
        args.port,
        args.landmarks,
        poll_delay_s=args.poll_delay_s,
        execution_mode=args.execution_mode,
    )
    print(
        f"[semantic-intent] listening on http://{args.host}:{args.port}/v1 "
        f"mode={args.execution_mode}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
