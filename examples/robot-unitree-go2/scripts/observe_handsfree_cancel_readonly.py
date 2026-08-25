#!/usr/bin/env python3
"""Bounded, read-only evidence capture for Hands-free cancellation.

The observer attaches to an already-running loopback Robonix Client.  It does
not start the Client, enable or disable Hands-free mode, submit work, or touch
the robot.  A live observation requires the explicit ``--observe-live`` gate.
"""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import stat
import sys
import time
from typing import Any, Mapping, TextIO


ROOT = Path(__file__).resolve().parents[1]
LOOPBACK_HOST = "127.0.0.1"
SETTINGS_PATH = "/api/settings"
STATUS_PATH = "/api/handsfree/status"
PLANS_PATH = "/api/executor/active-plans"
EVENTS_PATH = "/ws/handsfree-events"

ALLOWED_HTTP_OPERATIONS = frozenset(
    {
        ("GET", SETTINGS_PATH),
        ("POST", STATUS_PATH),
        ("POST", PLANS_PATH),
    }
)
ALLOWED_WEBSOCKET_PATHS = frozenset({EVENTS_PATH})
ALLOWED_OUTPUT_ROOTS = (ROOT / "logs", ROOT / "rbnx-build")

MIN_DURATION_SECONDS = 30.0
MAX_DURATION_SECONDS = 300.0
MIN_POST_DISABLE_OBSERVATION_SECONDS = 25.0
MIN_POLL_INTERVAL_SECONDS = 0.5
MAX_POLL_INTERVAL_SECONDS = 0.5
MAX_ENDPOINT_SAMPLE_GAP_SECONDS = 1.0
HTTP_TIMEOUT_SECONDS = 2.0
MAX_RESPONSE_BYTES = 2 * 1024 * 1024
MAX_HTTP_HEADER_BYTES = 64 * 1024
LATE_EVENT_KINDS = (
    "asr_final",
    "pilot",
    "tts_started",
    "tts_done",
    "session_done",
)
KNOWN_VOICE_EVENT_KINDS = frozenset(
    {
        "session_started",
        "recording_started",
        "recording_done",
        "asr_partial",
        "asr_final",
        "user_identified",
        "pilot",
        "tts_started",
        "tts_done",
        "session_done",
        "error",
    }
)
AUDIO_ROUTE_SETTING_KEYS = (
    "micNodeId",
    "micDeviceId",
    "speakerNodeId",
    "speakerDeviceId",
)


def _utc_now() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _bounded_float(
    value: str,
    *,
    label: str,
    minimum: float,
    maximum: float,
) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a number") from exc
    if not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{label} must be between {minimum:g} and {maximum:g} seconds"
        )
    return parsed


def _duration(value: str) -> float:
    return _bounded_float(
        value,
        label="duration",
        minimum=MIN_DURATION_SECONDS,
        maximum=MAX_DURATION_SECONDS,
    )


def _poll_interval(value: str) -> float:
    return _bounded_float(
        value,
        label="poll interval",
        minimum=MIN_POLL_INTERVAL_SECONDS,
        maximum=MAX_POLL_INTERVAL_SECONDS,
    )


def _port(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("client port must be an integer") from exc
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("client port must be between 1 and 65535")
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Observe an already-running loopback Robonix Client for a bounded "
            "Hands-free cancellation acceptance window."
        )
    )
    parser.add_argument(
        "--observe-live",
        action="store_true",
        help="explicitly permit loopback observation; never starts the Client",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="new absolute directory below package logs/ or rbnx-build/",
    )
    parser.add_argument(
        "--duration-seconds",
        required=True,
        type=_duration,
        help=f"bounded window ({MIN_DURATION_SECONDS:g}-{MAX_DURATION_SECONDS:g}s)",
    )
    parser.add_argument(
        "--poll-interval-seconds",
        default=0.5,
        type=_poll_interval,
        help="fixed status/plan poll interval (0.5s)",
    )
    parser.add_argument(
        "--client-port",
        default=7860,
        type=_port,
        help="loopback Client port (default: 7860)",
    )
    return parser


def _root_for_target(target: Path) -> tuple[Path, Path]:
    for candidate in ALLOWED_OUTPUT_ROOTS:
        root = Path(os.path.abspath(os.fspath(candidate)))
        try:
            root_stat = os.lstat(root)
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
            raise RuntimeError(f"allowed output root is not a real directory: {root}")
        try:
            relative = target.relative_to(root)
        except ValueError:
            continue
        if relative == Path("."):
            raise RuntimeError("output directory must be a new child directory")
        return root, relative
    allowed = " or ".join(str(path) for path in ALLOWED_OUTPUT_ROOTS)
    raise RuntimeError(f"output directory must be below {allowed}")


def _prepare_output(requested: Path) -> Path:
    """Create one new private directory without following symlink components."""

    expanded = requested.expanduser()
    if not expanded.is_absolute():
        raise RuntimeError("output directory must be an absolute path")
    target = Path(os.path.abspath(os.fspath(expanded)))
    root, relative = _root_for_target(target)

    directory_flags = os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0)
    current_fd = os.open(root, directory_flags)
    current = root
    parts = relative.parts
    try:
        for index, part in enumerate(parts):
            current = current / part
            is_final = index == len(parts) - 1
            try:
                current_stat = os.stat(
                    part,
                    dir_fd=current_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                try:
                    os.mkdir(part, mode=0o700, dir_fd=current_fd)
                except FileExistsError as exc:
                    raise RuntimeError(
                        f"output path changed while it was being created: {current}"
                    ) from exc
            else:
                if stat.S_ISLNK(current_stat.st_mode):
                    raise RuntimeError(f"output path contains a symlink: {current}")
                if not stat.S_ISDIR(current_stat.st_mode):
                    raise RuntimeError(
                        f"output path component is not a directory: {current}"
                    )
                if is_final:
                    raise RuntimeError(
                        f"refusing to overwrite existing output: {current}"
                    )

            next_fd = os.open(part, directory_flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd

        opened_path = os.readlink(f"/proc/self/fd/{current_fd}")
        if opened_path != os.fspath(target):
            raise RuntimeError("output directory changed while it was being created")
        os.fchmod(current_fd, 0o700)
    finally:
        os.close(current_fd)
    return target


def _open_prepared_output(output_dir: Path) -> int:
    """Open the prepared directory by dirfd traversal and retain its identity."""

    target = Path(os.path.abspath(os.fspath(output_dir)))
    root, relative = _root_for_target(target)
    flags = os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0)
    current_fd = os.open(root, flags)
    current = root
    try:
        for part in relative.parts:
            current = current / part
            try:
                component_stat = os.stat(
                    part,
                    dir_fd=current_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError as exc:
                raise RuntimeError(f"evidence output disappeared: {current}") from exc
            if stat.S_ISLNK(component_stat.st_mode):
                raise RuntimeError(f"evidence output contains a symlink: {current}")
            if not stat.S_ISDIR(component_stat.st_mode):
                raise RuntimeError(f"evidence output is not a directory: {current}")
            next_fd = os.open(part, flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd

        opened_path = os.readlink(f"/proc/self/fd/{current_fd}")
        if opened_path != os.fspath(target):
            raise RuntimeError("evidence output identity changed before opening")
        opened_stat = os.fstat(current_fd)
        if opened_stat.st_uid != os.geteuid():
            raise RuntimeError("evidence output is not owned by the current user")
        if stat.S_IMODE(opened_stat.st_mode) != 0o700:
            raise RuntimeError("evidence output permissions are not exactly 0700")
        result_fd = current_fd
        current_fd = -1
        return result_fd
    finally:
        if current_fd >= 0:
            os.close(current_fd)


class EvidenceWriter:
    """Stage both evidence files privately and publish each with an atomic rename."""

    LOG_NAME = "observations.jsonl"
    SUMMARY_NAME = "summary.json"
    LOG_TEMP_NAME = ".observations.jsonl.tmp"
    SUMMARY_TEMP_NAME = ".summary.json.tmp"

    def __init__(self, output_dir: Path) -> None:
        self._directory_fd = _open_prepared_output(output_dir)
        directory_stat = os.fstat(self._directory_fd)
        if not stat.S_ISDIR(directory_stat.st_mode):
            os.close(self._directory_fd)
            raise RuntimeError("evidence output is not a directory")
        self.output_dir = output_dir
        self._closed = False
        self._log_stream = self._open_private_text(self.LOG_TEMP_NAME)

    def _open_private_text(self, name: str) -> TextIO:
        flags = (
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_NOFOLLOW", 0)
        )
        descriptor = os.open(name, flags, 0o600, dir_fd=self._directory_fd)
        os.fchmod(descriptor, 0o600)
        return os.fdopen(descriptor, "w", encoding="utf-8")

    def write(self, record: Mapping[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("evidence writer is already closed")
        self._log_stream.write(
            json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
            + "\n"
        )

    def finalize(self, summary: Mapping[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("evidence writer is already closed")
        summary_stream: TextIO | None = None
        try:
            self._log_stream.flush()
            os.fsync(self._log_stream.fileno())
            self._log_stream.close()

            summary_stream = self._open_private_text(self.SUMMARY_TEMP_NAME)
            json.dump(summary, summary_stream, ensure_ascii=False, indent=2, sort_keys=True)
            summary_stream.write("\n")
            summary_stream.flush()
            os.fsync(summary_stream.fileno())
            summary_stream.close()
            summary_stream = None

            for temporary_name, final_name in (
                (self.LOG_TEMP_NAME, self.LOG_NAME),
                (self.SUMMARY_TEMP_NAME, self.SUMMARY_NAME),
            ):
                try:
                    os.link(
                        temporary_name,
                        final_name,
                        src_dir_fd=self._directory_fd,
                        dst_dir_fd=self._directory_fd,
                        follow_symlinks=False,
                    )
                except FileExistsError as exc:
                    raise RuntimeError(
                        f"refusing to overwrite evidence file: {final_name}"
                    ) from exc
                os.unlink(temporary_name, dir_fd=self._directory_fd)
            os.fsync(self._directory_fd)
            self._closed = True
            os.close(self._directory_fd)
        except Exception:
            if summary_stream is not None and not summary_stream.closed:
                summary_stream.close()
            self.abort()
            raise

    def abort(self) -> None:
        if self._closed:
            return
        if not self._log_stream.closed:
            self._log_stream.close()
        for name in (self.LOG_TEMP_NAME, self.SUMMARY_TEMP_NAME):
            try:
                os.unlink(name, dir_fd=self._directory_fd)
            except FileNotFoundError:
                pass
        self._closed = True
        os.close(self._directory_fd)


def classify_voice_event(payload: Mapping[str, Any]) -> str | None:
    if payload.get("type") != "voice_event":
        return None
    event = payload.get("event")
    if not isinstance(event, Mapping):
        return None
    kind = event.get("kind")
    if not isinstance(kind, str):
        return None
    normalized = kind.strip().lower()
    return normalized or None


def _status_object(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    nested = payload.get("status")
    return nested if isinstance(nested, Mapping) else payload


def _plan_object(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    nested = payload.get("activePlans")
    return nested if isinstance(nested, Mapping) else payload


class AcceptanceTracker:
    """Classify observations around the first valid enabled-to-disabled edge."""

    def __init__(self, started_monotonic_ns: int | None = None) -> None:
        self.started_monotonic_ns = started_monotonic_ns or time.monotonic_ns()
        self.settings_loaded = False
        self.websocket_connected = False
        self.websocket_accepted = False
        self.websocket_covered_to_deadline = False
        self.websocket_messages = 0
        self.websocket_error_messages = 0
        self.websocket_malformed_messages = 0
        self.websocket_unknown_event_kinds = 0
        self.operator_ready_announced = False
        self.operator_ready_at_utc: str | None = None
        self.operator_ready_at_monotonic_ns: int | None = None
        self.status_samples = 0
        self.status_unavailable_samples = 0
        self.status_malformed_samples = 0
        self.last_status_sample_monotonic_ns: int | None = None
        self.max_status_sample_gap_seconds = 0.0
        self.plan_samples = 0
        self.last_plan_sample_monotonic_ns: int | None = None
        self.max_plan_sample_gap_seconds = 0.0
        self.pre_disable_plan_samples = 0
        self.saw_enabled = False
        self.previous_enabled: bool | None = None
        self.last_enabled_transcript: str | None = None
        self.disable_observed_at_utc: str | None = None
        self.disable_observed_at_monotonic_ns: int | None = None
        self.disable_elapsed_seconds: float | None = None
        self.transcript_at_disable: str | None = None
        self.transition_window_transcript_changed = False
        self.post_disable_last_transcript: str | None = None
        self.post_disable_transcript_change_count = 0
        self.post_disable_status_samples = 0
        self.post_disable_status_unavailable_samples = 0
        self.reenabled_after_disable_samples = 0
        self.pre_disable_plan_issue_samples = 0
        self.post_disable_plan_samples = 0
        self.post_disable_plan_unavailable_samples = 0
        self.post_disable_plan_malformed_samples = 0
        self.post_disable_plan_nonzero_samples = 0
        self.post_disable_max_plan_count = 0
        self.late_event_counts = {kind: 0 for kind in LATE_EVENT_KINDS}
        self.pending_transition_event_counts = {
            kind: 0 for kind in LATE_EVENT_KINDS
        }
        self.transition_window_event_counts = {
            kind: 0 for kind in LATE_EVENT_KINDS
        }
        self.observer_error_count = 0
        self.observer_errors: list[dict[str, str]] = []

    def elapsed_seconds(self, observed_monotonic_ns: int) -> float:
        return round(
            (observed_monotonic_ns - self.started_monotonic_ns) / 1_000_000_000,
            6,
        )

    def is_post_disable(self, observed_monotonic_ns: int) -> bool:
        boundary = self.disable_observed_at_monotonic_ns
        return boundary is not None and observed_monotonic_ns >= boundary

    def _observe_endpoint_sample(
        self, endpoint: str, observed_monotonic_ns: int
    ) -> None:
        if endpoint == "status":
            previous = self.last_status_sample_monotonic_ns
            if previous is not None:
                self.max_status_sample_gap_seconds = max(
                    self.max_status_sample_gap_seconds,
                    (observed_monotonic_ns - previous) / 1_000_000_000,
                )
            self.last_status_sample_monotonic_ns = observed_monotonic_ns
            return
        if endpoint == "plans":
            previous = self.last_plan_sample_monotonic_ns
            if previous is not None:
                self.max_plan_sample_gap_seconds = max(
                    self.max_plan_sample_gap_seconds,
                    (observed_monotonic_ns - previous) / 1_000_000_000,
                )
            self.last_plan_sample_monotonic_ns = observed_monotonic_ns
            return
        raise ValueError(f"unknown observation endpoint: {endpoint}")

    def observe_status(
        self,
        payload: Mapping[str, Any],
        *,
        observed_monotonic_ns: int,
        observed_at_utc: str,
    ) -> dict[str, Any]:
        self.status_samples += 1
        self._observe_endpoint_sample("status", observed_monotonic_ns)
        nested_status = payload.get("status")
        nested_malformed = "status" in payload and not isinstance(
            nested_status, Mapping
        )
        status_payload = _status_object(payload)
        available_value = status_payload.get("available")
        available = available_value is True
        enabled_value = status_payload.get("enabled")
        enabled = enabled_value if isinstance(enabled_value, bool) else None
        transcript_value = status_payload.get("lastTranscript")
        transcript = transcript_value if isinstance(transcript_value, str) else None
        state_value = status_payload.get("state")
        malformed = nested_malformed or not isinstance(available_value, bool)
        if available:
            malformed = (
                malformed
                or enabled is None
                or transcript is None
                or not isinstance(state_value, str)
            )

        if malformed:
            self.status_malformed_samples += 1
            if self.is_post_disable(observed_monotonic_ns):
                self.post_disable_status_samples += 1
                self.post_disable_status_unavailable_samples += 1
            return {
                "available": False,
                "malformed": True,
                "enabled": enabled,
                "post_disable": self.is_post_disable(observed_monotonic_ns),
                "disable_transition": False,
            }

        if not available:
            self.status_unavailable_samples += 1
            if self.is_post_disable(observed_monotonic_ns):
                self.post_disable_status_samples += 1
                self.post_disable_status_unavailable_samples += 1
            return {
                "available": False,
                "malformed": False,
                "enabled": enabled,
                "post_disable": self.is_post_disable(observed_monotonic_ns),
                "disable_transition": False,
            }

        transition = self.previous_enabled is True and enabled is False
        if transition and self.disable_observed_at_monotonic_ns is None:
            self.disable_observed_at_monotonic_ns = observed_monotonic_ns
            self.disable_observed_at_utc = observed_at_utc
            self.disable_elapsed_seconds = self.elapsed_seconds(observed_monotonic_ns)
            self.transcript_at_disable = transcript
            self.post_disable_last_transcript = transcript
            if (
                self.last_enabled_transcript is not None
                and transcript != self.last_enabled_transcript
            ):
                self.transition_window_transcript_changed = True
            self.transition_window_event_counts = dict(
                self.pending_transition_event_counts
            )

        post_disable = self.is_post_disable(observed_monotonic_ns)
        if post_disable:
            self.post_disable_status_samples += 1
            if enabled:
                self.reenabled_after_disable_samples += 1
            if self.post_disable_last_transcript is None:
                self.post_disable_last_transcript = transcript
            elif transcript != self.post_disable_last_transcript:
                self.post_disable_transcript_change_count += 1
                self.post_disable_last_transcript = transcript

        if enabled:
            self.saw_enabled = True
            if not post_disable:
                if not self.operator_ready_announced:
                    self.last_enabled_transcript = transcript
                    # Before READY, a confirmed enabled sample establishes a
                    # fresh transcript/event baseline.  After READY, freeze
                    # both: an HTTP request started before the operator's click
                    # can return a stale enabled=true response after a
                    # transcript change or WebSocket event and must not erase
                    # either from the fail-closed disable-boundary evidence.
                    self.pending_transition_event_counts = {
                        kind: 0 for kind in LATE_EVENT_KINDS
                    }
        self.previous_enabled = enabled
        return {
            "available": True,
            "malformed": False,
            "enabled": enabled,
            "post_disable": post_disable,
            "disable_transition": transition,
            "transcript_changed_after_disable": (
                post_disable and self.post_disable_transcript_change_count > 0
            ),
        }

    def observe_plans(
        self,
        payload: Mapping[str, Any],
        *,
        observed_monotonic_ns: int,
    ) -> dict[str, Any]:
        self.plan_samples += 1
        self._observe_endpoint_sample("plans", observed_monotonic_ns)
        plan_payload = _plan_object(payload)
        available = plan_payload.get("available") is True
        plans = plan_payload.get("plans")
        plans_length = len(plans) if isinstance(plans, list) else None
        raw_count = plan_payload.get("count")
        count_is_integer = isinstance(raw_count, int) and not isinstance(
            raw_count, bool
        )
        count = (
            raw_count
            if count_is_integer
            else None
        )
        malformed = available and (
            not count_is_integer
            or plans_length is None
            or count != plans_length
        )
        nonzero = available and not malformed and count != 0
        post_disable = self.is_post_disable(observed_monotonic_ns)
        issue = not available or malformed or nonzero

        if not post_disable:
            self.pre_disable_plan_samples += 1
            if issue:
                self.pre_disable_plan_issue_samples += 1
        if post_disable:
            self.post_disable_plan_samples += 1
            if not available:
                self.post_disable_plan_unavailable_samples += 1
            if malformed:
                self.post_disable_plan_malformed_samples += 1
            if nonzero:
                self.post_disable_plan_nonzero_samples += 1
            if count is not None and not malformed:
                self.post_disable_max_plan_count = max(
                    self.post_disable_max_plan_count, count
                )

        return {
            "available": available,
            "count": count,
            "malformed": malformed,
            "nonzero": nonzero,
            "post_disable": post_disable,
        }

    def observe_websocket(
        self,
        payload: Mapping[str, Any],
        *,
        observed_monotonic_ns: int,
    ) -> dict[str, Any]:
        self.websocket_messages += 1
        message_type = payload.get("type")
        malformed = False
        unknown_event_kind = False
        kind: str | None = None
        if message_type == "accepted":
            self.websocket_accepted = True
        elif message_type == "error":
            self.websocket_error_messages += 1
            if not isinstance(payload.get("error"), str):
                malformed = True
        elif message_type == "voice_event":
            kind = classify_voice_event(payload)
            if kind is None:
                malformed = True
            elif kind not in KNOWN_VOICE_EVENT_KINDS:
                unknown_event_kind = True
            elif kind == "error":
                self.websocket_error_messages += 1
        else:
            malformed = True
        if malformed:
            self.websocket_malformed_messages += 1
        if unknown_event_kind:
            self.websocket_unknown_event_kinds += 1
        post_disable = self.is_post_disable(observed_monotonic_ns)
        if (
            not malformed
            and not unknown_event_kind
            and post_disable
            and kind in self.late_event_counts
        ):
            self.late_event_counts[kind] += 1
        elif (
            not malformed
            and not unknown_event_kind
            and kind in self.pending_transition_event_counts
            and self.saw_enabled
            and self.disable_observed_at_monotonic_ns is None
        ):
            # A UI disable click isn't timestamped by the observation API.  Any
            # target event after the last confirmed enabled sample and before
            # the first confirmed disabled sample is therefore ambiguous and
            # must fail closed if that next sample is disabled.
            self.pending_transition_event_counts[kind] += 1
        return {
            "message_type": message_type,
            "event_kind": kind,
            "malformed": malformed,
            "unknown_event_kind": unknown_event_kind,
            "post_disable": post_disable,
            "counted_as_late": post_disable and kind in self.late_event_counts,
            "pending_disable_transition": (
                not post_disable
                and kind in self.pending_transition_event_counts
                and self.saw_enabled
                and self.disable_observed_at_monotonic_ns is None
            ),
        }

    def observe_error(self, source: str, message: str) -> None:
        self.observer_error_count += 1
        if len(self.observer_errors) < 20:
            self.observer_errors.append(
                {"source": source, "message": message.replace("\n", " ")[:500]}
            )

    def summary(
        self,
        *,
        requested_duration_seconds: float,
        poll_interval_seconds: float,
        finished_monotonic_ns: int,
        interrupted: bool,
    ) -> dict[str, Any]:
        late_tts = (
            self.late_event_counts["tts_started"]
            + self.late_event_counts["tts_done"]
        )
        post_disable_observation_seconds: float | None = None
        if self.disable_observed_at_monotonic_ns is not None:
            post_disable_observation_seconds = round(
                max(
                    0,
                    finished_monotonic_ns
                    - self.disable_observed_at_monotonic_ns,
                )
                / 1_000_000_000,
                6,
            )
        status_tail_gap_seconds = (
            max(
                0.0,
                (finished_monotonic_ns - self.last_status_sample_monotonic_ns)
                / 1_000_000_000,
            )
            if self.last_status_sample_monotonic_ns is not None
            else 0.0
        )
        plan_tail_gap_seconds = (
            max(
                0.0,
                (finished_monotonic_ns - self.last_plan_sample_monotonic_ns)
                / 1_000_000_000,
            )
            if self.last_plan_sample_monotonic_ns is not None
            else 0.0
        )
        effective_status_gap_seconds = max(
            self.max_status_sample_gap_seconds, status_tail_gap_seconds
        )
        effective_plan_gap_seconds = max(
            self.max_plan_sample_gap_seconds, plan_tail_gap_seconds
        )
        reasons: list[str] = []
        if interrupted:
            reasons.append("observer_interrupted")
        if not self.settings_loaded:
            reasons.append("settings_not_loaded")
        if not self.websocket_connected:
            reasons.append("websocket_not_connected")
        if not self.websocket_accepted:
            reasons.append("websocket_not_accepted")
        if not self.websocket_covered_to_deadline:
            reasons.append("websocket_did_not_cover_full_window")
        if not self.operator_ready_announced:
            reasons.append("operator_ready_boundary_not_announced")
        if not self.saw_enabled:
            reasons.append("enabled_state_not_observed")
        if self.disable_observed_at_monotonic_ns is None:
            reasons.append("enabled_to_disabled_transition_not_observed")
        elif (
            post_disable_observation_seconds
            < MIN_POST_DISABLE_OBSERVATION_SECONDS
        ):
            reasons.append("post_disable_observation_too_short")
        if self.post_disable_status_samples == 0:
            reasons.append("no_post_disable_status_sample")
        if self.post_disable_plan_samples == 0:
            reasons.append("no_post_disable_plan_sample")
        if self.pre_disable_plan_samples == 0:
            reasons.append("no_pre_disable_plan_sample")
        if self.transition_window_transcript_changed:
            reasons.append("transcript_changed_across_disable_boundary")
        if self.post_disable_transcript_change_count:
            reasons.append("transcript_changed_after_disable")
        if self.reenabled_after_disable_samples:
            reasons.append("handsfree_reenabled_after_disable")
        if self.post_disable_status_unavailable_samples:
            reasons.append("status_unavailable_after_disable")
        if self.status_malformed_samples:
            reasons.append("handsfree_status_schema_malformed")
        if self.pre_disable_plan_issue_samples:
            reasons.append("active_plan_baseline_not_zero_or_unavailable")
        if self.post_disable_plan_unavailable_samples:
            reasons.append("active_plans_unavailable_after_disable")
        if self.post_disable_plan_malformed_samples:
            reasons.append("active_plans_malformed_after_disable")
        if self.post_disable_plan_nonzero_samples:
            reasons.append("active_plans_nonzero_after_disable")
        if effective_status_gap_seconds > MAX_ENDPOINT_SAMPLE_GAP_SECONDS:
            reasons.append("handsfree_status_sample_gap_exceeded")
        if effective_plan_gap_seconds > MAX_ENDPOINT_SAMPLE_GAP_SECONDS:
            reasons.append("active_plans_sample_gap_exceeded")
        if any(self.late_event_counts.values()):
            reasons.append("voice_events_received_after_disable")
        if any(self.transition_window_event_counts.values()):
            reasons.append("voice_events_in_disable_transition_window")
        if self.websocket_error_messages:
            reasons.append("websocket_reported_error")
        if self.websocket_malformed_messages:
            reasons.append("websocket_message_schema_malformed")
        if self.websocket_unknown_event_kinds:
            reasons.append("websocket_unknown_voice_event_kind")
        if self.observer_error_count:
            reasons.append("observer_runtime_error")

        return {
            "schema_version": 1,
            "generated_at_utc": _utc_now(),
            "acceptance": {
                "passed": not reasons,
                "failure_reasons": reasons,
            },
            "read_only_contract": {
                "loopback_host": LOOPBACK_HOST,
                "http_operations": [
                    {"method": method, "path": path}
                    for method, path in sorted(ALLOWED_HTTP_OPERATIONS)
                ],
                "websocket_paths": sorted(ALLOWED_WEBSOCKET_PATHS),
                "starts_client_or_robot_process": False,
                "changes_handsfree_state": False,
                "submits_or_cancels_work": False,
                "clears_audio_route_selection_in_websocket_copy": True,
                "websocket_proxy_disabled": True,
                "websocket_redirect_disabled_by_preconnected_socket": True,
            },
            "window": {
                "requested_duration_seconds": requested_duration_seconds,
                "actual_elapsed_seconds": self.elapsed_seconds(finished_monotonic_ns),
                "poll_interval_seconds": poll_interval_seconds,
                "interrupted": interrupted,
            },
            "connection": {
                "settings_loaded": self.settings_loaded,
                "websocket_connected": self.websocket_connected,
                "websocket_accepted": self.websocket_accepted,
                "websocket_covered_to_deadline": (
                    self.websocket_covered_to_deadline
                ),
                "websocket_messages": self.websocket_messages,
                "websocket_error_messages": self.websocket_error_messages,
                "websocket_malformed_messages": (
                    self.websocket_malformed_messages
                ),
                "websocket_unknown_event_kinds": (
                    self.websocket_unknown_event_kinds
                ),
                "operator_ready_announced": self.operator_ready_announced,
                "operator_ready_at_utc": self.operator_ready_at_utc,
                "operator_ready_elapsed_seconds": (
                    self.elapsed_seconds(self.operator_ready_at_monotonic_ns)
                    if self.operator_ready_at_monotonic_ns is not None
                    else None
                ),
            },
            "disable_boundary": {
                "saw_enabled": self.saw_enabled,
                "observed_at_utc": self.disable_observed_at_utc,
                "elapsed_seconds": self.disable_elapsed_seconds,
                "minimum_post_disable_observation_seconds": (
                    MIN_POST_DISABLE_OBSERVATION_SECONDS
                ),
                "post_disable_observation_seconds": (
                    post_disable_observation_seconds
                ),
                "transcript_at_disable": self.transcript_at_disable,
                "transition_window_transcript_changed": (
                    self.transition_window_transcript_changed
                ),
            },
            "status": {
                "samples": self.status_samples,
                "unavailable_samples": self.status_unavailable_samples,
                "malformed_samples": self.status_malformed_samples,
                "maximum_sample_gap_seconds": round(
                    effective_status_gap_seconds, 6
                ),
                "sample_gap_limit_seconds": MAX_ENDPOINT_SAMPLE_GAP_SECONDS,
                "post_disable_samples": self.post_disable_status_samples,
                "post_disable_unavailable_samples": (
                    self.post_disable_status_unavailable_samples
                ),
                "post_disable_transcript_change_count": (
                    self.post_disable_transcript_change_count
                ),
                "reenabled_after_disable_samples": (
                    self.reenabled_after_disable_samples
                ),
            },
            "active_plans": {
                "samples": self.plan_samples,
                "maximum_sample_gap_seconds": round(
                    effective_plan_gap_seconds, 6
                ),
                "sample_gap_limit_seconds": MAX_ENDPOINT_SAMPLE_GAP_SECONDS,
                "pre_disable_samples": self.pre_disable_plan_samples,
                "pre_disable_issue_samples": self.pre_disable_plan_issue_samples,
                "post_disable_samples": self.post_disable_plan_samples,
                "post_disable_unavailable_samples": (
                    self.post_disable_plan_unavailable_samples
                ),
                "post_disable_malformed_samples": (
                    self.post_disable_plan_malformed_samples
                ),
                "post_disable_nonzero_samples": (
                    self.post_disable_plan_nonzero_samples
                ),
                "post_disable_max_count": self.post_disable_max_plan_count,
            },
            "post_disable_voice_events": {
                **self.late_event_counts,
                "tts": late_tts,
                "total": sum(self.late_event_counts.values()),
            },
            "disable_transition_voice_events": {
                **self.transition_window_event_counts,
                "tts": (
                    self.transition_window_event_counts["tts_started"]
                    + self.transition_window_event_counts["tts_done"]
                ),
                "total": sum(self.transition_window_event_counts.values()),
            },
            "observer_errors": {
                "count": self.observer_error_count,
                "retained": self.observer_errors,
            },
            "evidence_files": {
                "observations": EvidenceWriter.LOG_NAME,
                "summary": EvidenceWriter.SUMMARY_NAME,
                "file_mode": "0600",
            },
        }


async def _request_json(
    method: str,
    path: str,
    *,
    port: int,
    settings: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    operation = (method, path)
    if operation not in ALLOWED_HTTP_OPERATIONS:
        raise RuntimeError(f"HTTP operation is outside the read-only allowlist: {operation}")
    if method == "POST" and settings is None:
        raise RuntimeError("read-only POST observation requires Client settings")
    if method == "GET" and settings is not None:
        raise RuntimeError("GET settings observation does not accept a body")

    body = b""
    request_headers = [
        f"Host: {LOOPBACK_HOST}:{port}",
        "Accept: application/json",
        "Connection: close",
    ]
    if settings is not None:
        body = json.dumps({"settings": settings}, ensure_ascii=False).encode("utf-8")
        request_headers.extend(
            (
                "Content-Type: application/json; charset=utf-8",
                f"Content-Length: {len(body)}",
            )
        )

    writer: asyncio.StreamWriter | None = None
    try:
        reader, writer = await asyncio.open_connection(
            LOOPBACK_HOST,
            port,
            limit=MAX_HTTP_HEADER_BYTES,
        )
        request = (
            f"{method} {path} HTTP/1.1\r\n"
            + "\r\n".join(request_headers)
            + "\r\n\r\n"
        ).encode("ascii") + body
        writer.write(request)
        await writer.drain()

        try:
            header_bytes = await reader.readuntil(b"\r\n\r\n")
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError) as exc:
            raise RuntimeError("Client returned an incomplete or oversized HTTP header") from exc
        if len(header_bytes) > MAX_HTTP_HEADER_BYTES:
            raise RuntimeError("Client returned an oversized HTTP header")
        try:
            header_lines = header_bytes.decode("iso-8859-1").split("\r\n")
            protocol, status_text, reason = header_lines[0].split(" ", 2)
            status = int(status_text)
        except (UnicodeDecodeError, ValueError, IndexError) as exc:
            raise RuntimeError("Client returned an invalid HTTP status line") from exc
        if protocol not in ("HTTP/1.0", "HTTP/1.1"):
            raise RuntimeError("Client returned an unsupported HTTP protocol")

        response_headers: dict[str, str] = {}
        for line in header_lines[1:]:
            if not line:
                continue
            if ":" not in line:
                raise RuntimeError("Client returned a malformed HTTP header")
            name, value = line.split(":", 1)
            normalized_name = name.strip().lower()
            if normalized_name in response_headers:
                raise RuntimeError(
                    f"Client returned a duplicate HTTP header: {normalized_name}"
                )
            response_headers[normalized_name] = value.strip()

        transfer_encoding = response_headers.get("transfer-encoding", "").lower()
        if transfer_encoding and transfer_encoding != "identity":
            raise RuntimeError("Client returned unsupported transfer encoding")
        content_length_text = response_headers.get("content-length")
        if content_length_text is not None:
            try:
                content_length = int(content_length_text)
            except ValueError as exc:
                raise RuntimeError("Client returned invalid Content-Length") from exc
            if not 0 <= content_length <= MAX_RESPONSE_BYTES:
                raise RuntimeError(
                    f"response exceeded {MAX_RESPONSE_BYTES} bytes"
                )
            content = await reader.readexactly(content_length)
        else:
            content = await reader.read(MAX_RESPONSE_BYTES + 1)
        if len(content) > MAX_RESPONSE_BYTES:
            raise RuntimeError(f"response exceeded {MAX_RESPONSE_BYTES} bytes")
        if status != 200:
            raise RuntimeError(f"HTTP {status} {reason}")
        try:
            decoded = json.loads(content.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("Client returned invalid JSON") from exc
        if not isinstance(decoded, dict):
            raise RuntimeError("Client returned a non-object JSON response")
        return decoded
    finally:
        if writer is not None:
            writer.close()


def _settings_from_response(payload: Mapping[str, Any]) -> dict[str, Any]:
    if payload.get("ok") is not True:
        raise RuntimeError("settings response did not report ok=true")
    settings = payload.get("settings")
    if not isinstance(settings, dict):
        raise RuntimeError("settings response has no settings object")
    return settings


def _websocket_uri(port: int, path: str = EVENTS_PATH) -> str:
    if path not in ALLOWED_WEBSOCKET_PATHS:
        raise RuntimeError(f"WebSocket path is outside the read-only allowlist: {path}")
    return f"ws://{LOOPBACK_HOST}:{port}{path}"


def _settings_for_event_observation(
    settings: Mapping[str, Any],
) -> dict[str, Any]:
    """Copy settings while preventing the event endpoint from starting audio I/O."""

    observed = dict(settings)
    for key in AUDIO_ROUTE_SETTING_KEYS:
        observed[key] = ""
    return observed


async def _open_loopback_socket(port: int, timeout: float) -> socket.socket:
    """Pre-connect exactly to IPv4 loopback so redirects cannot change origin."""

    loop = asyncio.get_running_loop()
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.setblocking(False)
    try:
        await asyncio.wait_for(
            loop.sock_connect(connection, (LOOPBACK_HOST, port)),
            timeout=timeout,
        )
    except BaseException:
        connection.close()
        raise
    return connection


def _record(
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
    record_type: str,
    *,
    observed_monotonic_ns: int,
    **values: Any,
) -> None:
    writer.write(
        {
            "record_type": record_type,
            "observed_at_utc": _utc_now(),
            "elapsed_seconds": tracker.elapsed_seconds(observed_monotonic_ns),
            **values,
        }
    )


def _announce_ready_if_needed(
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
    *,
    observed_monotonic_ns: int,
) -> None:
    if (
        tracker.operator_ready_announced
        or not tracker.websocket_accepted
        or not tracker.saw_enabled
        or tracker.pre_disable_plan_samples == 0
        or tracker.pre_disable_plan_issue_samples != 0
        or tracker.disable_observed_at_monotonic_ns is not None
    ):
        return
    tracker.operator_ready_announced = True
    tracker.operator_ready_at_utc = _utc_now()
    tracker.operator_ready_at_monotonic_ns = observed_monotonic_ns
    # The operator is instructed to click only after this boundary.  Discard
    # any earlier target events once, freeze the current transcript baseline,
    # then retain both until the first confirmed disabled status, including
    # across stale enabled replies.
    tracker.pending_transition_event_counts = {
        kind: 0 for kind in LATE_EVENT_KINDS
    }
    _record(
        writer,
        tracker,
        "operator_ready",
        observed_monotonic_ns=observed_monotonic_ns,
        instruction="disable Hands-free once now",
    )
    print(
        "READY: event stream accepted, Hands-free enabled, active plans zero; "
        "disable Hands-free once now",
        flush=True,
    )


async def _poll_readonly_endpoint(
    *,
    port: int,
    settings: Mapping[str, Any],
    deadline: float,
    poll_interval_seconds: float,
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
    path: str,
    record_type: str,
) -> None:
    if ("POST", path) not in ALLOWED_HTTP_OPERATIONS:
        raise RuntimeError(f"poll endpoint is outside the read-only allowlist: {path}")
    loop = asyncio.get_running_loop()
    while loop.time() < deadline:
        cycle_started = loop.time()
        remaining = deadline - loop.time()
        if remaining <= 0:
            return
        try:
            payload = await asyncio.wait_for(
                _request_json(
                    "POST",
                    path,
                    port=port,
                    settings=settings,
                ),
                timeout=min(HTTP_TIMEOUT_SECONDS, remaining),
            )
            observed_ns = time.monotonic_ns()
            if path == STATUS_PATH:
                analysis = tracker.observe_status(
                    payload,
                    observed_monotonic_ns=observed_ns,
                    observed_at_utc=_utc_now(),
                )
            elif path == PLANS_PATH:
                analysis = tracker.observe_plans(
                    payload, observed_monotonic_ns=observed_ns
                )
            else:
                raise RuntimeError(f"unsupported read-only poll endpoint: {path}")
            _announce_ready_if_needed(
                writer,
                tracker,
                observed_monotonic_ns=observed_ns,
            )
            _record(
                writer,
                tracker,
                record_type,
                observed_monotonic_ns=observed_ns,
                analysis=analysis,
                response=payload,
            )
        except asyncio.TimeoutError:
            message = f"{record_type} request exceeded its bounded timeout"
            tracker.observe_error(record_type, message)
            _record(
                writer,
                tracker,
                "observer_error",
                observed_monotonic_ns=time.monotonic_ns(),
                source=record_type,
                message=message,
            )
        except Exception as exc:
            message = str(exc)
            tracker.observe_error(record_type, message)
            _record(
                writer,
                tracker,
                "observer_error",
                observed_monotonic_ns=time.monotonic_ns(),
                source=record_type,
                message=message.replace("\n", " ")[:500],
            )

        sleep_seconds = min(
            max(0.0, poll_interval_seconds - (loop.time() - cycle_started)),
            max(0.0, deadline - loop.time()),
        )
        if sleep_seconds:
            await asyncio.sleep(sleep_seconds)


async def _poll_readonly_endpoints(
    *,
    port: int,
    settings: Mapping[str, Any],
    deadline: float,
    poll_interval_seconds: float,
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
) -> None:
    await asyncio.gather(
        _poll_readonly_endpoint(
            port=port,
            settings=settings,
            deadline=deadline,
            poll_interval_seconds=poll_interval_seconds,
            writer=writer,
            tracker=tracker,
            path=STATUS_PATH,
            record_type="handsfree_status",
        ),
        _poll_readonly_endpoint(
            port=port,
            settings=settings,
            deadline=deadline,
            poll_interval_seconds=poll_interval_seconds,
            writer=writer,
            tracker=tracker,
            path=PLANS_PATH,
            record_type="active_plans",
        ),
    )


async def _observe_websocket(
    *,
    port: int,
    settings: Mapping[str, Any],
    deadline: float,
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
) -> None:
    try:
        import websockets
    except ImportError as exc:
        message = "Python package 'websockets' is unavailable"
        tracker.observe_error("handsfree_websocket", message)
        _record(
            writer,
            tracker,
            "observer_error",
            observed_monotonic_ns=time.monotonic_ns(),
            source="handsfree_websocket",
            message=message,
        )
        return

    loop = asyncio.get_running_loop()
    remaining = deadline - loop.time()
    if remaining <= 0:
        return
    uri = _websocket_uri(port)
    loopback_socket: socket.socket | None = None
    try:
        loopback_socket = await _open_loopback_socket(
            port, min(HTTP_TIMEOUT_SECONDS, remaining)
        )
        async with websockets.connect(
            uri,
            sock=loopback_socket,
            proxy=None,
            open_timeout=min(HTTP_TIMEOUT_SECONDS, remaining),
            close_timeout=0.0,
            max_size=MAX_RESPONSE_BYTES,
            ping_interval=10.0,
            ping_timeout=5.0,
        ) as websocket:
            tracker.websocket_connected = True
            connected_ns = time.monotonic_ns()
            _record(
                writer,
                tracker,
                "websocket_connected",
                observed_monotonic_ns=connected_ns,
                path=EVENTS_PATH,
            )
            await websocket.send(
                json.dumps(
                    {"settings": _settings_for_event_observation(settings)},
                    ensure_ascii=False,
                )
            )

            while loop.time() < deadline:
                remaining = deadline - loop.time()
                try:
                    raw_message = await asyncio.wait_for(
                        websocket.recv(), timeout=remaining
                    )
                except asyncio.TimeoutError:
                    tracker.websocket_covered_to_deadline = True
                    return
                if isinstance(raw_message, bytes):
                    raw_message = raw_message.decode("utf-8")
                if len(raw_message.encode("utf-8")) > MAX_RESPONSE_BYTES:
                    raise RuntimeError("WebSocket message exceeded the size limit")
                payload = json.loads(raw_message)
                if not isinstance(payload, dict):
                    raise RuntimeError("WebSocket message was not a JSON object")
                observed_ns = time.monotonic_ns()
                analysis = tracker.observe_websocket(
                    payload, observed_monotonic_ns=observed_ns
                )
                _announce_ready_if_needed(
                    writer,
                    tracker,
                    observed_monotonic_ns=observed_ns,
                )
                _record(
                    writer,
                    tracker,
                    "handsfree_websocket",
                    observed_monotonic_ns=observed_ns,
                    analysis=analysis,
                    message=payload,
                )
            tracker.websocket_covered_to_deadline = True
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        if loop.time() >= deadline:
            tracker.websocket_covered_to_deadline = True
        else:
            message = str(exc).replace("\n", " ")[:500]
            tracker.observe_error("handsfree_websocket", message)
            _record(
                writer,
                tracker,
                "observer_error",
                observed_monotonic_ns=time.monotonic_ns(),
                source="handsfree_websocket",
                message=message,
            )
    finally:
        if loopback_socket is not None:
            loopback_socket.close()


async def observe_live(
    *,
    port: int,
    duration_seconds: float,
    poll_interval_seconds: float,
    writer: EvidenceWriter,
    tracker: AcceptanceTracker,
) -> None:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + duration_seconds
    try:
        settings_response = await asyncio.wait_for(
            _request_json(
                "GET",
                SETTINGS_PATH,
                port=port,
            ),
            timeout=min(HTTP_TIMEOUT_SECONDS, duration_seconds),
        )
        settings = _settings_from_response(settings_response)
        tracker.settings_loaded = True
        _record(
            writer,
            tracker,
            "settings_loaded",
            observed_monotonic_ns=time.monotonic_ns(),
            settings_keys=sorted(str(key) for key in settings),
        )
    except Exception as exc:
        message = str(exc).replace("\n", " ")[:500]
        tracker.observe_error("settings", message)
        _record(
            writer,
            tracker,
            "observer_error",
            observed_monotonic_ns=time.monotonic_ns(),
            source="settings",
            message=message,
        )
        return

    await asyncio.gather(
        _poll_readonly_endpoints(
            port=port,
            settings=settings,
            deadline=deadline,
            poll_interval_seconds=poll_interval_seconds,
            writer=writer,
            tracker=tracker,
        ),
        _observe_websocket(
            port=port,
            settings=settings,
            deadline=deadline,
            writer=writer,
            tracker=tracker,
        ),
    )


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    if not arguments.observe_live:
        print(
            "refusing live observation without the explicit --observe-live gate",
            file=sys.stderr,
        )
        return 2

    try:
        output_dir = _prepare_output(arguments.output_dir)
    except RuntimeError as exc:
        print(f"output error: {exc}", file=sys.stderr)
        return 2

    tracker = AcceptanceTracker()
    writer: EvidenceWriter | None = None
    interrupted = False
    try:
        writer = EvidenceWriter(output_dir)
        _record(
            writer,
            tracker,
            "observer_started",
            observed_monotonic_ns=tracker.started_monotonic_ns,
            client_host=LOOPBACK_HOST,
            client_port=arguments.client_port,
            requested_duration_seconds=arguments.duration_seconds,
            poll_interval_seconds=arguments.poll_interval_seconds,
        )
        try:
            asyncio.run(
                observe_live(
                    port=arguments.client_port,
                    duration_seconds=arguments.duration_seconds,
                    poll_interval_seconds=arguments.poll_interval_seconds,
                    writer=writer,
                    tracker=tracker,
                )
            )
        except KeyboardInterrupt:
            interrupted = True
            tracker.observe_error("observer", "interrupted by operator")
        except Exception as exc:
            tracker.observe_error("observer", str(exc))

        finished_ns = time.monotonic_ns()
        summary = tracker.summary(
            requested_duration_seconds=arguments.duration_seconds,
            poll_interval_seconds=arguments.poll_interval_seconds,
            finished_monotonic_ns=finished_ns,
            interrupted=interrupted,
        )
        _record(
            writer,
            tracker,
            "observer_finished",
            observed_monotonic_ns=finished_ns,
            acceptance_passed=summary["acceptance"]["passed"],
            failure_reasons=summary["acceptance"]["failure_reasons"],
        )
        writer.finalize(summary)
    except Exception as exc:
        if writer is not None:
            writer.abort()
        print(f"observer failed: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    print(f"evidence: {output_dir}")
    if interrupted:
        return 130
    return 0 if summary["acceptance"]["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
