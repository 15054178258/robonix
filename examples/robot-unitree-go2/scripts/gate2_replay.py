#!/usr/bin/env python3
"""Gate 2 isolated rosbag replay acceptance runner.

This runner is intentionally unable to reach a physical Go2.  Every child is
placed in a localhost-only ROS domain and the only /cmd_vel consumer is the
subscription-only sink in gate2_noop_cmd_vel_sink.py.  A fixture can exercise
the evaluator, but fixture evidence is always reported as FIXTURE_ONLY and can
never become an acceptance pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import Any, Iterable
import urllib.error
import urllib.request

import yaml


ROOT = Path(__file__).resolve().parents[1]
SKIP_EXIT = 77
FORBIDDEN_TOPICS = {"/api/sport/request", "/cmd_vel", "/lowcmd"}
EXPECTED_PROCESSES = (
    "cmd_vel_sink",
    "observer",
    "robot_state_publisher",
    "pointcloud_to_laserscan",
    "mapping",
    "nav2",
    "dashboard",
)


class ConfigurationError(ValueError):
    """A scenario or evidence file violates the Gate 2 contract."""


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"cannot load YAML {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigurationError(f"{path} must contain a YAML object")
    return value


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConfigurationError(f"cannot load JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ConfigurationError(f"{path} must contain a JSON object")
    return value


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _inside_root(raw: Any, label: str) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise ConfigurationError(f"{label} must be a non-empty repository-relative path")
    candidate = Path(raw)
    if candidate.is_absolute():
        raise ConfigurationError(f"{label} must be repository-relative")
    resolved = (ROOT / candidate).resolve()
    try:
        resolved.relative_to(ROOT)
    except ValueError as exc:
        raise ConfigurationError(f"{label} escapes the repository") from exc
    return resolved


def _number(value: Any, label: str, *, minimum: float | None = None) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ConfigurationError(f"{label} must be finite")
    if minimum is not None and result < minimum:
        raise ConfigurationError(f"{label} must be >= {minimum}")
    return result


def _topic(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or len(value) > 200
        or any(character.isspace() for character in value)
    ):
        raise ConfigurationError(f"{label} must be a bounded absolute ROS topic")
    return value


def _window(value: Any, label: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ConfigurationError(f"{label} must be [start_s, end_s]")
    start = _number(value[0], f"{label}[0]", minimum=0.0)
    end = _number(value[1], f"{label}[1]", minimum=0.0)
    if end <= start:
        raise ConfigurationError(f"{label} end must be greater than start")
    return start, end


def validate_scenario(raw: dict[str, Any]) -> dict[str, Any]:
    """Normalize a scenario without touching ROS or the filesystem."""

    if raw.get("schema_version") != 1:
        raise ConfigurationError("schema_version must be 1")
    name = raw.get("name")
    if not isinstance(name, str) or not name.strip() or len(name) > 100:
        raise ConfigurationError("name must be a non-empty string of at most 100 chars")

    runtime = raw.get("runtime")
    if not isinstance(runtime, dict):
        raise ConfigurationError("runtime must be an object")
    try:
        domain_id = int(runtime.get("domain_id"))
    except (TypeError, ValueError) as exc:
        raise ConfigurationError("runtime.domain_id must be an integer") from exc
    if domain_id < 1 or domain_id > 232:
        raise ConfigurationError("runtime.domain_id must be between 1 and 232")
    max_wall_seconds = _number(
        runtime.get("max_wall_seconds", 240),
        "runtime.max_wall_seconds",
        minimum=10.0,
    )
    if max_wall_seconds > 1800:
        raise ConfigurationError("runtime.max_wall_seconds must be <= 1800")
    play_rate = _number(runtime.get("play_rate", 1.0), "runtime.play_rate", minimum=0.05)
    if play_rate > 4.0:
        raise ConfigurationError("runtime.play_rate must be <= 4.0")
    dashboard_port = int(
        _number(runtime.get("dashboard_port", 18092), "runtime.dashboard_port")
    )
    if dashboard_port < 1024 or dashboard_port > 65535:
        raise ConfigurationError("runtime.dashboard_port must be between 1024 and 65535")

    source = raw.get("source")
    if not isinstance(source, dict):
        raise ConfigurationError("source must be an object")
    bag_path = _inside_root(source.get("bag"), "source.bag")
    entries = source.get("replay_topics")
    if not isinstance(entries, list) or not entries:
        raise ConfigurationError("source.replay_topics must be a non-empty list")
    replay_topics: list[dict[str, Any]] = []
    seen_topics: set[str] = set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ConfigurationError(f"source.replay_topics[{index}] must be an object")
        topic_name = _topic(entry.get("topic"), f"source.replay_topics[{index}].topic")
        message_type = entry.get("type")
        if not isinstance(message_type, str) or "/msg/" not in message_type:
            raise ConfigurationError(
                f"source.replay_topics[{index}].type must be a ROS message type"
            )
        if topic_name in seen_topics:
            raise ConfigurationError(f"duplicate replay topic: {topic_name}")
        if topic_name in FORBIDDEN_TOPICS:
            raise ConfigurationError(f"motion topic may not be replayed: {topic_name}")
        seen_topics.add(topic_name)
        replay_topics.append(
            {"topic": topic_name, "type": message_type, "required": bool(entry.get("required", True))}
        )

    topics = raw.get("topics")
    if not isinstance(topics, dict):
        raise ConfigurationError("topics must be an object")
    normalized_topics = {
        key: _topic(topics.get(key), f"topics.{key}")
        for key in (
            "odom",
            "cloud",
            "imu",
            "camera",
            "tf",
            "tf_static",
            "scan",
            "map",
            "local_costmap",
            "global_costmap",
            "nav_status",
            "cmd_vel",
        )
    }
    if normalized_topics["cmd_vel"] != "/cmd_vel":
        raise ConfigurationError("topics.cmd_vel must be exactly /cmd_vel for the no-op sink")
    generated = {normalized_topics["scan"], normalized_topics["map"], normalized_topics["local_costmap"], normalized_topics["global_costmap"], normalized_topics["nav_status"], "/cmd_vel"}
    accidental_outputs = seen_topics & generated
    if accidental_outputs:
        raise ConfigurationError(
            f"generated/command topics may not be replayed: {sorted(accidental_outputs)}"
        )

    frames = raw.get("frames")
    if not isinstance(frames, dict):
        raise ConfigurationError("frames must be an object")
    normalized_frames: dict[str, str] = {}
    for key in ("map", "odom", "base"):
        value = frames.get(key)
        if not isinstance(value, str) or not value or len(value) > 100 or value.startswith("/"):
            raise ConfigurationError(f"frames.{key} must be a bounded TF frame without leading slash")
        normalized_frames[key] = value

    map_config = raw.get("map")
    if not isinstance(map_config, dict):
        raise ConfigurationError("map must be an object")
    database = _inside_root(map_config.get("database"), "map.database")
    if map_config.get("mode") != "localization":
        raise ConfigurationError("map.mode must be localization for Gate 2")

    goal = raw.get("goal")
    if not isinstance(goal, dict):
        raise ConfigurationError("goal must be an object")
    normalized_goal = {
        "x": _number(goal.get("x"), "goal.x"),
        "y": _number(goal.get("y"), "goal.y"),
        "yaw": _number(goal.get("yaw"), "goal.yaw"),
        "dispatch_at_s": _number(goal.get("dispatch_at_s"), "goal.dispatch_at_s", minimum=0.0),
        "cancel_at_s": _number(goal.get("cancel_at_s"), "goal.cancel_at_s", minimum=0.0),
    }
    if normalized_goal["cancel_at_s"] <= normalized_goal["dispatch_at_s"]:
        raise ConfigurationError("goal.cancel_at_s must be after dispatch_at_s")

    phases = raw.get("phases")
    if not isinstance(phases, dict):
        raise ConfigurationError("phases must be an object")
    normalized_phases = {
        key: list(_window(phases.get(key), f"phases.{key}"))
        for key in ("obstacle", "cleared", "sensor_gap")
    }
    if normalized_phases["cleared"][0] < normalized_phases["obstacle"][1]:
        raise ConfigurationError("cleared phase must start after obstacle phase")

    costmaps = raw.get("costmaps")
    if not isinstance(costmaps, dict):
        raise ConfigurationError("costmaps must be an object")
    regions: dict[str, dict[str, Any]] = {}
    for name_key, expected_frame in (("local", normalized_frames["odom"]), ("global", normalized_frames["map"])):
        region = costmaps.get(name_key)
        if not isinstance(region, dict):
            raise ConfigurationError(f"costmaps.{name_key} must be an object")
        if region.get("frame_id") != expected_frame:
            raise ConfigurationError(
                f"costmaps.{name_key}.frame_id must be {expected_frame!r}"
            )
        regions[name_key] = {
            "frame_id": expected_frame,
            "x": _number(region.get("x"), f"costmaps.{name_key}.x"),
            "y": _number(region.get("y"), f"costmaps.{name_key}.y"),
            "radius": _number(region.get("radius"), f"costmaps.{name_key}.radius", minimum=0.05),
        }
    normalized_costmaps = {
        "lethal_threshold": int(_number(costmaps.get("lethal_threshold", 100), "costmaps.lethal_threshold", minimum=1)),
        "min_marked_cells": int(_number(costmaps.get("min_marked_cells", 1), "costmaps.min_marked_cells", minimum=1)),
        "max_cleared_cells": int(_number(costmaps.get("max_cleared_cells", 0), "costmaps.max_cleared_cells", minimum=0)),
        "min_cleared_delta": int(_number(costmaps.get("min_cleared_delta", 1), "costmaps.min_cleared_delta", minimum=1)),
        **regions,
    }

    stop = raw.get("stop")
    if not isinstance(stop, dict):
        raise ConfigurationError("stop must be an object")
    normalized_stop = {
        "zero_epsilon": _number(stop.get("zero_epsilon", 1.0e-4), "stop.zero_epsilon", minimum=0.0),
        "grace_s": _number(stop.get("grace_s", 0.5), "stop.grace_s", minimum=0.0),
        "velocity_timeout_s": _number(stop.get("velocity_timeout_s", 0.25), "stop.velocity_timeout_s", minimum=0.05),
        "require_nonzero_before_gap": bool(stop.get("require_nonzero_before_gap", True)),
    }

    return {
        "schema_version": 1,
        "name": name.strip(),
        "runtime": {
            "domain_id": domain_id,
            "max_wall_seconds": max_wall_seconds,
            "play_rate": play_rate,
            "dashboard_port": dashboard_port,
            "startup_delay_s": _number(runtime.get("startup_delay_s", 4.0), "runtime.startup_delay_s", minimum=0.5),
            "post_playback_grace_s": _number(runtime.get("post_playback_grace_s", 1.0), "runtime.post_playback_grace_s", minimum=0.0),
        },
        "source": {"bag": str(bag_path), "replay_topics": replay_topics},
        "topics": normalized_topics,
        "frames": normalized_frames,
        "map": {"database": str(database), "mode": "localization"},
        "goal": normalized_goal,
        "phases": normalized_phases,
        "costmaps": normalized_costmaps,
        "stop": normalized_stop,
    }


def _bag_metadata(bag_path: Path) -> dict[str, Any]:
    metadata_path = bag_path / "metadata.yaml" if bag_path.is_dir() else bag_path.parent / "metadata.yaml"
    metadata = _load_yaml(metadata_path)
    info = metadata.get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise ConfigurationError(f"{metadata_path} has no rosbag2_bagfile_information")
    return info


def _metadata_topics(info: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    entries = info.get("topics_with_message_count")
    if not isinstance(entries, list):
        raise ConfigurationError("rosbag metadata has no topics_with_message_count list")
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        metadata = entry.get("topic_metadata")
        if not isinstance(metadata, dict):
            continue
        name = metadata.get("name")
        message_type = metadata.get("type")
        if isinstance(name, str) and isinstance(message_type, str):
            result[name] = {"type": message_type, "count": int(entry.get("message_count", 0))}
    return result


def _duration_seconds(info: dict[str, Any]) -> float:
    duration = info.get("duration")
    if isinstance(duration, dict):
        nanoseconds = duration.get("nanoseconds")
    else:
        nanoseconds = duration
    return _number(nanoseconds, "rosbag duration.nanoseconds", minimum=1.0) / 1_000_000_000.0


def _fingerprint_bag(path: Path) -> str:
    digest = hashlib.sha256()
    files = [path] if path.is_file() else sorted(item for item in path.rglob("*") if item.is_file())
    for item in files:
        relative = item.name if path.is_file() else str(item.relative_to(path))
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        with item.open("rb") as stream:
            while True:
                block = stream.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
    return digest.hexdigest()


def preflight(scenario: dict[str, Any], environment: dict[str, str]) -> dict[str, Any]:
    """Return readiness details.  Missing inputs/dependencies are SKIP reasons."""

    errors: list[str] = []
    skipped: list[str] = []
    bag_path = Path(scenario["source"]["bag"])
    map_database = Path(scenario["map"]["database"])
    info: dict[str, Any] | None = None
    duration = 0.0
    fingerprint = ""
    if not bag_path.exists():
        skipped.append(f"real rosbag is absent: {bag_path}")
    else:
        try:
            info = _bag_metadata(bag_path)
            observed = _metadata_topics(info)
            forbidden = sorted(FORBIDDEN_TOPICS & observed.keys())
            if forbidden:
                errors.append(f"rosbag contains forbidden motion topic(s): {forbidden}")
            for expected in scenario["source"]["replay_topics"]:
                actual = observed.get(expected["topic"])
                if actual is None:
                    if expected["required"]:
                        errors.append(f"required rosbag topic missing: {expected['topic']}")
                    continue
                if actual["type"] != expected["type"]:
                    errors.append(
                        f"{expected['topic']} type is {actual['type']}, expected {expected['type']}"
                    )
                if expected["required"] and actual["count"] <= 0:
                    errors.append(f"required rosbag topic is empty: {expected['topic']}")
            duration = _duration_seconds(info)
            latest_required = max(
                scenario["goal"]["cancel_at_s"],
                *(window[1] for window in scenario["phases"].values()),
            )
            if duration < latest_required:
                errors.append(
                    f"rosbag duration {duration:.3f}s is shorter than required event {latest_required:.3f}s"
                )
            estimated_wall = (
                duration / scenario["runtime"]["play_rate"]
                + scenario["runtime"]["startup_delay_s"]
                + scenario["runtime"]["post_playback_grace_s"]
            )
            if estimated_wall >= scenario["runtime"]["max_wall_seconds"]:
                errors.append(
                    "runtime.max_wall_seconds is too short for the bag at the configured play rate"
                )
            fingerprint = _fingerprint_bag(bag_path)
        except ConfigurationError as exc:
            errors.append(str(exc))
    if not map_database.is_file():
        skipped.append(f"saved RTAB-Map database is absent: {map_database}")

    required_paths = {
        "mapping launch": ROOT.parent.parent / "upstream" / "service-map-rbnx" / "launch" / "rtabmap_2d.launch.py",
        "URDF": ROOT / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf",
        "dashboard Python": ROOT / "packages" / "go2_dashboard" / "rbnx-build" / "venv" / "bin" / "python",
    }
    for label, path in required_paths.items():
        if not path.exists():
            skipped.append(f"{label} is absent: {path}")
    if shutil.which("ros2", path=environment.get("PATH")) is None:
        skipped.append("ros2 is not available; source /opt/ros/humble/setup.bash")
    else:
        for package in (
            "nav2_bringup",
            "pointcloud_to_laserscan",
            "robot_state_publisher",
            "rtabmap_slam",
        ):
            result = subprocess.run(
                ["ros2", "pkg", "prefix", package],
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=8,
            )
            if result.returncode != 0:
                skipped.append(f"ROS package is absent: {package}")

    if not skipped and not errors:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.bind(("127.0.0.1", scenario["runtime"]["dashboard_port"]))
        except OSError as exc:
            errors.append(
                f"dashboard loopback port {scenario['runtime']['dashboard_port']} is unavailable: {exc}"
            )
        result = subprocess.run(
            ["ros2", "node", "list"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
            timeout=8,
        )
        if result.returncode != 0:
            errors.append(f"cannot audit isolated ROS graph: {result.stderr.strip()}")
        elif result.stdout.strip():
            errors.append(
                "isolated ROS domain is not empty before replay: "
                + ", ".join(line for line in result.stdout.splitlines() if line.strip())
            )

    return {
        "ready": not errors and not skipped,
        "errors": errors,
        "skip_reasons": skipped,
        "bag_duration_s": duration,
        "bag_sha256": fingerprint,
        "domain_id": scenario["runtime"]["domain_id"],
        "localhost_only": True,
    }


def isolated_environment(scenario: dict[str, Any], run_dir: Path) -> dict[str, str]:
    environment = os.environ.copy()
    for name in tuple(environment):
        if name.startswith("GO2_") or name.startswith("UNITREE_"):
            environment.pop(name, None)
    environment.update(
        {
            "ROS_DOMAIN_ID": str(scenario["runtime"]["domain_id"]),
            "ROS_LOCALHOST_ONLY": "1",
            "ROS2CLI_NO_DAEMON": "1",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "CYCLONEDDS_URI": (
                "<CycloneDDS><Domain><General>"
                "<AllowMulticast>false</AllowMulticast>"
                "</General></Domain></CycloneDDS>"
            ),
            "GO2_ALLOW_MOTION": "false",
            "GO2_OPERATOR_PRESENT": "false",
            "GO2_DASHBOARD_BROWSER_VOICE_ENABLED": "0",
            "GO2_DASHBOARD_HOST": "127.0.0.1",
            "GO2_DASHBOARD_PORT": str(scenario["runtime"]["dashboard_port"]),
            "GO2_DASHBOARD_CAMERA_TOPIC": scenario["topics"]["camera"],
            "GO2_DASHBOARD_SCAN_TOPIC": scenario["topics"]["scan"],
            "GO2_DASHBOARD_CLOUD_TOPIC": scenario["topics"]["cloud"],
            "GO2_DASHBOARD_MAP_TOPIC": scenario["topics"]["map"],
            "GO2_DASHBOARD_ODOM_TOPIC": scenario["topics"]["odom"],
            "GO2_DASHBOARD_NAV_STATUS_TOPIC": scenario["topics"]["nav_status"],
            "GO2_DASHBOARD_MAP_FRAME": scenario["frames"]["map"],
            "GO2_DASHBOARD_BASE_FRAME": scenario["frames"]["base"],
            "GO2_DASHBOARD_PID_FILE": str(run_dir / "dashboard.pid"),
            "ROS_LOG_DIR": str(run_dir / "ros-logs"),
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    dashboard_python_path = str(ROOT / "packages" / "go2_dashboard")
    previous_python_path = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = (
        dashboard_python_path
        if not previous_python_path
        else dashboard_python_path + os.pathsep + previous_python_path
    )
    return environment


def _write_runtime_files(scenario: dict[str, Any], run_dir: Path) -> dict[str, Path]:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "ros-logs").mkdir(exist_ok=True)
    nav_template = (ROOT / "config" / "nav2_params_go2.yaml").read_text(encoding="utf-8")
    substitutions = {
        "__ROBONIX_MAP_TOPIC__": scenario["topics"]["map"],
        "__ROBONIX_ODOM_TOPIC__": scenario["topics"]["odom"],
        "__ROBONIX_SCAN_TOPIC__": scenario["topics"]["scan"],
        "__ROBONIX_BT_XML__": str(ROOT / "config" / "navigate.xml"),
        "__ROBONIX_BT_THROUGH_POSES_XML__": str(
            ROOT / "config" / "navigate_through_poses.xml"
        ),
        "__ROBONIX_FOOTPRINT__": "[[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]",
    }
    for token, value in substitutions.items():
        nav_template = nav_template.replace(token, value)
    nav_template = nav_template.replace("use_sim_time: false", "use_sim_time: true")
    if "__ROBONIX_" in nav_template:
        raise ConfigurationError("unresolved token remains in Nav2 parameters")
    nav_path = run_dir / "nav2_params.yaml"
    nav_path.write_text(nav_template, encoding="utf-8")
    yaml.safe_load(nav_template)

    overrides = _load_yaml(ROOT / "config" / "rtabmap_params.yaml")
    override_path = run_dir / "rtabmap_overrides.json"
    override_path.write_text(json.dumps(overrides, sort_keys=True), encoding="utf-8")

    urdf = (ROOT / "packages" / "go2_description" / "urdf" / "go2_robonix.urdf").read_text(encoding="utf-8")
    rsp_path = run_dir / "robot_state_publisher.yaml"
    rsp_path.write_text(
        yaml.safe_dump(
            {
                "robot_state_publisher": {
                    "ros__parameters": {"use_sim_time": True, "robot_description": urdf}
                }
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    scenario_path = run_dir / "scenario.resolved.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8")
    map_copy = run_dir / "localization.db"
    shutil.copy2(Path(scenario["map"]["database"]), map_copy)
    return {
        "nav": nav_path,
        "overrides": override_path,
        "rsp": rsp_path,
        "scenario": scenario_path,
        "map_db": map_copy,
    }


class OwnedProcess:
    def __init__(self, name: str, command: list[str], environment: dict[str, str], run_dir: Path) -> None:
        self.name = name
        self.command = command
        self.log_path = run_dir / f"{name}.log"
        self._log = self.log_path.open("wb")
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def poll(self) -> int | None:
        return self.process.poll()

    def stop(self) -> None:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)

    def terminate(self) -> None:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)

    def kill(self) -> None:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGKILL)

    def close(self) -> None:
        self._log.close()


def _dashboard_snapshot(port: int) -> dict[str, Any] | None:
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/status", headers={"Cache-Control": "no-store"}
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=0.8) as response:
            value = json.loads(response.read().decode("utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        return None


def _start_commands(scenario: dict[str, Any], paths: dict[str, Path], run_dir: Path) -> list[tuple[str, list[str]]]:
    topics = scenario["topics"]
    frames = scenario["frames"]
    mapping_launch = ROOT.parent.parent / "upstream" / "service-map-rbnx" / "launch" / "rtabmap_2d.launch.py"
    dashboard_python = ROOT / "packages" / "go2_dashboard" / "rbnx-build" / "venv" / "bin" / "python"
    return [
        (
            "cmd_vel_sink",
            [sys.executable, str(ROOT / "scripts" / "gate2_noop_cmd_vel_sink.py"), "--topic", topics["cmd_vel"], "--output", str(run_dir / "cmd_vel_sink.json")],
        ),
        (
            "observer",
            [sys.executable, str(ROOT / "scripts" / "gate2_observer.py"), "--scenario", str(paths["scenario"]), "--output", str(run_dir / "observer.json")],
        ),
        (
            "robot_state_publisher",
            ["ros2", "run", "robot_state_publisher", "robot_state_publisher", "--ros-args", "--params-file", str(paths["rsp"])],
        ),
        (
            "pointcloud_to_laserscan",
            [
                "ros2", "run", "pointcloud_to_laserscan", "pointcloud_to_laserscan_node",
                "--ros-args", "-r", f"cloud_in:={topics['cloud']}", "-r", f"scan:={topics['scan']}",
                "-p", "use_sim_time:=true", "-p", f"target_frame:={frames['base']}",
                "-p", "transform_tolerance:=0.20", "-p", "min_height:=-0.20", "-p", "max_height:=1.50",
                "-p", "angle_min:=-3.14159", "-p", "angle_max:=3.14159", "-p", "angle_increment:=0.00872665",
                "-p", "scan_time:=0.10", "-p", "range_min:=0.30", "-p", "range_max:=9.0", "-p", "use_inf:=true",
            ],
        ),
        (
            "mapping",
            [
                "ros2", "launch", str(mapping_launch), "use_sim_time:=true", "enable_viz:=false",
                "scan_topic:=<none>", f"scan_cloud_topic:={topics['cloud']}", f"odom_topic:={topics['odom']}",
                "rgb_topic:=<none>", "depth_topic:=<none>", f"imu_topic:={topics['imu']}",
                f"base_frame:={frames['base']}", f"odom_frame:={frames['odom']}", "deskew_lidar:=false",
                f"database_path:={paths['map_db']}", "map_mode:=localization", "reset_map:=false",
                f"rtabmap_overrides_file:={paths['overrides']}",
            ],
        ),
        (
            "nav2",
            ["ros2", "launch", "nav2_bringup", "navigation_launch.py", "use_sim_time:=true", "autostart:=true", f"params_file:={paths['nav']}"],
        ),
        (
            "dashboard",
            [str(dashboard_python), "-m", "go2_dashboard.main", "--host", "127.0.0.1", "--port", str(scenario["runtime"]["dashboard_port"]), "--log-level", "warning"],
        ),
    ]


def _shutdown(processes: Iterable[OwnedProcess]) -> None:
    items = list(processes)
    for process in reversed(items):
        process.stop()
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline and any(process.poll() is None for process in items):
        time.sleep(0.1)
    for process in reversed(items):
        process.terminate()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and any(process.poll() is None for process in items):
        time.sleep(0.1)
    for process in reversed(items):
        process.kill()
    for process in items:
        try:
            process.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            pass
        process.close()


def _check(name: str, passed: bool, detail: str) -> dict[str, Any]:
    return {"name": name, "passed": bool(passed), "detail": detail}


def _samples_in(samples: list[dict[str, Any]], window: list[float]) -> list[dict[str, Any]]:
    return [sample for sample in samples if window[0] <= float(sample.get("t", -1)) <= window[1]]


def evaluate_evidence(scenario: dict[str, Any], evidence: dict[str, Any]) -> dict[str, Any]:
    """Evaluate measured evidence.  Source authenticity remains a required check."""

    checks: list[dict[str, Any]] = []
    source = evidence.get("source", {})
    checks.append(_check("real_rosbag_source", source.get("kind") == "rosbag" and len(str(source.get("sha256", ""))) == 64, f"kind={source.get('kind')!r}"))
    isolation = evidence.get("isolation", {})
    checks.append(_check("localhost_isolated_domain", isolation.get("localhost_only") is True and int(isolation.get("domain_id", -1)) == scenario["runtime"]["domain_id"], f"domain={isolation.get('domain_id')}, localhost_only={isolation.get('localhost_only')}"))

    processes = evidence.get("processes", {})
    for name in EXPECTED_PROCESSES:
        state = processes.get(name, {})
        checks.append(_check(f"process_{name}", state.get("started") is True and state.get("unexpected_exit") is not True, str(state)))

    sink = evidence.get("cmd_vel_sink", {})
    checks.append(_check("cmd_vel_sink_is_subscription_only", sink.get("subscribes_only") is True and int(sink.get("forwarded", -1)) == 0 and sink.get("topic") == "/cmd_vel", f"subscribes_only={sink.get('subscribes_only')}, forwarded={sink.get('forwarded')}"))
    checks.append(
        _check(
            "cmd_vel_sink_valid_samples",
            not sink.get("error") and int(sink.get("invalid_samples", 0)) == 0,
            f"error={sink.get('error')!r}, invalid_samples={sink.get('invalid_samples', 0)}",
        )
    )
    observer = evidence.get("observer", {})
    checks.append(
        _check(
            "observer_no_errors",
            not observer.get("errors") and not observer.get("error"),
            f"errors={observer.get('errors', [])}, error={observer.get('error', '')!r}",
        )
    )
    tf_state = observer.get("tf", {})
    checks.append(_check("tf_map_to_odom", tf_state.get("map_to_odom") is True, str(tf_state)))
    checks.append(_check("tf_odom_to_base", tf_state.get("odom_to_base") is True, str(tf_state)))

    counts = observer.get("counts", {})
    for key in ("map", "odom", "cloud", "scan", "imu", "camera", "nav_status"):
        checks.append(_check(f"topic_{key}", int(counts.get(key, 0)) > 0, f"count={counts.get(key, 0)}"))

    navigation = observer.get("navigation", {})
    checks.extend(
        [
            _check("nav_goal_accepted", navigation.get("goal_accepted") is True, str(navigation)),
            _check("nav_cancel_accepted", navigation.get("cancel_accepted") is True, str(navigation)),
            _check("nav_result_canceled", navigation.get("result") == "CANCELED", str(navigation)),
        ]
    )

    costmap_samples = observer.get("costmaps", {})
    for name in ("local", "global"):
        samples = costmap_samples.get(name, [])
        marked = _samples_in(samples, scenario["phases"]["obstacle"])
        cleared = _samples_in(samples, scenario["phases"]["cleared"])
        marked_max = max((int(sample.get("occupied_cells", 0)) for sample in marked), default=-1)
        cleared_min = min((int(sample.get("occupied_cells", 0)) for sample in cleared), default=10**9)
        passed = (
            bool(marked)
            and bool(cleared)
            and marked_max >= scenario["costmaps"]["min_marked_cells"]
            and cleared_min <= scenario["costmaps"]["max_cleared_cells"]
            and marked_max - cleared_min >= scenario["costmaps"]["min_cleared_delta"]
        )
        checks.append(_check(f"{name}_costmap_mark_and_clear", passed, f"marked_max={marked_max}, cleared_min={cleared_min}, marked_samples={len(marked)}, cleared_samples={len(cleared)}"))

    scan_times = [float(value) for value in observer.get("scan_times", [])]
    gap_start, gap_end = scenario["phases"]["sensor_gap"]
    before = any(value < gap_start for value in scan_times)
    during = any(gap_start <= value <= gap_end for value in scan_times)
    after = any(value > gap_end for value in scan_times)
    checks.append(_check("sensor_gap_observed", before and not during and after, f"before={before}, during={during}, after={after}"))

    cmd_samples = sink.get("samples", [])
    epsilon = scenario["stop"]["zero_epsilon"]
    def magnitude(sample: dict[str, Any]) -> float:
        return max(abs(float(sample.get(axis, 0.0))) for axis in ("x", "y", "z"))
    nonzero_before = any(float(sample.get("t", -1)) < gap_start and magnitude(sample) > epsilon for sample in cmd_samples)
    cutoff = gap_start + scenario["stop"]["grace_s"]
    unsafe_during = [sample for sample in cmd_samples if cutoff <= float(sample.get("t", -1)) <= gap_end and magnitude(sample) > epsilon]
    zero_after_start = any(gap_start <= float(sample.get("t", -1)) <= gap_end and magnitude(sample) <= epsilon for sample in cmd_samples)
    last_before_gap = max((float(sample.get("t", -1)) for sample in cmd_samples if float(sample.get("t", -1)) < gap_start), default=-1.0)
    command_silence = last_before_gap >= 0 and gap_end - last_before_gap >= scenario["stop"]["velocity_timeout_s"]
    require_nonzero = scenario["stop"]["require_nonzero_before_gap"]
    stopped = (nonzero_before or not require_nonzero) and not unsafe_during and (zero_after_start or command_silence)
    checks.append(_check("sensor_loss_stops_cmd_vel", stopped, f"nonzero_before={nonzero_before}, unsafe_after_grace={len(unsafe_during)}, zero_after_start={zero_after_start}, command_silence={command_silence}"))

    snapshots = evidence.get("dashboard_snapshots", [])
    simultaneous = False
    for snapshot in snapshots:
        topics = snapshot.get("topics", {}) if isinstance(snapshot, dict) else {}
        simultaneous = (
            snapshot.get("read_only") is True
            and snapshot.get("telemetry_read_only") is True
            and snapshot.get("bridge", {}).get("connected") is True
            and all(int(topics.get(key, {}).get("sequence", 0)) > 0 for key in ("camera", "point_cloud", "map", "odom", "pose_map", "nav_status"))
        )
        if simultaneous:
            break
    checks.append(_check("dashboard_simultaneous_telemetry", simultaneous, f"snapshots={len(snapshots)}"))

    passed = all(check["passed"] for check in checks)
    return {"acceptance_pass": passed, "checks": checks}


def run_replay(scenario: dict[str, Any], environment: dict[str, str], readiness: dict[str, Any]) -> dict[str, Any]:
    timestamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    run_dir = ROOT / "rbnx-build" / "gate2" / "runs" / timestamp
    paths = _write_runtime_files(scenario, run_dir)
    processes: list[OwnedProcess] = []
    process_evidence: dict[str, dict[str, Any]] = {}
    snapshots: list[dict[str, Any]] = []
    bag_process: OwnedProcess | None = None
    start = time.monotonic()
    failure = ""
    try:
        for name, command in _start_commands(scenario, paths, run_dir):
            owned = OwnedProcess(name, command, environment, run_dir)
            processes.append(owned)
            process_evidence[name] = {"started": True, "pid": owned.process.pid, "command": command}
            time.sleep(0.15)
            code = owned.poll()
            if code is not None:
                process_evidence[name]["unexpected_exit"] = True
                process_evidence[name]["exit_code"] = code
                raise RuntimeError(f"{name} exited during startup with {code}; see {owned.log_path}")
        deadline = time.monotonic() + scenario["runtime"]["startup_delay_s"]
        while time.monotonic() < deadline:
            for owned in processes:
                code = owned.poll()
                if code is not None:
                    process_evidence[owned.name]["unexpected_exit"] = True
                    process_evidence[owned.name]["exit_code"] = code
                    raise RuntimeError(f"{owned.name} exited before replay with {code}; see {owned.log_path}")
            snapshot = _dashboard_snapshot(scenario["runtime"]["dashboard_port"])
            if snapshot is not None:
                snapshots.append(snapshot)
            time.sleep(0.25)
        replay_topics = [entry["topic"] for entry in scenario["source"]["replay_topics"]]
        bag_command = [
            "ros2", "bag", "play", scenario["source"]["bag"],
            "--clock", "50", "--rate", str(scenario["runtime"]["play_rate"]),
            "--disable-keyboard-controls", "--topics", *replay_topics,
        ]
        bag_process = OwnedProcess("rosbag_play", bag_command, environment, run_dir)
        wall_deadline = start + scenario["runtime"]["max_wall_seconds"]
        while bag_process.poll() is None:
            if time.monotonic() >= wall_deadline:
                raise RuntimeError("Gate 2 replay exceeded runtime.max_wall_seconds")
            for owned in processes:
                code = owned.poll()
                if code is not None:
                    process_evidence[owned.name]["unexpected_exit"] = True
                    process_evidence[owned.name]["exit_code"] = code
                    raise RuntimeError(f"{owned.name} exited during replay with {code}; see {owned.log_path}")
            snapshot = _dashboard_snapshot(scenario["runtime"]["dashboard_port"])
            if snapshot is not None:
                snapshots.append(snapshot)
            time.sleep(0.25)
        if bag_process.poll() != 0:
            raise RuntimeError(f"ros2 bag play exited with {bag_process.poll()}; see {bag_process.log_path}")
        time.sleep(scenario["runtime"]["post_playback_grace_s"])
        snapshot = _dashboard_snapshot(scenario["runtime"]["dashboard_port"])
        if snapshot is not None:
            snapshots.append(snapshot)
    except Exception as exc:  # orchestration errors are evidence, not hidden tracebacks
        failure = str(exc)
    finally:
        if bag_process is not None:
            _shutdown([bag_process])
        _shutdown(processes)
        for owned in processes:
            code = owned.poll()
            state = process_evidence.setdefault(owned.name, {"started": True})
            state["exit_code"] = code
        if failure:
            for owned in processes:
                if owned.poll() not in (None, 0, -signal.SIGINT, -signal.SIGTERM, -signal.SIGKILL):
                    process_evidence[owned.name]["unexpected_exit"] = True

    observer_path = run_dir / "observer.json"
    sink_path = run_dir / "cmd_vel_sink.json"
    evidence = {
        "schema_version": 1,
        "source": {"kind": "rosbag", "path": scenario["source"]["bag"], "sha256": readiness["bag_sha256"]},
        "isolation": {"domain_id": scenario["runtime"]["domain_id"], "localhost_only": True, "network_interface": "lo", "motion_enabled": False},
        "processes": process_evidence,
        "observer": _load_json(observer_path) if observer_path.exists() else {},
        "cmd_vel_sink": _load_json(sink_path) if sink_path.exists() else {},
        "dashboard_snapshots": snapshots,
        "orchestration_error": failure,
        "run_dir": str(run_dir),
    }
    evaluation = evaluate_evidence(scenario, evidence)
    report = {"status": "PASS" if evaluation["acceptance_pass"] and not failure else "FAIL", **evaluation, "evidence": evidence}
    _atomic_json(run_dir / "report.json", report)
    return report


def _emit(payload: dict[str, Any], output: Path | None) -> None:
    if output is not None:
        _atomic_json(output, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Gate 2 localhost-only rosbag replay acceptance")
    parser.add_argument("--scenario", type=Path, default=ROOT / "config" / "gate2_replay.example.yaml")
    parser.add_argument("--run", action="store_true", help="run real rosbag replay after preflight")
    parser.add_argument("--fixture-evidence", type=Path, help="evaluate fixture shape; always exits SKIP/FIXTURE_ONLY")
    parser.add_argument("--output", type=Path, help="optional JSON report path inside this repository")
    args = parser.parse_args(argv)
    try:
        scenario = validate_scenario(_load_yaml(args.scenario.resolve()))
        output = args.output.resolve() if args.output else None
        if output is not None:
            try:
                output.relative_to(ROOT)
            except ValueError as exc:
                raise ConfigurationError("--output must remain inside the repository") from exc
        run_dir = ROOT / "rbnx-build" / "gate2" / "preflight"
        run_dir.mkdir(parents=True, exist_ok=True)
        environment = isolated_environment(scenario, run_dir)
        if args.fixture_evidence:
            evidence = _load_json(args.fixture_evidence.resolve())
            evaluation = evaluate_evidence(scenario, evidence)
            payload = {
                "status": "FIXTURE_ONLY",
                "acceptance_pass": False,
                "reason": "fixture evidence exercises the evaluator but is not a real rosbag run",
                "fixture_evaluation": evaluation,
            }
            _emit(payload, output)
            return SKIP_EXIT
        readiness = preflight(scenario, environment)
        if not readiness["ready"]:
            status = "FAIL" if readiness["errors"] else "SKIP"
            payload = {"status": status, "acceptance_pass": False, "preflight": readiness}
            _emit(payload, output)
            return 1 if readiness["errors"] else SKIP_EXIT
        if not args.run:
            payload = {
                "status": "READY_NOT_RUN",
                "acceptance_pass": False,
                "reason": "preflight passed; use --run only for the isolated bag replay",
                "preflight": readiness,
            }
            _emit(payload, output)
            return SKIP_EXIT
        report = run_replay(scenario, environment, readiness)
        _emit(report, output)
        return 0 if report["status"] == "PASS" else 1
    except ConfigurationError as exc:
        payload = {"status": "FAIL", "acceptance_pass": False, "error": str(exc)}
        _emit(payload, None)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
