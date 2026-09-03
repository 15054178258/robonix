"""Offline-safe validation and parameter materialization helpers."""

from __future__ import annotations

import hashlib
from pathlib import Path
import time
from typing import Iterable
import xml.etree.ElementTree as ET


MAX_URDF_BYTES = 2_000_000


def _frame_id(value: str) -> str:
    """Return the canonical TF spelling used for readiness comparisons."""

    return value.strip().lstrip("/")


def validate_urdf(urdf_xml: str) -> tuple[str, int, int]:
    encoded = urdf_xml.encode("utf-8")
    if not encoded or len(encoded) > MAX_URDF_BYTES:
        raise ValueError("URDF is empty or exceeds the bounded size")
    lowered = urdf_xml.lower()
    if "<!doctype" in lowered or "<!entity" in lowered:
        raise ValueError("URDF must not contain DTD or entity declarations")
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError("URDF root must be <robot>")
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [node.attrib.get("name", "").strip() for node in links]
    if not link_names or any(not name for name in link_names):
        raise ValueError("URDF must contain named links")
    if len(set(link_names)) != len(link_names):
        raise ValueError("URDF contains duplicate link names")
    known_links = set(link_names)
    children: list[str] = []
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = "" if parent is None else parent.attrib.get("link", "")
        child_name = "" if child is None else child.attrib.get("link", "")
        if parent_name not in known_links or child_name not in known_links:
            raise ValueError("URDF joint references an unknown link")
        children.append(child_name)
    roots = sorted(known_links - set(children))
    if roots != ["base_link"]:
        raise ValueError(f"URDF root link must be exactly base_link, found {roots}")
    return roots[0], len(links), len(joints)


def fixed_joint_transforms(urdf_xml: str) -> frozenset[tuple[str, str]]:
    """Return every fixed parent/child edge robot_state_publisher must latch."""

    validate_urdf(urdf_xml)
    root = ET.fromstring(urdf_xml)
    pairs = {
        (
            _frame_id(joint.find("parent").attrib["link"]),
            _frame_id(joint.find("child").attrib["link"]),
        )
        for joint in root.findall("joint")
        if joint.attrib.get("type", "").strip().lower() == "fixed"
    }
    if not pairs:
        raise ValueError("URDF must contain at least one fixed transform")
    return frozenset(pairs)


class StaticTransformSentinel:
    """Accumulate latched TF messages until the complete expected tree exists."""

    def __init__(self, expected: Iterable[tuple[str, str]]) -> None:
        self.expected = frozenset(
            (_frame_id(parent), _frame_id(child)) for parent, child in expected
        )
        if not self.expected or any(not parent or not child for parent, child in self.expected):
            raise ValueError("expected static transforms must contain named frames")
        self.observed: set[tuple[str, str]] = set()

    @property
    def ready(self) -> bool:
        return self.expected.issubset(self.observed)

    @property
    def missing(self) -> frozenset[tuple[str, str]]:
        return frozenset(self.expected - self.observed)

    def observe(self, message: object) -> None:
        for transform in getattr(message, "transforms", ()):
            header = getattr(transform, "header", None)
            parent = _frame_id(str(getattr(header, "frame_id", "")))
            child = _frame_id(str(getattr(transform, "child_frame_id", "")))
            if parent and child:
                self.observed.add((parent, child))


def static_transform_qos_profile():
    """Match the standard `/tf_static` reliable, transient-local publisher."""

    from rclpy.qos import (  # type: ignore
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def wait_for_static_transforms(
    expected: Iterable[tuple[str, str]],
    timeout_s: float,
    *,
    topic: str = "/tf_static",
) -> StaticTransformSentinel:
    """Boundedly verify latched static transforms using an isolated ROS context.

    A volatile subscriber created after ``robot_state_publisher`` starts cannot
    receive its already-published static tree.  This late-joining sentinel uses
    the same transient-local QoS as TF and validates the expected URDF edges,
    rather than accepting publisher discovery alone.
    """

    if timeout_s <= 0.0:
        raise ValueError("static transform timeout must be positive")

    import rclpy  # type: ignore
    from rclpy.context import Context  # type: ignore
    from rclpy.executors import SingleThreadedExecutor  # type: ignore
    from tf2_msgs.msg import TFMessage  # type: ignore

    sentinel = StaticTransformSentinel(expected)
    context = Context()
    node = None
    executor = None
    subscription = None
    try:
        # Do not install process-global signal handlers or reuse the capability
        # API's global ROS context from this lifecycle callback.
        context.init(args=None, initialize_logging=False)
        node = rclpy.create_node(
            "go2_description_tf_static_sentinel",
            context=context,
            use_global_arguments=True,
            enable_rosout=False,
            start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        subscription = node.create_subscription(
            TFMessage,
            topic,
            sentinel.observe,
            static_transform_qos_profile(),
        )

        deadline = time.monotonic() + timeout_s
        while not sentinel.ready:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            executor.spin_once(timeout_sec=min(0.1, remaining))
        return sentinel
    finally:
        if node is not None and subscription is not None:
            try:
                node.destroy_subscription(subscription)
            except Exception:
                pass
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        try:
            context.try_shutdown()
        except Exception:
            pass


def require_pinned_urdf(received: str, pinned_path: Path) -> str:
    pinned = pinned_path.read_text(encoding="utf-8")
    validate_urdf(pinned)
    validate_urdf(received)
    if received.encode("utf-8") != pinned.encode("utf-8"):
        expected = hashlib.sha256(pinned.encode("utf-8")).hexdigest()
        actual = hashlib.sha256(received.encode("utf-8")).hexdigest()
        raise ValueError(
            f"Soma URDF does not match pinned Go2 model: expected {expected}, got {actual}"
        )
    return hashlib.sha256(received.encode("utf-8")).hexdigest()


def write_robot_state_publisher_params(path: Path, urdf_xml: str) -> None:
    validate_urdf(urdf_xml)
    indented = "\n".join(f"      {line}" for line in urdf_xml.splitlines())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    robot_description: |\n"
        f"{indented}\n",
        encoding="utf-8",
    )
