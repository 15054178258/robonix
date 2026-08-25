"""Strict, side-effect-free runtime configuration for the Go2 chassis.

Robonix delivers per-instance configuration only through Driver(CMD_INIT).
Keeping normalization in this dependency-free module makes the fail-closed
rules testable without ROS, Robonix, or Unitree SDK2 installed.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import re
import stat
from typing import Mapping


SAFETY_ACK = "I_UNDERSTAND_GO2_CAN_MOVE"
UNKNOWN_MODE = 255
FIRST_MOTION_PROFILE = "workstation-first-motion-corrected-v1"
FIRST_MOTION_COMMAND_TOPIC = "/go2/commissioning/cmd_vel"
FIRST_MOTION_ODOM_TOPIC = "/odom"
FIRST_MOTION_MAX_VX_MPS = 0.05
FIRST_MOTION_MAX_VY_MPS = 0.0
FIRST_MOTION_MAX_WZ_RPS = 0.0
FIRST_MOTION_MAX_DURATION_S = 2.0
FIRST_MOTION_MAX_DISTANCE_M = 0.10
FIRST_MOTION_COMMAND_TIMEOUT_S = 0.20
SECOND_MOTION_PROFILE = "workstation-second-motion-corrected-v1"
SECOND_MOTION_COMMAND_TOPIC = "/go2/second_motion/cmd_vel"
SECOND_MOTION_ODOM_TOPIC = "/odom"
SECOND_MOTION_MAX_VX_MPS = 0.30
SECOND_MOTION_MAX_VY_MPS = 0.0
SECOND_MOTION_MAX_WZ_RPS = 0.0
SECOND_MOTION_MAX_LINEAR_ACCEL_MPS2 = 0.30
SECOND_MOTION_MAX_ANGULAR_ACCEL_RPS2 = 0.10
SECOND_MOTION_MAX_DURATION_S = 1.5
SECOND_MOTION_MAX_DISTANCE_M = 0.30
SECOND_MOTION_COMMAND_TIMEOUT_S = 0.20
SECOND_MOTION_CONTROL_RATE_HZ = 20.0
STAGED_NAV2_PROFILE = "workstation-staged-nav2-corrected-v1"
STAGED_NAV2_STAGE = "stage1"
STAGED_NAV2_COMMAND_TOPIC = "/go2/staged_nav2/cmd_vel"
STAGED_NAV2_NAV_COMMAND_TOPIC = "/cmd_vel_guard_input"
STAGED_NAV2_ODOM_TOPIC = "/odom"
STAGED_NAV2_MAX_VX_MPS = 0.30
STAGED_NAV2_MAX_VY_MPS = 0.0
STAGED_NAV2_MAX_WZ_RPS = 0.40
STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2 = 0.30
STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2 = 0.80
# Standard Nav2 has no artificial whole-task duration or trip-distance cap.
# Velocity/acceleration limits and the independent command watchdog remain.
STAGED_NAV2_MAX_DURATION_S = 0.0
STAGED_NAV2_MAX_DISTANCE_M = 0.0
STAGED_NAV2_COMMAND_TIMEOUT_S = 0.20
STAGED_NAV2_STATE_TIMEOUT_S = 1.0
STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S = 1.0
CLASSIC_MOTION_STATE_MARKERS = frozenset({100, 2010})
COMMISSIONING_STATE_TIMEOUT_S = 0.20
ODOM_SOURCE_SPORT_STATE = "sport_state"
ODOM_SOURCE_EXTERNAL_VERIFIED = "external_verified"
DEFAULT_EXTERNAL_ODOM_TOPIC = (
    "/robonix/time_corrected/raw/utlidar/robot_odom"
)
_EXTERNAL_ODOM_TOPIC_PREFIX = "/robonix/time_corrected/"
_TOPIC_RE = re.compile(r"^/[A-Za-z0-9_/]+$")
_FRAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_/]*$")
_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")

_KNOWN_KEYS = {
    "state_topic",
    "state_fallback_topic",
    "twist_in_topic",
    "odom_topic",
    "odom_source",
    "external_odom_topic",
    "external_odom_timeout_s",
    "max_external_odom_yaw_jump_rad",
    "publish_odom_tf",
    "stationary_pose_hold_enabled",
    "stationary_hold_dwell_s",
    "stationary_hold_sport_max_linear_mps",
    "stationary_hold_sport_max_yaw_rps",
    "stationary_hold_external_twist_max_linear_mps",
    "stationary_hold_external_twist_max_yaw_rps",
    "stationary_hold_pose_max_linear_rate_mps",
    "stationary_hold_pose_max_yaw_rate_rps",
    "imu_topic",
    "status_topic",
    "diagnostics_topic",
    "arm_service",
    "odom_frame",
    "base_frame",
    "imu_frame",
    "velocity_frame",
    "network_interface",
    "ipc_socket",
    "allow_motion",
    "motion_profile",
    "preserve_classic_walk",
    "operator_present",
    "safety_ack",
    "allowed_modes",
    "allowed_state_markers",
    "allow_passive_state_marker_transitions",
    "allow_motion_state_marker_transitions",
    "max_linear_x_mps",
    "max_linear_y_mps",
    "max_angular_z_rps",
    "max_linear_accel_mps2",
    "max_angular_accel_rps2",
    "command_timeout_s",
    "state_timeout_s",
    "max_source_stamp_age_s",
    "max_source_stamp_future_skew_s",
    "zero_preamble_s",
    "control_rate_hz",
    "max_position_jump_m",
    "startup_timeout_s",
    "commissioning_max_duration_s",
    "commissioning_max_distance_m",
}


class ConfigError(ValueError):
    """The delivered config is unsafe, ambiguous, or unsupported."""


def prepare_private_directory(path: Path) -> None:
    """Create or validate an owned, real directory and set it to mode 0700.

    ``Path.chmod`` follows symbolic links. The runtime socket parent is a
    security boundary, so validate with ``lstat`` before opening it with
    ``O_NOFOLLOW``, compare the opened inode, and chmod only that descriptor.
    """
    directory = Path(path)
    try:
        path_stat = os.lstat(directory)
    except FileNotFoundError:
        try:
            directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        except FileExistsError:
            # Another actor may have installed an entry between lstat and
            # mkdir. The mandatory lstat below decides whether it is safe.
            pass
        path_stat = os.lstat(directory)

    current_uid = os.geteuid()
    if stat.S_ISLNK(path_stat.st_mode):
        raise ConfigError(f"private directory must not be a symlink: {directory}")
    if not stat.S_ISDIR(path_stat.st_mode):
        raise ConfigError(f"private directory is not a directory: {directory}")
    if path_stat.st_uid != current_uid:
        raise ConfigError(
            f"private directory is not owned by the current UID: {directory}"
        )

    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
    descriptor = os.open(directory, flags)
    try:
        opened_stat = os.fstat(descriptor)
        if (
            not stat.S_ISDIR(opened_stat.st_mode)
            or opened_stat.st_uid != current_uid
            or opened_stat.st_dev != path_stat.st_dev
            or opened_stat.st_ino != path_stat.st_ino
        ):
            raise ConfigError(
                f"private directory changed during validation: {directory}"
            )
        os.fchmod(descriptor, 0o700)
        secured_stat = os.fstat(descriptor)
        if stat.S_IMODE(secured_stat.st_mode) != 0o700:
            raise ConfigError(
                f"private directory mode could not be set to 0700: {directory}"
            )
    finally:
        os.close(descriptor)

    final_stat = os.lstat(directory)
    if (
        not stat.S_ISDIR(final_stat.st_mode)
        or final_stat.st_uid != current_uid
        or final_stat.st_dev != path_stat.st_dev
        or final_stat.st_ino != path_stat.st_ino
        or stat.S_IMODE(final_stat.st_mode) != 0o700
    ):
        raise ConfigError(
            f"private directory changed after it was secured: {directory}"
        )


def _boolean(value, name: str, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    raise ConfigError(f"{name} must be an explicit boolean")


def _text(value, name: str, default: str = "", *, allow_empty: bool = False) -> str:
    if value is None:
        value = default
    if not isinstance(value, str):
        raise ConfigError(f"{name} must be a string")
    result = value.strip()
    if not result and not allow_empty:
        raise ConfigError(f"{name} must not be empty")
    return result


def _topic(value, name: str, default: str, *, allow_empty: bool = False) -> str:
    result = _text(value, name, default, allow_empty=allow_empty)
    if result and (_TOPIC_RE.fullmatch(result) is None or "//" in result):
        raise ConfigError(f"{name} is not a canonical absolute ROS topic")
    return result


def _frame(value, name: str, default: str) -> str:
    result = _text(value, name, default)
    if _FRAME_RE.fullmatch(result) is None or "//" in result:
        raise ConfigError(f"{name} is not a valid frame id")
    return result


def _number(
    value,
    name: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
    include_minimum: bool = True,
) -> float:
    if value is None:
        value = default
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number")
    result = float(value)
    minimum_ok = result >= minimum if include_minimum else result > minimum
    if not math.isfinite(result) or not minimum_ok or result > maximum:
        boundary = "at least" if include_minimum else "greater than"
        raise ConfigError(
            f"{name} must be finite, {boundary} {minimum}, and at most {maximum}"
        )
    return result


def _mode_list(value, name: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)) or not value:
        raise ConfigError(f"{name} must be a non-empty list")
    modes: list[int] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int) or not 0 <= item <= 255:
            raise ConfigError(f"{name} entries must be uint8 integers")
        if item not in modes:
            modes.append(item)
    return tuple(modes)


def _state_marker_list(value, name: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)):
        raise ConfigError(f"{name} must be a list")
    markers: list[int] = []
    for item in value:
        if (
            isinstance(item, bool)
            or not isinstance(item, int)
            or not 1 <= item <= 0xFFFFFFFF
        ):
            raise ConfigError(f"{name} entries must be non-zero uint32 integers")
        if item not in markers:
            markers.append(item)
    return tuple(markers)


def parse_audited_modes(environ: Mapping[str, str]) -> tuple[int, ...]:
    """Parse the operator-supplied modes observed during read-only auditing."""
    raw = environ.get("GO2_ALLOWED_MODES", "").strip()
    if not raw:
        raise ConfigError(
            "motion requires explicit GO2_ALLOWED_MODES from the read-only audit"
        )
    tokens = [token.strip() for token in raw.replace(";", ",").split(",")]
    if any(not token for token in tokens):
        raise ConfigError("GO2_ALLOWED_MODES contains an empty entry")
    modes: list[int] = []
    for token in tokens:
        if not token.isdecimal():
            raise ConfigError("GO2_ALLOWED_MODES entries must be decimal uint8 values")
        mode = int(token, 10)
        if not 0 <= mode < UNKNOWN_MODE:
            raise ConfigError("GO2_ALLOWED_MODES entries must be between 0 and 254")
        if mode not in modes:
            modes.append(mode)
    return tuple(modes)


def parse_allowed_state_markers(environ: Mapping[str, str]) -> tuple[int, ...]:
    """Parse explicitly allowed opaque, non-zero firmware state markers.

    ``SportModeState.error_code`` and ``SportModeState.mode`` are separate
    fields in Unitree's public message definition.  These values therefore
    remain opaque compatibility markers; this parser deliberately assigns no
    mode or health meaning to them.  Zero is intrinsically accepted by the
    adapter and must not be repeated in this exceptional allowlist.
    """
    raw = environ.get("GO2_ALLOWED_STATE_MARKERS", "").strip()
    if not raw:
        return ()
    tokens = [token.strip() for token in raw.replace(";", ",").split(",")]
    if any(not token for token in tokens):
        raise ConfigError("GO2_ALLOWED_STATE_MARKERS contains an empty entry")
    markers: list[int] = []
    for token in tokens:
        if not token.isdecimal():
            raise ConfigError(
                "GO2_ALLOWED_STATE_MARKERS entries must be decimal uint32 values"
            )
        marker = int(token, 10)
        if not 1 <= marker <= 0xFFFFFFFF:
            raise ConfigError(
                "GO2_ALLOWED_STATE_MARKERS entries must be between 1 and 4294967295"
            )
        if marker not in markers:
            markers.append(marker)
    return tuple(markers)


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    state_topic: str
    state_fallback_topic: str
    twist_in_topic: str
    odom_topic: str
    odom_source: str
    external_odom_topic: str
    external_odom_timeout_s: float
    max_external_odom_yaw_jump_rad: float
    publish_odom_tf: bool
    stationary_pose_hold_enabled: bool
    stationary_hold_dwell_s: float
    stationary_hold_sport_max_linear_mps: float
    stationary_hold_sport_max_yaw_rps: float
    stationary_hold_external_twist_max_linear_mps: float
    stationary_hold_external_twist_max_yaw_rps: float
    stationary_hold_pose_max_linear_rate_mps: float
    stationary_hold_pose_max_yaw_rate_rps: float
    imu_topic: str
    status_topic: str
    diagnostics_topic: str
    arm_service: str
    odom_frame: str
    base_frame: str
    imu_frame: str
    velocity_frame: str
    network_interface: str
    ipc_socket: Path
    allow_motion: bool
    motion_profile: str
    preserve_classic_walk: bool
    allowed_modes: tuple[int, ...]
    allowed_state_markers: tuple[int, ...]
    allow_passive_state_marker_transitions: bool
    allow_motion_state_marker_transitions: bool
    max_linear_x_mps: float
    max_linear_y_mps: float
    max_angular_z_rps: float
    max_linear_accel_mps2: float
    max_angular_accel_rps2: float
    command_timeout_s: float
    state_timeout_s: float
    max_source_stamp_age_s: float
    max_source_stamp_future_skew_s: float
    zero_preamble_s: float
    control_rate_hz: float
    max_position_jump_m: float
    startup_timeout_s: float
    commissioning_max_duration_s: float
    commissioning_max_distance_m: float

    @property
    def starts_sdk_daemon(self) -> bool:
        return self.allow_motion

    def process_env(self) -> dict[str, str]:
        return {
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "GO2_ALLOW_MOTION": "1" if self.allow_motion else "0",
            "GO2_MOTION_PROFILE": self.motion_profile,
            "GO2_NETWORK_INTERFACE": self.network_interface,
            "GO2_SDK_SOCKET": str(self.ipc_socket),
            "GO2_ALLOWED_MODES": ",".join(str(mode) for mode in self.allowed_modes),
            "GO2_ALLOWED_STATE_MARKERS": ",".join(
                str(marker) for marker in self.allowed_state_markers
            ),
        }

    def sdk_daemon_env(self, executable: Path) -> dict[str, str]:
        """Keep Unitree's bundled CycloneDDS ahead of every ROS overlay.

        Robonix is launched from a sourced ROS 2 environment.  Passing that
        inherited ``LD_LIBRARY_PATH`` to the SDK2-only daemon can combine the
        system ``libddsc`` with Unitree's bundled ``libddscxx``, which is an
        ABI-invalid process image.  The SDK daemon has no ROS dependency, so
        its dynamic-library search path is deliberately reduced to its
        adjacent private library directory.
        """
        environment = self.process_env()
        environment["LD_LIBRARY_PATH"] = str(
            executable.resolve().parent.parent / "lib"
        )
        return environment

    def adapter_argv(self, executable: Path, params_file: Path) -> list[str]:
        modes = "[" + ",".join(str(mode) for mode in self.allowed_modes) + "]"
        markers = (
            "["
            + ",".join(str(marker) for marker in self.allowed_state_markers)
            + "]"
        )
        parameters = {
            "allow_motion": "true" if self.allow_motion else "false",
            "sport_state_topic": self.state_topic,
            "state_fallback_topic": self.state_fallback_topic,
            "cmd_vel_topic": self.twist_in_topic,
            "odom_topic": self.odom_topic,
            "odom_source": self.odom_source,
            "external_odom_topic": self.external_odom_topic,
            "external_odom_timeout_sec": str(self.external_odom_timeout_s),
            "max_external_odom_yaw_jump_rad": str(
                self.max_external_odom_yaw_jump_rad
            ),
            "publish_odom_tf": "true" if self.publish_odom_tf else "false",
            "imu_topic": self.imu_topic,
            "status_topic": self.status_topic,
            "diagnostics_topic": self.diagnostics_topic,
            "arm_service": self.arm_service,
            "odom_frame": self.odom_frame,
            "base_frame": self.base_frame,
            "imu_frame": self.imu_frame,
            "state_velocity_frame": self.velocity_frame,
            "sdk_socket": str(self.ipc_socket),
            "preserve_classic_walk": (
                "true" if self.preserve_classic_walk else "false"
            ),
            "allowed_modes": modes,
            "allow_passive_state_marker_transitions": (
                "true"
                if self.allow_passive_state_marker_transitions
                else "false"
            ),
            "allow_motion_state_marker_transitions": (
                "true" if self.allow_motion_state_marker_transitions else "false"
            ),
            "state_timeout_sec": str(self.state_timeout_s),
            "max_source_stamp_age_sec": str(self.max_source_stamp_age_s),
            "max_source_stamp_future_skew_sec": str(
                self.max_source_stamp_future_skew_s
            ),
            "command_timeout_sec": str(self.command_timeout_s),
            "zero_preparation_sec": str(self.zero_preamble_s),
            "control_rate_hz": str(self.control_rate_hz),
            "max_vx": str(self.max_linear_x_mps),
            "max_vy": str(self.max_linear_y_mps),
            "max_wz": str(self.max_angular_z_rps),
            "max_linear_acceleration": str(self.max_linear_accel_mps2),
            "max_angular_acceleration": str(self.max_angular_accel_rps2),
            "max_position_jump_m": str(self.max_position_jump_m),
            "commissioning_max_duration_sec": str(
                self.commissioning_max_duration_s
            ),
            "commissioning_max_distance_m": str(
                self.commissioning_max_distance_m
            ),
        }
        # Keep the already-audited first-motion argv unchanged. The adapter's
        # missing-parameter default is that legacy profile; every later,
        # independently reviewed profile must add its exact selector.
        if self.motion_profile != FIRST_MOTION_PROFILE:
            parameters["motion_profile"] = self.motion_profile
        if self.stationary_pose_hold_enabled:
            parameters.update(
                {
                    "stationary_pose_hold_enabled": "true",
                    "stationary_hold_dwell_sec": str(
                        self.stationary_hold_dwell_s
                    ),
                    "stationary_hold_sport_max_linear_mps": str(
                        self.stationary_hold_sport_max_linear_mps
                    ),
                    "stationary_hold_sport_max_yaw_rps": str(
                        self.stationary_hold_sport_max_yaw_rps
                    ),
                    "stationary_hold_external_twist_max_linear_mps": str(
                        self.stationary_hold_external_twist_max_linear_mps
                    ),
                    "stationary_hold_external_twist_max_yaw_rps": str(
                        self.stationary_hold_external_twist_max_yaw_rps
                    ),
                    "stationary_hold_pose_max_linear_rate_mps": str(
                        self.stationary_hold_pose_max_linear_rate_mps
                    ),
                    "stationary_hold_pose_max_yaw_rate_rps": str(
                        self.stationary_hold_pose_max_yaw_rate_rps
                    ),
                }
            )
        if self.allowed_state_markers:
            parameters["allowed_state_markers"] = markers
        argv = [str(executable), "--ros-args", "--params-file", str(params_file)]
        for name, value in parameters.items():
            # rcl's command-line parameter parser rejects a bare empty value
            # (`name:=`).  Quote an intentional empty YAML string explicitly;
            # argv is passed directly to exec, so these quotes are data for
            # the YAML parser rather than shell syntax.
            rendered_value = "''" if value == "" else value
            argv.extend(("-p", f"{name}:={rendered_value}"))
        return argv

    def daemon_argv(self, executable: Path) -> list[str]:
        if not self.allow_motion:
            raise ConfigError("SDK daemon must not start while motion is disabled")
        arguments = [
            str(executable),
            "--socket",
            str(self.ipc_socket),
            "--watchdog-ms",
            "300",
            "--max-vx",
            str(self.max_linear_x_mps),
            "--max-vy",
            str(self.max_linear_y_mps),
            "--max-wz",
            str(self.max_angular_z_rps),
            "--max-motion-ms",
            str(int(self.commissioning_max_duration_s * 1000.0)),
            "--allow-motion",
            "--interface",
            self.network_interface,
            "--motion-ack",
            "GO2_PHYSICAL_MOTION_APPROVED",
        ]
        # As above, omission continues to select the byte-for-byte legacy
        # first-motion CLI contract.
        if self.motion_profile != FIRST_MOTION_PROFILE:
            arguments.extend(("--motion-profile", self.motion_profile))
        if self.preserve_classic_walk:
            arguments.append("--preserve-classic-walk")
        return arguments


def normalize_config(
    config: Mapping[str, object] | None,
    environ: Mapping[str, str] | None,
    package_root: Path,
) -> RuntimeConfig:
    if config is None:
        config = {}
    if not isinstance(config, Mapping):
        raise ConfigError("Driver config must be a mapping")
    unknown = sorted(set(config) - _KNOWN_KEYS)
    if unknown:
        raise ConfigError(f"unknown config keys: {', '.join(unknown)}")
    env = os.environ if environ is None else environ

    allow_motion = _boolean(config.get("allow_motion"), "allow_motion", False)
    motion_profile = _text(
        config.get("motion_profile"),
        "motion_profile",
        "",
        allow_empty=True,
    )
    preserve_classic_walk = _boolean(
        config.get("preserve_classic_walk"),
        "preserve_classic_walk",
        False,
    )
    operator_present = _boolean(
        config.get("operator_present"), "operator_present", False
    )
    safety_ack = _text(
        config.get("safety_ack"), "safety_ack", "", allow_empty=True
    )
    configured_modes = _mode_list(
        config.get("allowed_modes", [UNKNOWN_MODE]), "allowed_modes"
    )
    if configured_modes != (UNKNOWN_MODE,):
        raise ConfigError(
            "Driver allowed_modes must remain [255]; use GO2_ALLOWED_MODES "
            "only after a read-only audit"
        )

    network_interface = _text(
        config.get("network_interface"),
        "network_interface",
        "",
        allow_empty=not allow_motion,
    )
    if network_interface and _INTERFACE_RE.fullmatch(network_interface) is None:
        raise ConfigError("network_interface contains unsupported characters")

    if allow_motion:
        if not operator_present:
            raise ConfigError("motion requires operator_present=true")
        if safety_ack != SAFETY_ACK:
            raise ConfigError("motion requires the exact safety_ack phrase")
        if not network_interface:
            raise ConfigError("motion requires a dedicated network_interface")
        if motion_profile not in {
            FIRST_MOTION_PROFILE,
            SECOND_MOTION_PROFILE,
            STAGED_NAV2_PROFILE,
        }:
            raise ConfigError(
                "motion requires an independently audited profile: "
                f"{FIRST_MOTION_PROFILE}, {SECOND_MOTION_PROFILE}, or "
                f"{STAGED_NAV2_PROFILE}"
            )
        allowed_modes = parse_audited_modes(env)
    else:
        # A stale manifest list can never authorize motion. Until an operator
        # enables all gates and supplies audited modes, only the impossible
        # uint8 sentinel is injected into the ROS guard.
        allowed_modes = (UNKNOWN_MODE,)
    if preserve_classic_walk and (
        not allow_motion or motion_profile != STAGED_NAV2_PROFILE
    ):
        raise ConfigError(
            "preserve_classic_walk requires the staged Nav2 motion profile"
        )

    environment_markers = parse_allowed_state_markers(env)
    configured_marker_value = config.get("allowed_state_markers")
    if configured_marker_value is None:
        allowed_state_markers = environment_markers
    else:
        configured_markers = _state_marker_list(
            configured_marker_value, "allowed_state_markers"
        )
        if allow_motion and configured_markers:
            raise ConfigError(
                "motion state markers must come from the audited runtime permit, "
                "not Driver config"
            )
        if environment_markers and environment_markers != configured_markers:
            raise ConfigError(
                "allowed_state_markers disagrees with GO2_ALLOWED_STATE_MARKERS"
            )
        allowed_state_markers = configured_markers
    allow_passive_state_marker_transitions = _boolean(
        config.get("allow_passive_state_marker_transitions"),
        "allow_passive_state_marker_transitions",
        False,
    )
    allow_motion_state_marker_transitions = _boolean(
        config.get("allow_motion_state_marker_transitions"),
        "allow_motion_state_marker_transitions",
        False,
    )

    package_hash = hashlib.sha256(str(package_root.resolve()).encode()).hexdigest()[:12]
    socket_default = (
        Path("/tmp")
        / f"robonix-go2-{os.getuid()}-{package_hash}"
        / "sport.sock"
    )
    socket_text = _text(
        config.get("ipc_socket"), "ipc_socket", str(socket_default)
    )
    ipc_socket = Path(socket_text)
    if not ipc_socket.is_absolute() or len(str(ipc_socket)) >= 108:
        raise ConfigError("ipc_socket must be an absolute Unix socket path under 108 bytes")

    state_topic = _topic(
        config.get("state_topic"), "state_topic", "/sportmodestate"
    )
    state_fallback_topic = _topic(
        config.get("state_fallback_topic"),
        "state_fallback_topic",
        "/lf/sportmodestate",
        allow_empty=True,
    )
    if state_fallback_topic == state_topic:
        state_fallback_topic = ""

    velocity_frame = _text(
        config.get("velocity_frame"), "velocity_frame", "odom"
    )
    if velocity_frame not in {"odom", "base_link"}:
        raise ConfigError("velocity_frame must be 'odom' or 'base_link'")

    twist_in_topic = _topic(
        config.get("twist_in_topic"), "twist_in_topic", "/cmd_vel"
    )
    odom_topic = _topic(config.get("odom_topic"), "odom_topic", "/odom")
    odom_source = _text(
        config.get("odom_source"),
        "odom_source",
        ODOM_SOURCE_SPORT_STATE,
    )
    if odom_source not in {
        ODOM_SOURCE_SPORT_STATE,
        ODOM_SOURCE_EXTERNAL_VERIFIED,
    }:
        raise ConfigError(
            "odom_source must be 'sport_state' or 'external_verified'"
        )
    external_odom_topic = _topic(
        config.get("external_odom_topic"),
        "external_odom_topic",
        DEFAULT_EXTERNAL_ODOM_TOPIC,
    )
    external_odom_timeout_s = _number(
        config.get("external_odom_timeout_s"),
        "external_odom_timeout_s",
        (
            STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S
            if allow_motion and motion_profile == STAGED_NAV2_PROFILE
            else COMMISSIONING_STATE_TIMEOUT_S
        ),
        minimum=0.05,
        maximum=(
            STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S
            if allow_motion and motion_profile == STAGED_NAV2_PROFILE
            else COMMISSIONING_STATE_TIMEOUT_S
        ),
    )
    max_external_odom_yaw_jump_rad = _number(
        config.get("max_external_odom_yaw_jump_rad"),
        "max_external_odom_yaw_jump_rad",
        1.0,
        minimum=0.10,
        maximum=1.0,
    )
    publish_odom_tf = _boolean(
        config.get("publish_odom_tf"), "publish_odom_tf", True
    )
    stationary_pose_hold_enabled = _boolean(
        config.get("stationary_pose_hold_enabled"),
        "stationary_pose_hold_enabled",
        False,
    )
    stationary_hold_dwell_s = _number(
        config.get("stationary_hold_dwell_s"),
        "stationary_hold_dwell_s",
        2.0,
        minimum=1.0,
        maximum=10.0,
    )
    stationary_hold_sport_max_linear_mps = _number(
        config.get("stationary_hold_sport_max_linear_mps"),
        "stationary_hold_sport_max_linear_mps",
        0.03,
        minimum=0.0,
        maximum=0.03,
        include_minimum=False,
    )
    stationary_hold_sport_max_yaw_rps = _number(
        config.get("stationary_hold_sport_max_yaw_rps"),
        "stationary_hold_sport_max_yaw_rps",
        0.03,
        minimum=0.0,
        maximum=0.03,
        include_minimum=False,
    )
    stationary_hold_external_twist_max_linear_mps = _number(
        config.get("stationary_hold_external_twist_max_linear_mps"),
        "stationary_hold_external_twist_max_linear_mps",
        0.03,
        minimum=0.0,
        maximum=0.03,
        include_minimum=False,
    )
    stationary_hold_external_twist_max_yaw_rps = _number(
        config.get("stationary_hold_external_twist_max_yaw_rps"),
        "stationary_hold_external_twist_max_yaw_rps",
        0.03,
        minimum=0.0,
        maximum=0.03,
        include_minimum=False,
    )
    stationary_hold_pose_max_linear_rate_mps = _number(
        config.get("stationary_hold_pose_max_linear_rate_mps"),
        "stationary_hold_pose_max_linear_rate_mps",
        0.005,
        minimum=0.0,
        maximum=0.005,
        include_minimum=False,
    )
    stationary_hold_pose_max_yaw_rate_rps = _number(
        config.get("stationary_hold_pose_max_yaw_rate_rps"),
        "stationary_hold_pose_max_yaw_rate_rps",
        0.01,
        minimum=0.0,
        maximum=0.01,
        include_minimum=False,
    )
    if odom_source == ODOM_SOURCE_EXTERNAL_VERIFIED:
        if external_odom_topic == odom_topic:
            raise ConfigError(
                "external_odom_topic must differ from canonical odom_topic"
            )
        if not external_odom_topic.startswith(_EXTERNAL_ODOM_TOPIC_PREFIX):
            raise ConfigError(
                "external_verified odom must use a private "
                f"{_EXTERNAL_ODOM_TOPIC_PREFIX} topic"
            )
        if odom_topic != FIRST_MOTION_ODOM_TOPIC or not publish_odom_tf:
            raise ConfigError(
                "external_verified mode requires chassis-owned /odom and "
                "odom -> base_link TF"
            )
    if stationary_pose_hold_enabled and (
        allow_motion
        or odom_source != ODOM_SOURCE_EXTERNAL_VERIFIED
        or not publish_odom_tf
    ):
        raise ConfigError(
            "stationary_pose_hold_enabled requires allow_motion=false, "
            "odom_source=external_verified, and publish_odom_tf=true"
        )
    if allow_passive_state_marker_transitions:
        if allow_motion or odom_source != ODOM_SOURCE_EXTERNAL_VERIFIED:
            raise ConfigError(
                "allow_passive_state_marker_transitions requires "
                "allow_motion=false and odom_source=external_verified"
            )
        if len(allowed_state_markers) < 2:
            raise ConfigError(
                "allow_passive_state_marker_transitions requires at least "
                "two explicit allowed_state_markers"
            )
    if allow_motion_state_marker_transitions:
        marker_set = frozenset(allowed_state_markers)
        if (
            not allow_motion
            or motion_profile != STAGED_NAV2_PROFILE
            or odom_source != ODOM_SOURCE_EXTERNAL_VERIFIED
            or allow_passive_state_marker_transitions
            or len(allowed_modes) != 1
            or marker_set != CLASSIC_MOTION_STATE_MARKERS
        ):
            raise ConfigError(
                "allow_motion_state_marker_transitions requires staged Nav2, "
                "external_verified odometry, one audited mode, and the exact "
                "Classic marker allowlist {100,2010}"
            )
    max_linear_x_ceiling = (
        STAGED_NAV2_MAX_VX_MPS
        if allow_motion and motion_profile == STAGED_NAV2_PROFILE
        else (
            SECOND_MOTION_MAX_VX_MPS
            if allow_motion and motion_profile == SECOND_MOTION_PROFILE
            else 0.25
        )
    )
    max_linear_x_mps = _number(
        config.get("max_linear_x_mps"),
        "max_linear_x_mps",
        0.25,
        minimum=0.0,
        maximum=max_linear_x_ceiling,
        include_minimum=False,
    )
    max_linear_y_mps = _number(
        config.get("max_linear_y_mps"),
        "max_linear_y_mps",
        0.0,
        minimum=0.0,
        maximum=0.0,
    )
    max_angular_z_rps = _number(
        config.get("max_angular_z_rps"),
        "max_angular_z_rps",
        0.40,
        minimum=0.0,
        maximum=0.40,
    )
    command_timeout_s = _number(
        config.get("command_timeout_s"),
        "command_timeout_s",
        0.25,
        minimum=0.05,
        maximum=0.25,
    )
    commissioning_max_duration_s = _number(
        config.get("commissioning_max_duration_s"),
        "commissioning_max_duration_s",
        FIRST_MOTION_MAX_DURATION_S,
        minimum=0.0,
        maximum=20.0,
        include_minimum=True,
    )
    commissioning_max_distance_m = _number(
        config.get("commissioning_max_distance_m"),
        "commissioning_max_distance_m",
        FIRST_MOTION_MAX_DISTANCE_M,
        minimum=0.0,
        maximum=0.50,
        include_minimum=True,
    )
    if allow_motion:
        if motion_profile == FIRST_MOTION_PROFILE:
            profile_label = "first-motion"
            required_envelope = {
                "twist_in_topic": (twist_in_topic, FIRST_MOTION_COMMAND_TOPIC),
                "odom_topic": (odom_topic, FIRST_MOTION_ODOM_TOPIC),
                "publish_odom_tf": (publish_odom_tf, True),
                "max_linear_x_mps": (
                    max_linear_x_mps,
                    FIRST_MOTION_MAX_VX_MPS,
                ),
                "max_linear_y_mps": (
                    max_linear_y_mps,
                    FIRST_MOTION_MAX_VY_MPS,
                ),
                "max_angular_z_rps": (
                    max_angular_z_rps,
                    FIRST_MOTION_MAX_WZ_RPS,
                ),
                "commissioning_max_duration_s": (
                    commissioning_max_duration_s,
                    FIRST_MOTION_MAX_DURATION_S,
                ),
                "commissioning_max_distance_m": (
                    commissioning_max_distance_m,
                    FIRST_MOTION_MAX_DISTANCE_M,
                ),
                "command_timeout_s": (
                    command_timeout_s,
                    FIRST_MOTION_COMMAND_TIMEOUT_S,
                ),
                "state_timeout_s": (
                    _number(
                        config.get("state_timeout_s"),
                        "state_timeout_s",
                        COMMISSIONING_STATE_TIMEOUT_S,
                        minimum=0.05,
                        maximum=STAGED_NAV2_STATE_TIMEOUT_S,
                    ),
                    COMMISSIONING_STATE_TIMEOUT_S,
                ),
                "max_source_stamp_age_s": (
                    _number(
                        config.get("max_source_stamp_age_s"),
                        "max_source_stamp_age_s",
                        0.20,
                        minimum=0.05,
                        maximum=0.20,
                    ),
                    0.20,
                ),
            }
        elif motion_profile == SECOND_MOTION_PROFILE:
            profile_label = "second-motion"
            required_envelope = {
                "twist_in_topic": (
                    twist_in_topic,
                    SECOND_MOTION_COMMAND_TOPIC,
                ),
                "odom_topic": (odom_topic, SECOND_MOTION_ODOM_TOPIC),
                "publish_odom_tf": (publish_odom_tf, True),
                "odom_source": (
                    odom_source,
                    ODOM_SOURCE_EXTERNAL_VERIFIED,
                ),
                "external_odom_topic": (
                    external_odom_topic,
                    DEFAULT_EXTERNAL_ODOM_TOPIC,
                ),
                "external_odom_timeout_s": (
                    external_odom_timeout_s,
                    COMMISSIONING_STATE_TIMEOUT_S,
                ),
                "max_linear_x_mps": (
                    max_linear_x_mps,
                    SECOND_MOTION_MAX_VX_MPS,
                ),
                "max_linear_y_mps": (
                    max_linear_y_mps,
                    SECOND_MOTION_MAX_VY_MPS,
                ),
                "max_angular_z_rps": (
                    max_angular_z_rps,
                    SECOND_MOTION_MAX_WZ_RPS,
                ),
                "max_linear_accel_mps2": (
                    _number(
                        config.get("max_linear_accel_mps2"),
                        "max_linear_accel_mps2",
                        0.30,
                        minimum=0.0,
                        maximum=0.30,
                        include_minimum=False,
                    ),
                    SECOND_MOTION_MAX_LINEAR_ACCEL_MPS2,
                ),
                "max_angular_accel_rps2": (
                    _number(
                        config.get("max_angular_accel_rps2"),
                        "max_angular_accel_rps2",
                        0.80,
                        minimum=0.0,
                        maximum=0.80,
                        include_minimum=False,
                    ),
                    SECOND_MOTION_MAX_ANGULAR_ACCEL_RPS2,
                ),
                "commissioning_max_duration_s": (
                    commissioning_max_duration_s,
                    SECOND_MOTION_MAX_DURATION_S,
                ),
                "commissioning_max_distance_m": (
                    commissioning_max_distance_m,
                    SECOND_MOTION_MAX_DISTANCE_M,
                ),
                "command_timeout_s": (
                    command_timeout_s,
                    SECOND_MOTION_COMMAND_TIMEOUT_S,
                ),
                "control_rate_hz": (
                    _number(
                        config.get("control_rate_hz"),
                        "control_rate_hz",
                        50.0,
                        minimum=10.0,
                        maximum=100.0,
                    ),
                    SECOND_MOTION_CONTROL_RATE_HZ,
                ),
                "state_timeout_s": (
                    _number(
                        config.get("state_timeout_s"),
                        "state_timeout_s",
                        COMMISSIONING_STATE_TIMEOUT_S,
                        minimum=0.05,
                        maximum=STAGED_NAV2_STATE_TIMEOUT_S,
                    ),
                    COMMISSIONING_STATE_TIMEOUT_S,
                ),
                "max_source_stamp_age_s": (
                    _number(
                        config.get("max_source_stamp_age_s"),
                        "max_source_stamp_age_s",
                        0.20,
                        minimum=0.05,
                        maximum=0.20,
                    ),
                    0.20,
                ),
                "max_source_stamp_future_skew_s": (
                    _number(
                        config.get("max_source_stamp_future_skew_s"),
                        "max_source_stamp_future_skew_s",
                        0.05,
                        minimum=0.0,
                        maximum=0.05,
                    ),
                    0.05,
                ),
            }
        else:
            profile_label = "staged-nav2 stage1"
            required_envelope = {
                "twist_in_topic": (twist_in_topic, STAGED_NAV2_COMMAND_TOPIC),
                "odom_topic": (odom_topic, STAGED_NAV2_ODOM_TOPIC),
                "publish_odom_tf": (publish_odom_tf, True),
                "odom_source": (
                    odom_source,
                    ODOM_SOURCE_EXTERNAL_VERIFIED,
                ),
                "external_odom_topic": (
                    external_odom_topic,
                    DEFAULT_EXTERNAL_ODOM_TOPIC,
                ),
                "external_odom_timeout_s": (
                    external_odom_timeout_s,
                    STAGED_NAV2_EXTERNAL_ODOM_TIMEOUT_S,
                ),
                "max_linear_x_mps": (
                    max_linear_x_mps,
                    STAGED_NAV2_MAX_VX_MPS,
                ),
                "max_linear_y_mps": (
                    max_linear_y_mps,
                    STAGED_NAV2_MAX_VY_MPS,
                ),
                "max_angular_z_rps": (
                    max_angular_z_rps,
                    STAGED_NAV2_MAX_WZ_RPS,
                ),
                "max_linear_accel_mps2": (
                    _number(
                        config.get("max_linear_accel_mps2"),
                        "max_linear_accel_mps2",
                        0.30,
                        minimum=0.0,
                        maximum=0.30,
                        include_minimum=False,
                    ),
                    STAGED_NAV2_MAX_LINEAR_ACCEL_MPS2,
                ),
                "max_angular_accel_rps2": (
                    _number(
                        config.get("max_angular_accel_rps2"),
                        "max_angular_accel_rps2",
                        0.80,
                        minimum=0.0,
                        maximum=0.80,
                        include_minimum=False,
                    ),
                    STAGED_NAV2_MAX_ANGULAR_ACCEL_RPS2,
                ),
                "commissioning_max_duration_s": (
                    commissioning_max_duration_s,
                    STAGED_NAV2_MAX_DURATION_S,
                ),
                "commissioning_max_distance_m": (
                    commissioning_max_distance_m,
                    STAGED_NAV2_MAX_DISTANCE_M,
                ),
                "command_timeout_s": (
                    command_timeout_s,
                    STAGED_NAV2_COMMAND_TIMEOUT_S,
                ),
                "state_timeout_s": (
                    _number(
                        config.get("state_timeout_s"),
                        "state_timeout_s",
                        STAGED_NAV2_STATE_TIMEOUT_S,
                        minimum=0.05,
                        maximum=STAGED_NAV2_STATE_TIMEOUT_S,
                    ),
                    STAGED_NAV2_STATE_TIMEOUT_S,
                ),
                "max_source_stamp_age_s": (
                    _number(
                        config.get("max_source_stamp_age_s"),
                        "max_source_stamp_age_s",
                        0.20,
                        minimum=0.05,
                        maximum=0.20,
                    ),
                    0.20,
                ),
                "max_source_stamp_future_skew_s": (
                    _number(
                        config.get("max_source_stamp_future_skew_s"),
                        "max_source_stamp_future_skew_s",
                        0.05,
                        minimum=0.0,
                        maximum=0.05,
                    ),
                    0.05,
                ),
            }
        mismatched = [
            name
            for name, (actual, expected) in required_envelope.items()
            if actual != expected
        ]
        if mismatched:
            raise ConfigError(
                f"{profile_label} profile requires the exact motion "
                "envelope; mismatched keys: " + ", ".join(mismatched)
            )

    return RuntimeConfig(
        state_topic=state_topic,
        state_fallback_topic=state_fallback_topic,
        twist_in_topic=twist_in_topic,
        odom_topic=odom_topic,
        odom_source=odom_source,
        external_odom_topic=external_odom_topic,
        external_odom_timeout_s=external_odom_timeout_s,
        max_external_odom_yaw_jump_rad=max_external_odom_yaw_jump_rad,
        publish_odom_tf=publish_odom_tf,
        stationary_pose_hold_enabled=stationary_pose_hold_enabled,
        stationary_hold_dwell_s=stationary_hold_dwell_s,
        stationary_hold_sport_max_linear_mps=(
            stationary_hold_sport_max_linear_mps
        ),
        stationary_hold_sport_max_yaw_rps=(
            stationary_hold_sport_max_yaw_rps
        ),
        stationary_hold_external_twist_max_linear_mps=(
            stationary_hold_external_twist_max_linear_mps
        ),
        stationary_hold_external_twist_max_yaw_rps=(
            stationary_hold_external_twist_max_yaw_rps
        ),
        stationary_hold_pose_max_linear_rate_mps=(
            stationary_hold_pose_max_linear_rate_mps
        ),
        stationary_hold_pose_max_yaw_rate_rps=(
            stationary_hold_pose_max_yaw_rate_rps
        ),
        imu_topic=_topic(config.get("imu_topic"), "imu_topic", "/imu/data"),
        status_topic=_topic(
            config.get("status_topic"), "status_topic", "/go2_chassis/status"
        ),
        diagnostics_topic=_topic(
            config.get("diagnostics_topic"), "diagnostics_topic", "/diagnostics"
        ),
        arm_service=_topic(
            config.get("arm_service"), "arm_service", "/go2_chassis/arm"
        ),
        odom_frame=_frame(config.get("odom_frame"), "odom_frame", "odom"),
        base_frame=_frame(
            config.get("base_frame"), "base_frame", "base_link"
        ),
        imu_frame=_frame(config.get("imu_frame"), "imu_frame", "imu"),
        velocity_frame=velocity_frame,
        network_interface=network_interface,
        ipc_socket=ipc_socket,
        allow_motion=allow_motion,
        motion_profile=motion_profile,
        preserve_classic_walk=preserve_classic_walk,
        allowed_modes=allowed_modes,
        allowed_state_markers=allowed_state_markers,
        allow_passive_state_marker_transitions=(
            allow_passive_state_marker_transitions
        ),
        allow_motion_state_marker_transitions=(
            allow_motion_state_marker_transitions
        ),
        max_linear_x_mps=max_linear_x_mps,
        max_linear_y_mps=max_linear_y_mps,
        max_angular_z_rps=max_angular_z_rps,
        max_linear_accel_mps2=_number(
            config.get("max_linear_accel_mps2"),
            "max_linear_accel_mps2",
            0.30,
            minimum=0.0,
            maximum=0.30,
            include_minimum=False,
        ),
        max_angular_accel_rps2=_number(
            config.get("max_angular_accel_rps2"),
            "max_angular_accel_rps2",
            0.80,
            minimum=0.0,
            maximum=0.80,
            include_minimum=False,
        ),
        command_timeout_s=command_timeout_s,
        state_timeout_s=_number(
            config.get("state_timeout_s"),
            "state_timeout_s",
            (
                STAGED_NAV2_STATE_TIMEOUT_S
                if allow_motion and motion_profile == STAGED_NAV2_PROFILE
                else COMMISSIONING_STATE_TIMEOUT_S
            ),
            minimum=0.05,
            maximum=(
                STAGED_NAV2_STATE_TIMEOUT_S
                if allow_motion and motion_profile == STAGED_NAV2_PROFILE
                else COMMISSIONING_STATE_TIMEOUT_S
            ),
        ),
        max_source_stamp_age_s=_number(
            config.get("max_source_stamp_age_s"),
            "max_source_stamp_age_s",
            0.20,
            minimum=0.05,
            maximum=0.20,
        ),
        max_source_stamp_future_skew_s=_number(
            config.get("max_source_stamp_future_skew_s"),
            "max_source_stamp_future_skew_s",
            0.05,
            minimum=0.0,
            maximum=0.05,
        ),
        zero_preamble_s=_number(
            config.get("zero_preamble_s"),
            "zero_preamble_s",
            0.50,
            minimum=0.50,
            maximum=5.0,
        ),
        control_rate_hz=_number(
            config.get("control_rate_hz"),
            "control_rate_hz",
            50.0,
            minimum=10.0,
            maximum=100.0,
        ),
        max_position_jump_m=_number(
            config.get("max_position_jump_m"),
            "max_position_jump_m",
            1.0,
            minimum=0.10,
            maximum=2.0,
        ),
        startup_timeout_s=_number(
            config.get("startup_timeout_s"),
            "startup_timeout_s",
            15.0,
            minimum=1.0,
            maximum=60.0,
        ),
        commissioning_max_duration_s=commissioning_max_duration_s,
        commissioning_max_distance_m=commissioning_max_distance_m,
    )
