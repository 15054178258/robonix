from __future__ import annotations

from pathlib import Path


class ConfigError(Exception):
    pass


def _resolve_env(key: str, env: dict[str, str], default: str = "") -> str:
    """Resolve an env variable, falling back to the provided default."""
    return env.get(key, env.get(f"G1_{key}", default))


def _resolve_path(key: str, env: dict[str, str]) -> Path:
    """Resolve a path env variable to an absolute Path."""
    raw = _resolve_env(key, env)
    if not raw:
        raise ConfigError(f"required env variable {key} is not set")
    return Path(raw)


class RuntimeConfig:
    """Parsed and validated runtime configuration for G1 chassis."""

    def __init__(
        self,
        network_interface: str,
        allow_motion: bool,
        ipc_socket: Path,
        twist_in_topic: str,
        odom_topic: str,
        odom_source: str,
        imu_topic: str,
        joint_state_topic: str,
        odom_frame: str,
        base_frame: str,
        imu_frame: str,
        velocity_frame: str,
        publish_odom_tf: bool,
        stand_height: float,
        swing_height: float,
        max_linear_x_mps: float,
        max_linear_y_mps: float,
        max_angular_z_rps: float,
        max_linear_accel_mps2: float,
        max_angular_accel_rps2: float,
        command_timeout_s: float,
        state_timeout_s: float,
        startup_timeout_s: float,
    ):
        self.network_interface = network_interface
        self.allow_motion = allow_motion
        self.ipc_socket = ipc_socket
        self.twist_in_topic = twist_in_topic
        self.odom_topic = odom_topic
        self.odom_source = odom_source
        self.imu_topic = imu_topic
        self.joint_state_topic = joint_state_topic
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.imu_frame = imu_frame
        self.velocity_frame = velocity_frame
        self.publish_odom_tf = publish_odom_tf
        self.stand_height = stand_height
        self.swing_height = swing_height
        self.max_linear_x_mps = max_linear_x_mps
        self.max_linear_y_mps = max_linear_y_mps
        self.max_angular_z_rps = max_angular_z_rps
        self.max_linear_accel_mps2 = max_linear_accel_mps2
        self.max_angular_accel_rps2 = max_angular_accel_rps2
        self.command_timeout_s = command_timeout_s
        self.state_timeout_s = state_timeout_s
        self.startup_timeout_s = startup_timeout_s

    @property
    def starts_sdk_daemon(self) -> bool:
        return self.allow_motion and self.network_interface != ""

    def process_env(self) -> dict[str, str]:
        """Environment for the ROS adapter process."""
        env = {
            "ROBONIX_VELOCITY_INPUT_TOPIC": self.twist_in_topic,
            "ROBONIX_ODOM_TOPIC": self.odom_topic,
            "ROBONIX_ODOM_SOURCE": self.odom_source,
            "ROBONIX_IMU_TOPIC": self.imu_topic,
            "ROBONIX_JOINT_STATE_TOPIC": self.joint_state_topic,
            "ROBONIX_ODOM_FRAME": self.odom_frame,
            "ROBONIX_BASE_FRAME": self.base_frame,
            "ROBONIX_IMU_FRAME": self.imu_frame,
            "ROBONIX_VELOCITY_FRAME": self.velocity_frame,
            "ROBONIX_PUBLISH_ODOM_TF": str(int(self.publish_odom_tf)),
            "ROBONIX_COMMAND_TIMEOUT_S": str(self.command_timeout_s),
            "ROBONIX_STATE_TIMEOUT_S": str(self.state_timeout_s),
        }
        # Pass daemon socket through environment — the adapter discovers
        # the socket path at runtime and passes it to the daemon core.
        env["G1_IPC_SOCKET"] = str(self.ipc_socket)
        env["G1_MAX_LINEAR_X_MPS"] = str(self.max_linear_x_mps)
        env["G1_MAX_LINEAR_Y_MPS"] = str(self.max_linear_y_mps)
        env["G1_MAX_ANGULAR_Z_RPS"] = str(self.max_angular_z_rps)
        env["G1_STAND_HEIGHT"] = str(self.stand_height)
        env["G1_SWING_HEIGHT"] = str(self.swing_height)
        return env

    def sdk_daemon_env(self, daemon_binary: Path) -> dict[str, str]:
        """Environment for the SDK daemon process (contains private DDS libs)."""
        # The daemon needs Unitree SDK2 DDS on its LD_LIBRARY_PATH so it
        # can link libddsc/libddscxx from the private install.
        daemon_dir = daemon_binary.parent
        lib_dir = daemon_dir.parent / "lib"
        env = {
            "LD_LIBRARY_PATH": str(lib_dir),
            "G1_IPC_SOCKET": str(self.ipc_socket),
            "G1_NETWORK_INTERFACE": self.network_interface,
            "G1_ALLOW_MOTION": "1" if self.allow_motion else "0",
        }
        return env

    def daemon_argv(self, daemon_binary: Path) -> list[str]:
        """Command-line for the SDK daemon."""
        argv = [
            str(daemon_binary),
            "--socket", str(self.ipc_socket),
            "--interface", self.network_interface,
            "--watchdog-ms", "300",
            "--max-vx", str(self.max_linear_x_mps),
            "--max-vy", str(self.max_linear_y_mps),
            "--max-wz", str(self.max_angular_z_rps),
            "--max-motion-ms", "0",
        ]
        if self.allow_motion:
            argv.insert(2, "--allow-motion")
            argv.extend(["--motion-ack", "G1_PHYSICAL_MOTION_APPROVED"])
        return argv

    def adapter_argv(self, adapter_binary: Path, params_file: Path) -> list[str]:
        """Command-line for the ROS2 adapter node."""
        return [
            str(adapter_binary),
            f"twist_in_topic:={self.twist_in_topic}",
            f"odom_topic:={self.odom_topic}",
        ]


def normalize_config(
    config: dict,
    env: dict[str, str],
    package_root: Path,
) -> RuntimeConfig:
    """Parse provider config into a validated RuntimeConfig."""
    raw_network = _resolve_env("NETWORK_INTERFACE", env, "lo")
    raw_allow_motion = (
        config.get("allow_motion", env.get("G1_ALLOW_MOTION", "false"))
        == "true"
    )

    tmp_dir = package_root / "rbnx-build" / "tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    ipc_socket = tmp_dir / "g1_daemon.sock"

    return RuntimeConfig(
        network_interface=raw_network,
        allow_motion=raw_allow_motion,
        ipc_socket=ipc_socket,
        twist_in_topic=str(config.get("twist_in_topic", "/cmd_vel")),
        odom_topic=str(config.get("odom_topic", "/odom")),
        odom_source=str(config.get("odom_source", "base_controller")),
        imu_topic=str(config.get("imu_topic", "/imu/data")),
        joint_state_topic=str(config.get("joint_state_topic", "/joint_states")),
        odom_frame=str(config.get("odom_frame", "odom")),
        base_frame=str(config.get("base_frame", "base_link")),
        imu_frame=str(config.get("imu_frame", "imu")),
        velocity_frame=str(config.get("velocity_frame", "odom")),
        publish_odom_tf=config.get("publish_odom_tf", True),
        stand_height=float(config.get("stand_height", 0.55)),
        swing_height=float(config.get("swing_height", 0.08)),
        max_linear_x_mps=float(config.get("max_linear_x_mps", 1.0)),
        max_linear_y_mps=float(config.get("max_linear_y_mps", 0.5)),
        max_angular_z_rps=float(config.get("max_angular_z_rps", 2.0)),
        max_linear_accel_mps2=float(config.get("max_linear_accel_mps2", 1.5)),
        max_angular_accel_rps2=float(config.get("max_angular_accel_rps2", 3.0)),
        command_timeout_s=float(config.get("command_timeout_s", 0.5)),
        state_timeout_s=float(config.get("state_timeout_s", 0.5)),
        startup_timeout_s=float(config.get("startup_timeout_s", 30.0)),
    )


def prepare_private_directory(socket_parent: Path) -> None:
    """Ensure the IPC socket parent directory is owned and mode 0700."""
    import os as _os
    import stat as _stat

    socket_parent.mkdir(parents=True, exist_ok=True)
    current_uid = _os.geteuid()
    st = socket_parent.stat()
    if st.st_uid != current_uid:
        raise OSError(
            f"socket parent {socket_parent} owned by uid {st.st_uid}, "
            f"expected {current_uid}"
        )
    socket_parent.chmod(_stat.S_IRWXU)
