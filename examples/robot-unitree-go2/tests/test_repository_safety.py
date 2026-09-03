import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class RepositorySafetyTest(unittest.TestCase):
    @staticmethod
    def _soma_exports(node: dict) -> list[tuple[str, str]]:
        exports = [
            (entry["provider_id"], capability["path"])
            for entry in node.get("exports", [])
            for capability in entry.get("capabilities", [])
        ]
        for child in node.get("components", []):
            exports.extend(RepositorySafetyTest._soma_exports(child))
        return exports

    def test_default_motion_is_disabled(self) -> None:
        env = (ROOT / ".env.example").read_text(encoding="utf-8")
        self.assertRegex(env, r"(?m)^GO2_ALLOW_MOTION=false$")
        self.assertRegex(env, r"(?m)^GO2_OPERATOR_PRESENT=false$")
        self.assertRegex(env, r"(?m)^GO2_ALLOWED_MODES=$")
        self.assertRegex(env, r"(?m)^GO2_ALLOWED_STATE_MARKERS=$")

        deployment = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        chassis = next(
            item for item in deployment["primitive"] if item["name"] == "go2_chassis"
        )
        self.assertEqual(chassis["config"]["allowed_modes"], [255])
        self.assertNotIn("ipc_socket", chassis["config"])

    def test_readonly_audits_do_not_spawn_or_reuse_ros2_daemon(self) -> None:
        for relative in (
            "scripts/list_go2_topics.sh",
            "scripts/check_tf.sh",
            "scripts/collect_go2_publisher_locators_readonly.sh",
        ):
            with self.subTest(relative=relative):
                source = (ROOT / relative).read_text(encoding="utf-8")
                self.assertIn("--no-daemon", source)
        ownership_wrapper = (
            ROOT / "scripts" / "check_runtime_ownership.sh"
        ).read_text(encoding="utf-8")
        ownership_checker = (
            ROOT / "scripts" / "check_runtime_ownership.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("ros2 topic info", ownership_wrapper.replace("`", ""))
        self.assertIn("_rclpy.Node(", ownership_checker)
        self.assertIn("get_count_publishers", ownership_checker)
        self.assertNotIn("from rclpy.node import Node", ownership_checker)
        self.assertNotIn("create_publisher", ownership_checker)
        self.assertNotIn("create_subscription", ownership_checker)

    def test_readonly_services_are_loopback_and_use_short_socket_defaults(self) -> None:
        deployment = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        dashboard = next(
            item for item in deployment["service"] if item["name"] == "go2_dashboard"
        )
        sensors = next(
            item for item in deployment["primitive"] if item["name"] == "go2_sensors"
        )
        self.assertEqual(dashboard["config"]["host"], "127.0.0.1")
        self.assertEqual(
            dashboard["config"]["pose_topic"], "/robonix/map/pose"
        )
        self.assertNotIn("camera_ipc_socket", sensors["config"])
        self.assertEqual(
            sensors["config"]["source_mode"],
            "${GO2_SENSOR_SOURCE_MODE}",
        )

    def test_runtime_placement_is_explicit_and_disables_duplicate_publishers(self) -> None:
        env = (ROOT / ".env.example").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        package_start = (
            ROOT / "packages" / "go2_sensors" / "start.sh"
        ).read_text(encoding="utf-8")
        provider = (
            ROOT
            / "packages"
            / "go2_sensors"
            / "go2_sensors_provider"
            / "main.py"
        ).read_text(encoding="utf-8")

        self.assertRegex(env, r"(?m)^GO2_RUNTIME_PLACEMENT=workstation-local$")
        self.assertIn(
            'export GO2_RUNTIME_PLACEMENT="${GO2_RUNTIME_PLACEMENT:-workstation-local}"',
            start,
        )
        self.assertIn("workstation-full-nx-sensors", start)
        self.assertIn("workstation-ui-nx-full", start)
        self.assertIn("check_runtime_ownership.sh", start)
        self.assertIn("local|external", package_start)
        self.assertIn(
            "EXTERNAL/NX MODE: no local relay, camera daemon, or bridge is started.",
            package_start,
        )
        self.assertIn('if cfg["source_mode"] == "local":', provider)
        self.assertNotIn('source_mode == "auto"', provider)

    def test_semantic_router_is_a_fail_closed_boot_dependency(self) -> None:
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        stop = (ROOT / "stop.sh").read_text(encoding="utf-8")
        signal_guard = (
            ROOT / "scripts" / "runtime_signal_guard.sh"
        ).read_text(encoding="utf-8")
        health = (ROOT / "scripts" / "check_semantic_intent_health.py").read_text(
            encoding="utf-8"
        )

        self.assertLess(
            start.index("start_semantic_router\n"),
            start.index('"$RBNX_CLI" boot --no-update-check -f "$MANIFEST"'),
        )
        self.assertIn("semantic-intent-router.lock", start)
        self.assertIn("go2-semantic-router-lease-v1", start)
        self.assertIn('"${expected_vlm_url}/models"', start)
        self.assertIn('process_is_same_and_live "$SEMANTIC_ROUTER_PID"', start)
        self.assertIn("go2_runtime_install_cleanup_traps", start)
        self.assertIn("go2_runtime_wait_for_first_exit", start)
        self.assertIn("wait -n -p EXITED_RUNTIME_PID", signal_guard)
        self.assertIn("EXITED_RUNTIME_PID=\"${EXITED_RUNTIME_PID:-}\"", signal_guard)
        self.assertIn("stop_semantic_router", stop)
        self.assertIn("process_holds_lock", stop)
        self.assertIn("127.0.0.1", health)
        self.assertIn('parsed.path != "/v1/models"', health)
        self.assertIn("ProxyHandler({})", health)
        self.assertEqual(
            start.count("export SEMANTIC_INTENT_EXECUTION_MODE=live"), 1
        )
        self.assertEqual(
            start.count("export SEMANTIC_INTENT_EXECUTION_MODE=preview"), 1
        )
        self.assertLess(
            start.index("export GO2_ALLOW_MOTION=false", start.index("case \"${GO2_ALLOW_MOTION,,}\"")),
            start.index("export SEMANTIC_INTENT_EXECUTION_MODE=preview"),
        )

    def test_start_and_stop_fall_back_to_workspace_rbnx(self) -> None:
        for relative in ("start.sh", "stop.sh"):
            with self.subTest(relative=relative):
                source = (ROOT / relative).read_text(encoding="utf-8")
                self.assertIn('/.tools/rbnx/bin"', source)
                self.assertIn('[[ -x "$WORKSPACE_TOOLS_BIN/rbnx" ]]', source)
                self.assertIn('export PATH="$WORKSPACE_TOOLS_BIN:$PATH"', source)
                self.assertIn(
                    'export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"',
                    source,
                )
                self.assertIn("validate_robonix_home.py", source)
                self.assertNotIn("$HOME/.robonix", source)
                if relative == "start.sh":
                    self.assertGreater(
                        source.rindex(
                            'export ROBONIX_HOME="$WORKSPACE_ROOT/.tools/robonix-home"'
                        ),
                        source.index('source "$DEPLOY_DIR/.env"'),
                    )

    def test_start_requires_a_workspace_cached_funasr_model(self) -> None:
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertIn(
            'export MODELSCOPE_CACHE="$DEPLOY_DIR/.cache/modelscope"', start
        )
        self.assertIn(
            'export MODELSCOPE_CREDENTIALS_PATH="$DEPLOY_DIR/.cache/modelscope/credentials"',
            start,
        )
        self.assertIn("speech_paraformer-large_asr_nat-zh-cn", start)
        self.assertIn("for required_model_file in model.pt config.yaml", start)
        self.assertIn('[[ -s "$GO2_FUNASR_MODEL_PATH/$required_model_file" ]]', start)
        self.assertIn("runtime download fallback is disabled", start)
        self.assertEqual(
            manifest["env"]["MODELSCOPE_CREDENTIALS_PATH"],
            "${ROBONIX_DEPLOY_DIR}/.cache/modelscope/credentials",
        )
        speech = next(
            item for item in manifest["service"] if item["name"] == "speech"
        )
        self.assertEqual(speech["config"]["funasr_model"], "${GO2_FUNASR_MODEL_PATH}")

    def test_control_plane_and_optional_uis_are_loopback_only(self) -> None:
        deployment = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        env = deployment["env"]
        self.assertEqual(env["ROBONIX_PROVIDER_BIND_HOST"], "127.0.0.1")
        self.assertEqual(env["ROBONIX_ADVERTISE_HOST"], "127.0.0.1")
        self.assertEqual(env["SPEECH_BIND_ADDR"], "127.0.0.1")
        self.assertEqual(env["MAPPING_ENABLE_VIZ"], "false")
        self.assertEqual(env["MAPPING_WEBUI_HOST"], "127.0.0.1")
        self.assertEqual(env["SCENE_WEB_HOST"], "127.0.0.1")
        self.assertEqual(env["ROBONIX_FORCE_CPU"], "1")
        self.assertEqual(env["SCENE_CG_FORCE_CPU"], "1")
        self.assertNotIn("SCENE_WEB_PORT", env)
        self.assertNotIn("MAPPING_WEBUI_PORT", env)
        self.assertEqual(
            env["SCENE_DATA_DIR"],
            "${ROBONIX_DEPLOY_DIR}/rbnx-build/data/scene",
        )
        self.assertNotIn("SCENE_OBJECT_MEMORY_DB", env)
        self.assertNotIn("SCENE_GRAPH_CACHE_DIR", env)
        self.assertNotIn("SCENE_ANNOTATIONS_DIR", env)

        for name, config in deployment["system"].items():
            listen = config.get("listen")
            if listen is not None:
                with self.subTest(system=name):
                    self.assertTrue(str(listen).startswith("127.0.0.1:"))
        self.assertEqual(deployment["system"]["scene"]["web_host"], "127.0.0.1")
        self.assertEqual(deployment["system"]["scene"]["web_port"], 50107)

        mapping = next(
            item for item in deployment["service"] if item["name"] == "mapping"
        )
        dashboard = next(
            item for item in deployment["service"] if item["name"] == "go2_dashboard"
        )
        audio = next(
            item for item in deployment["primitive"]
            if item["name"] == "audio_client_bridge"
        )
        self.assertEqual(mapping["config"]["webui_host"], "127.0.0.1")
        self.assertEqual(mapping["config"]["webui_port"], 8091)
        self.assertEqual(dashboard["config"]["host"], "127.0.0.1")
        self.assertEqual(audio["config"]["listen_host"], "127.0.0.1")

        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        owned = (
            "export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1",
            "export ROBONIX_ADVERTISE_HOST=127.0.0.1",
            "export SPEECH_BIND_ADDR=127.0.0.1",
            "export MAPPING_ENABLE_VIZ=false",
            "export MAPPING_WEBUI_HOST=127.0.0.1",
            "export SCENE_WEB_HOST=127.0.0.1",
        )
        for source in (start, build):
            for expected in owned:
                self.assertIn(expected, source)
            self.assertIn("unset DISPLAY", source)
            self.assertNotIn("ROBONIX_PROVIDER_BIND_HOST:-", source)
            self.assertNotIn("ROBONIX_ADVERTISE_HOST:-", source)

    def test_x86_services_use_docker_defaults_and_inherit_ros_domain(self) -> None:
        deployment = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        services = {item["name"]: item for item in deployment["service"]}
        for name in ("mapping", "nav2"):
            with self.subTest(name=name):
                self.assertNotIn("manifest", services[name])
                self.assertIn("path", services[name])
                self.assertNotIn("url", services[name])
                self.assertNotIn("branch", services[name])
        self.assertNotIn("ROS_DOMAIN_ID", deployment.get("env", {}))

        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        self.assertIn(
            "bzip2 cmake colcon curl docker g++ git python3 rbnx tar uv",
            build,
        )
        self.assertLess(
            build.index('--log-base "$DEPLOY_DIR/rbnx-build/unitree_ros2/log"'),
            build.index("  build \\\n"),
            "colcon global options must precede the build verb",
        )
        self.assertIn("ros-humble-rosidl-generator-dds-idl", build)
        self.assertIn("required_commands=(flock ip nmcli timeout)", start)
        self.assertIn("required_commands+=(docker git rbnx)", start)
        self.assertNotIn("jetson-native", build)
        self.assertNotIn("jetson-native", start)

        overlay_helper = (
            ROOT / "scripts" / "build_robonix_ros2_overlay.sh"
        ).read_text(encoding="utf-8")
        for package in (
            "builtin_interfaces",
            "geometry_msgs",
            "sensor_msgs",
            "std_msgs",
            "rcl_interfaces",
        ):
            self.assertIn(package, overlay_helper)
        self.assertIn("--packages-skip", overlay_helper)
        self.assertIn(
            'rm -rf -- "$idl_root/build" "$idl_root/install" "$idl_root/log"',
            overlay_helper,
        )
        selected_packages = {
            "packages/go2_chassis/scripts/build.sh": "--packages-select lifecycle",
            "packages/go2_sensors/scripts/build_ros.sh": "--packages-select lifecycle",
            "packages/go2_description/scripts/build.sh": "--packages-select lifecycle",
            "packages/semantic_navigation/scripts/build.sh": (
                "--packages-select lifecycle semantic_navigation map"
            ),
        }
        for relative, package_selection in selected_packages.items():
            package_build = (ROOT / relative).read_text(encoding="utf-8")
            self.assertIn("build_robonix_ros2_overlay.sh", package_build)
            self.assertIn("robonix_build_ros2_overlay", package_build)
            self.assertIn(package_selection, package_build.replace("\\\n  ", ""))
            self.assertNotIn("--packages-up-to map", package_build)

    def test_ros2_overlay_helper_cleans_only_stale_system_packages(self) -> None:
        helper = ROOT / "scripts" / "build_robonix_ros2_overlay.sh"
        with tempfile.TemporaryDirectory() as temp:
            temp_root = Path(temp)
            fake_bin = temp_root / "bin"
            fake_bin.mkdir()
            fake_colcon = fake_bin / "colcon"
            fake_colcon.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            fake_colcon.chmod(0o755)
            env = os.environ.copy()
            env["PATH"] = f"{fake_bin}:{env['PATH']}"

            command = (
                'source "$1"; '
                'robonix_build_ros2_overlay "$2" --packages-select lifecycle'
            )
            stale_layouts = (
                Path("build/sensor_msgs"),
                Path("install/sensor_msgs"),
                Path("install/share/sensor_msgs"),
            )
            for index, stale_relative in enumerate(stale_layouts):
                with self.subTest(stale_relative=stale_relative):
                    idl_root = temp_root / f"stale-{index}"
                    (idl_root / "src").mkdir(parents=True)
                    (idl_root / "build" / "lifecycle").mkdir(parents=True)
                    (idl_root / "install" / "lifecycle").mkdir(parents=True)
                    (idl_root / "log").mkdir()
                    (idl_root / stale_relative).mkdir(parents=True, exist_ok=True)
                    subprocess.run(
                        ["bash", "-c", command, "bash", str(helper), str(idl_root)],
                        check=True,
                        env=env,
                    )
                    self.assertFalse((idl_root / "build").exists())
                    self.assertFalse((idl_root / "install").exists())
                    self.assertFalse((idl_root / "log").exists())

            clean_root = temp_root / "custom-only"
            (clean_root / "src").mkdir(parents=True)
            custom_build = clean_root / "build" / "lifecycle"
            custom_install = clean_root / "install" / "lifecycle"
            custom_build.mkdir(parents=True)
            custom_install.mkdir(parents=True)
            subprocess.run(
                ["bash", "-c", command, "bash", str(helper), str(clean_root)],
                check=True,
                env=env,
            )
            self.assertTrue(custom_build.is_dir())
            self.assertTrue(custom_install.is_dir())

    def test_upstream_compatibility_gate_is_strict_and_traceable(self) -> None:
        build = (ROOT / "build.sh").read_text(encoding="utf-8")
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        compatibility = (
            ROOT / "scripts" / "verify_upstream_compatibility.sh"
        ).read_text(encoding="utf-8")
        self.assertNotIn("gsub(/^[\"'", compatibility)
        self.assertNotIn("[[:space:]]{2}", compatibility)
        self.assertNotIn("[[:space:]]{4}", compatibility)
        self.assertIn('path="${path#\\\"}"', compatibility)
        self.assertIn('path="${path#\\\'}"', compatibility)
        self.assertIn("robonix/primitive/lidar/lidar3d", compatibility)
        self.assertIn("robonix/primitive/chassis/odom", compatibility)
        self.assertIn("Scene strict perception policy", compatibility)
        self.assertIn("Scene preview-only perception gate", compatibility)
        self.assertIn("if not perception_enabled(config):", compatibility)
        self.assertIn("def topic_qos_policy", compatibility)
        self.assertIn("ros-humble-rmw-cyclonedds-cpp", compatibility)
        self.assertIn("cancel queued until goal acceptance", compatibility)
        self.assertIn("ROBONIX_PROVIDER_BIND_HOST", compatibility)
        self.assertIn("host=self._bind_host", compatibility)
        self.assertIn("ROBONIX_ADVERTISE_HOST", compatibility)
        self.assertIn("Scene Docker GPU opt-out gate", compatibility)
        self.assertIn("ROBONIX_FORCE_CPU", compatibility)
        self.assertIn("SCENE_HOST_DATA_DIR", compatibility)
        self.assertIn('/data/robonix', compatibility)
        self.assertIn('MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"', compatibility)
        self.assertIn('if [[ "$VIZ_ENABLED" == true ]]; then', compatibility)
        self.assertIn("XHOST_AUTHORIZED=true", compatibility)
        self.assertIn("xhost -local:docker", compatibility)
        self.assertIn("Mapping generated system-interface isolation", compatibility)
        self.assertIn("colcon build --packages-select map", compatibility)
        self.assertIn("Navigation MCP-only code generation", compatibility)
        self.assertIn("Navigation generated ROS system-interface overlay", compatibility)
        self.assertIn("Navigation native generated ROS overlay source", compatibility)
        self.assertIn("Navigation container generated ROS overlay source", compatibility)
        self.assertIn("forbid_text", compatibility)
        self.assertIn("upstream-lock.txt", compatibility)
        self.assertIn("manifest_repo_path mapping", compatibility)
        self.assertIn("manifest_repo_path nav2", compatibility)
        self.assertIn("status --porcelain --untracked-files=normal", compatibility)
        self.assertNotIn('rbnx-boot/cache/mapping', compatibility)
        self.assertNotIn('rbnx-boot/cache/nav2', compatibility)

        manifest = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        services = {item["name"]: item for item in manifest["service"]}
        self.assertEqual(
            Path(services["mapping"]["path"].rstrip("/")).stem,
            "service-map-rbnx",
        )
        self.assertEqual(
            Path(services["nav2"]["path"].rstrip("/")).stem,
            "service-navigation-rbnx",
        )
        self.assertIn("verify_upstream_compatibility.sh", build)
        self.assertIn("verify_submodule_pins.sh", build)
        self.assertIn("ROBONIX_CARGO_PACKAGE_ARGS=(-p robonix-codegen)", build)
        self.assertIn('ROBONIX_CARGO_PACKAGE_ARGS+=(-p "$package")', build)
        self.assertIn('--manifest-path "$ROBONIX_SOURCE_ROOT/Cargo.toml"', build)
        self.assertIn("export ROBONIX_CODEGEN_BIN=", build)
        self.assertIn("WORKSPACE_RBNX_PYTHON_DIR", build)
        self.assertIn("WORKSPACE_RBNX_PYTHON_DIR", start)
        self.assertIn(
            '"$(command -v python3)" == "$WORKSPACE_RBNX_PYTHON_DIR/python3"',
            start,
        )
        self.assertLess(
            start.index('source "$DEPLOY_DIR/.env"'),
            start.index('export PATH="$WORKSPACE_RBNX_PYTHON_DIR:$PATH"'),
        )
        self.assertIn("python3 -c 'import grpc'", start)
        self.assertIn("import grpc_tools.protoc", build)
        self.assertIn("export RBNX_BUILD_PROXY=1", build)
        self.assertNotIn("127.0.0.1:7897", build)
        self.assertIn("verify_upstream_compatibility.sh", start)

    def test_start_owns_and_validates_the_dedicated_dds_interface(self) -> None:
        start = (ROOT / "start.sh").read_text(encoding="utf-8")
        self.assertIn('"192.168.123.99/24"', start)
        self.assertIn('"/virtual/"', start)
        self.assertIn('/wireless"', start)
        self.assertIn('route show default dev "$GO2_NETWORK_INTERFACE"', start)
        self.assertIn("ip -o -4 addr show dev", start)
        self.assertNotIn('scope global', start)
        self.assertIn("ip -o -6 addr show dev", start)
        self.assertIn("ip -6 route show default dev", start)
        self.assertIn(
            "--get-values IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS",
            start,
        )
        self.assertIn('export CYCLONEDDS_URI="$GO2_CYCLONEDDS_URI"', start)
        self.assertNotIn('if [[ -z "${CYCLONEDDS_URI:-}" ]]', start)

    def test_private_sdk_runtime_installs_third_party_licenses(self) -> None:
        chassis = (
            ROOT / "packages" / "go2_chassis" / "sdk_daemon" / "CMakeLists.txt"
        ).read_text(encoding="utf-8")
        camera = (
            ROOT / "packages" / "go2_sensors" / "camera_daemon" / "CMakeLists.txt"
        ).read_text(encoding="utf-8")
        for source in (chassis, camera):
            self.assertIn("share/licenses/unitree_sdk2", source)
            self.assertIn("eclipse-cyclonedds/cyclonedds/LICENSE", source)
            self.assertIn("eclipse-cyclonedds/cyclonedds-cxx/LICENSE", source)
            self.assertIn("eclipse-iceoryx/iceoryx/LICENSE", source)
            self.assertIn("Tencent/rapidjson/LICENSE", source)

        inventory = (ROOT / "THIRD_PARTY.md").read_text(encoding="utf-8")
        self.assertIn("EPL-2.0 OR EDL-1.0", inventory)
        self.assertIn("BSD-3-Clause", inventory)

    def test_public_landmark_is_not_verified(self) -> None:
        data = yaml.safe_load((ROOT / "config" / "semantic_landmarks.yaml").read_text(encoding="utf-8"))
        self.assertEqual(data["schema_version"], 2)
        self.assertIs(type(data["map_generation"]), int)
        self.assertGreaterEqual(data["map_generation"], 0)
        vending = next(row for row in data["landmarks"] if row["name"] == "自动售货机")
        self.assertFalse(vending["verified"])

        deployment = yaml.safe_load(
            (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        )
        semantic = next(item for item in deployment["skill"] if item["name"] == "semantic_navigation")
        self.assertEqual(semantic["config"]["mapping_provider_id"], "mapping")
        self.assertGreater(float(semantic["config"]["lifecycle_wait_s"]), 0.0)
        self.assertEqual(
            semantic["config"]["status_endpoint"],
            "http://127.0.0.1:${GO2_DASHBOARD_PORT}/api/semantic-task",
        )

    def test_no_posture_or_low_level_capability(self) -> None:
        manifest = (ROOT / "robonix_manifest.yaml").read_text(encoding="utf-8")
        soma = (ROOT / "soma.yaml").read_text(encoding="utf-8")
        for forbidden in ("/lowcmd", "chassis/posture", "RecoveryStand", "StandUp", "StandDown"):
            self.assertNotIn(forbidden, manifest)
            self.assertNotIn(forbidden, soma)

    def test_navigation_tree_has_no_spin_or_backup(self) -> None:
        for filename in ("navigate.xml", "navigate_through_poses.xml"):
            xml = ET.parse(ROOT / "config" / filename)
            tags = {node.tag for node in xml.iter()}
            self.assertNotIn("Spin", tags)
            self.assertNotIn("BackUp", tags)

    def test_audit_scripts_do_not_publish(self) -> None:
        readonly_names = ("check", "audit", "list", "record")
        for path in (ROOT / "scripts").glob("*.sh"):
            if path.stem.startswith(readonly_names):
                source = path.read_text(encoding="utf-8")
                self.assertIsNone(re.search(r"ros2\s+topic\s+pub", source), path)

    def test_nx_audit_is_bounded_and_non_mutating(self) -> None:
        path = ROOT / "scripts" / "audit_nx_readonly.sh"
        source = path.read_text(encoding="utf-8")
        self.assertTrue(os.access(path, os.X_OK))
        self.assertIn("timeout --signal=INT --kill-after=1s", source)
        self.assertIn("systemctl is-active --quiet docker.service", source)
        self.assertIn("--host unix:///var/run/docker.sock", source)
        self.assertNotIn('run "Docker client/server" docker version', source)
        self.assertNotIn("DOCKER_HOST", source)
        self.assertNotIn("DOCKER_CONTEXT", source)
        self.assertIsNone(re.search(r"(?m)^[ \t]*sudo(?:[ \t]|$)", source))
        for forbidden in (
            r"\bapt(?:-get)?\b",
            r"\bnmcli\s+(?:connection|con)\s+(?:add|modify|delete|up|down)\b",
            r"\bip\s+(?:addr|route|link)\s+(?:add|del|set|replace)\b",
            r"\btimedatectl\s+set-",
            r"\bsystemctl\s+(?:start|stop|restart|enable|disable)\b",
            r"/(?:etc/shadow|root/\.ssh|home/[^/]+/\.ssh)",
            r"\b(?:printenv|history)\b",
        ):
            self.assertIsNone(re.search(forbidden, source), forbidden)

    def test_local_soma_exports_exist_in_package_manifests(self) -> None:
        soma = yaml.safe_load((ROOT / "soma.yaml").read_text(encoding="utf-8"))
        local_packages = {
            "go2_chassis": ROOT / "packages" / "go2_chassis",
            "go2_sensors": ROOT / "packages" / "go2_sensors",
            "robot_description": ROOT / "packages" / "go2_description",
            "semantic_navigation": ROOT / "packages" / "semantic_navigation",
        }
        declared = {
            provider_id: {
                item["name"]
                for item in yaml.safe_load(
                    (package / "package_manifest.yaml").read_text(encoding="utf-8")
                )["capabilities"]
            }
            for provider_id, package in local_packages.items()
        }
        local_exports = [
            (provider_id, capability)
            for provider_id, capability in self._soma_exports(soma["robot"])
            if provider_id in local_packages
        ]
        self.assertTrue(local_exports)
        for provider_id, capability in local_exports:
            with self.subTest(provider_id=provider_id, capability=capability):
                self.assertIn(capability, declared[provider_id])


if __name__ == "__main__":
    unittest.main()
