import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


class RuntimeIntegrationTest(unittest.TestCase):
    def test_docker_start_mounts_manifest_directory_read_only(self):
        bash_major = int(
            subprocess.run(
                ["bash", "-c", "printf %s ${BASH_VERSINFO[0]}"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
        )
        if bash_major < 4:
            self.skipTest("provider Docker wrapper requires Bash 4 or newer")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / "package"
            deploy = root / "robot deploy"
            fake_bin = root / "bin"
            runtime_proto = root / "runtime-proto"
            package.mkdir()
            deploy.mkdir()
            fake_bin.mkdir()
            runtime_proto.mkdir()
            (runtime_proto / "atlas.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            proto_staging = package / "rbnx-build" / "proto-staging"
            proto_staging.mkdir(parents=True)
            (proto_staging / "navigation.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            docker_args = root / "docker.args"
            docker = fake_bin / "docker"
            docker.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ "${1:-}" == run ]]; then\n'
                '  if [[ " $* " == *" --network none "* ]]; then\n'
                '    for arg in "$@"; do\n'
                '      if [[ "$arg" == *:/proto-gen ]]; then\n'
                '        output="${arg%:/proto-gen}"\n'
                '        touch "$output/atlas_pb2.py"\n'
                '        touch "$output/navigation_pb2.py"\n'
                '        touch "$output/robonix_contracts_pb2_grpc.py"\n'
                '      fi\n'
                '    done\n'
                '    exit 0\n'
                '  fi\n'
                '  printf "%s\\n" "$@" > "$DOCKER_ARGS_FILE"\n'
                "fi\n",
                encoding="utf-8",
            )
            docker.chmod(0o755)
            rbnx = fake_bin / "rbnx"
            rbnx.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ "${1:-}" == path && "${2:-}" == runtime-proto ]]; then\n'
                '  echo "$RUNTIME_PROTO_DIR"\n'
                'else\n'
                '  echo /tmp/robonix-api\n'
                'fi\n',
                encoding="utf-8",
            )
            rbnx.chmod(0o755)
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{fake_bin}:/usr/bin:/bin",
                    "DOCKER_ARGS_FILE": str(docker_args),
                    "RUNTIME_PROTO_DIR": str(runtime_proto),
                    "RBNX_PACKAGE_ROOT": str(package),
                    "RBNX_INVOCATION_CWD": str(deploy),
                    "ROBONIX_NAV2_FORCE": "docker",
                }
            )
            subprocess.run(
                ["bash", str(ROOT / "scripts" / "start.sh")],
                check=True,
                env=env,
                capture_output=True,
                text=True,
            )
            args = docker_args.read_text(encoding="utf-8").splitlines()
            self.assertIn(f"RBNX_INVOCATION_CWD={deploy}", args)
            self.assertIn(f"{deploy}:{deploy}:ro", args)
            self.assertIn(
                f"{package}/rbnx-build/codegen/nav2_proto_gen:"
                "/nav2/rbnx-build/codegen/proto_gen:ro",
                args,
            )

    def test_nav_consumes_provider_pinned_canonical_odom(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('"odom",  "robonix/primitive/chassis/odom"', source)
        self.assertIn('providers = dict(cfg.get("provider_ids", {}) or {})', source)
        self.assertIn("provider_id=provider_id", source)
        self.assertNotIn('("odom",  "robonix/service/map/odom"', source)

    def test_pointcloud_projection_is_explicit(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn("scan_projection_config(cfg)", source)
        self.assertIn('"rtabmap_util", "lidar_deskewing"', source)
        self.assertIn('"scan:=/scanner/scan_raw"', source)
        self.assertIn('"-m", "nav2_wrapper.scan_filter"', source)

    def test_final_velocity_guard_owns_cmd_vel(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        guard = (ROOT / "nav2_wrapper" / "velocity_guard.py").read_text()
        self.assertIn("('cmd_vel', 'cmd_vel_guard_input')", source)
        self.assertIn("('cmd_vel_smoothed', 'cmd_vel_guard_input')", source)
        self.assertIn('"-m", "nav2_wrapper.velocity_guard"', source)
        self.assertIn('output_topic = resolve_velocity_output_topic(cfg)', source)
        self.assertIn('"ROBONIX_VELOCITY_OUTPUT_TOPIC": output_topic', source)
        self.assertIn('output_topic = resolve_velocity_output_topic({})', guard)
        self.assertIn('create_publisher(Twist, output_topic, 10)', guard)
        self.assertNotIn('create_publisher(Twist, "/cmd_vel"', guard)
        start = (ROOT / "scripts" / "start.sh").read_text()
        self.assertIn('"${ROBONIX_VELOCITY_OUTPUT_TOPIC+x}" == "x"', start)
        self.assertIn('"${VELOCITY_OUTPUT_ARGS[@]}"', start)

    def test_dynamic_speed_uses_nav2_live_speed_limit_without_replacing_goal(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        guard = (ROOT / "nav2_wrapper" / "velocity_guard.py").read_text()
        manifests = [
            (ROOT / name).read_text()
            for name in (
                "package_manifest.yaml",
                "package_manifest.jetson-docker.yaml",
                "package_manifest.jetson-native.yaml",
            )
        ]

        self.assertIn("from nav2_msgs.msg import SpeedLimit", source)
        self.assertIn(
            "node.create_publisher(\n            _SpeedLimit, settings.topic, 10",
            source,
        )
        self.assertIn(
            "_GUARD_SPEED_LIMIT_TOPIC = "
            '"/robonix/navigation/speed_limit_guard"',
            source,
        )
        self.assertIn("guard_publisher.publish(message)", source)
        self.assertIn("ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC", guard)
        self.assertIn("def _speed_channels_available()", source)
        self.assertIn("node.create_timer(1.0, _refresh_speed_subscriber)", source)
        self.assertNotIn("node.create_timer(1.0, _republish_speed_limit)", source)
        self.assertIn("def _adjust_speed_impl(", source)
        self.assertIn("def _set_speed_limit_impl(", source)
        self.assertIn("def _get_speed_limit_impl()", source)
        self.assertIn("_publish_speed_limit(decision.percentage)", source)
        speed_source = source[source.index("def _adjust_speed_impl"):]
        self.assertNotIn("_nav_queue.put", speed_source)
        self.assertIn("ROBONIX_NAV_MAX_LINEAR_SPEED_MPS", source)
        self.assertIn("bounded_linear_velocity(", guard)
        for manifest in manifests:
            self.assertIn("robonix/service/navigation/adjust_speed", manifest)
            self.assertIn(
                "robonix/service/navigation/set_speed_limit",
                manifest,
            )
            self.assertIn(
                "robonix/service/navigation/get_speed_limit",
                manifest,
            )

    def test_invalid_velocity_topic_is_rejected_before_dependency_discovery(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        validation = source.index("resolve_velocity_output_topic(cfg)", source.index("def init"))
        discovery = source.index("_build_remap_args(cfg)", source.index("def init"))
        self.assertLess(validation, discovery)
        self.assertIn('return Err(f"invalid velocity_output_topic: {error}")', source)

    def test_failed_init_cleans_up_nav_children(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn(
            'except Exception as e:  # noqa: BLE001\n        _kill_nav2()\n'
            '        return Err(f"spawn nav2 failed: {e}")',
            source,
        )

    def test_cancel_is_latched_before_action_handle_exists(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('state["cancel_requested"] = True', source)
        self.assertIn('state["state"] = "CANCELED"', source)
        self.assertIn("cancel queued until goal acceptance", source)
        self.assertIn("if cancel_requested:\n        _issue_cancel(gh, gid)", source)
        self.assertIn("def _cancel_response_cb", source)

    def test_docker_runtime_supports_interface_bound_cyclonedds(self):
        dockerfile = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn("ros-humble-rmw-cyclonedds-cpp", dockerfile)
        self.assertIn('-e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"', start)
        self.assertIn(
            '-e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}"',
            start,
        )
        self.assertIn(
            '-e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}"', start
        )

    def test_docker_codegen_matches_and_validates_runtime_protobuf(self):
        dockerfile = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("'setuptools>=77,<80'", dockerfile)
        self.assertIn("grpcio-tools>=1.60.0", dockerfile)
        self.assertIn('runtime_proto="$(rbnx path runtime-proto)"', start)
        self.assertIn("--network none", start)
        self.assertIn("python3 -m grpc_tools.protoc", start)
        self.assertIn("[importlib.import_module(name) for name in modules]", start)
        self.assertIn("codegen/nav2_proto_gen", start)
        self.assertIn(
            "/nav2/rbnx-build/codegen/proto_gen:ro",
            start,
        )
        self.assertIn("missing runtime-compatible protobuf stubs", entrypoint)

    def test_codegen_is_mcp_only_for_every_deployment_target(self):
        build = (ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")
        native_start = (ROOT / "scripts" / "start_native.sh").read_text(
            encoding="utf-8"
        )
        docker_entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("FLAGS=(--mcp)", build)
        self.assertNotIn("--ros2", build)
        self.assertNotIn("ROS2_IDL", build)
        for runtime_script in (native_start, docker_entrypoint):
            self.assertNotIn("codegen/ros2_idl", runtime_script)

    def test_config_directory_contains_only_the_neutral_template(self):
        names = sorted(path.name for path in (ROOT / "config").glob("*"))
        self.assertEqual(names, ["nav2_params.example.yml"])

    def test_frozen_legacy_assets_are_internal_compatibility_data(self):
        names = sorted(
            path.name for path in (ROOT / "nav2_wrapper" / "legacy_config").glob("*")
        )
        self.assertEqual(
            names,
            [
                "nav2_params.yml",
                "nav2_params_ranger_mini_v3.yml",
                "nav2_params_sim.yml",
                "nav2_params_slam.yml",
                "ranger_mini_v3_navigate.xml",
            ],
        )
        spec = (ROOT / "config.spec").read_text()
        self.assertIn("params_file:", spec)
        self.assertIn("path_base: directory containing robonix_manifest.yaml", spec)
        readme = (ROOT / "README.md").read_text()
        self.assertNotIn("params_profile:", readme)


if __name__ == "__main__":
    unittest.main()
