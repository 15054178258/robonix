#!/usr/bin/env python3
from __future__ import annotations

import re
import subprocess
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "deploy" / "jetson-full-nomotion"


def read(name: str) -> str:
    return (PROFILE / name).read_text(encoding="utf-8")


class JetsonFullNomotionProfileTests(unittest.TestCase):
    def test_blueprint_file_set_is_explicit(self) -> None:
        expected = {
            "Dockerfile",
            "Dockerfile.dockerignore",
            "README.md",
            "build.sh",
            "entrypoint.sh",
            "healthcheck.sh",
            "profile.yaml",
            "robonix_manifest.yaml",
            "run.sh",
            "stop.sh",
            "validate-network.sh",
            "validate-static.sh",
            "verify-image-rootfs.sh",
        }
        self.assertEqual(expected, {path.name for path in PROFILE.iterdir()})
        self.assertIn("not a runnable", read("README.md"))
        self.assertIn("runtime-complete=false", read("README.md"))

    def test_shell_syntax(self) -> None:
        for path in sorted(PROFILE.glob("*.sh")):
            subprocess.run(["bash", "-n", str(path)], check=True)

    def test_network_gate_is_exact_and_read_only(self) -> None:
        script = read("validate-network.sh")
        for required in (
            'expected_arch="aarch64"',
            'expected_interface="eth0"',
            'expected_cidr="192.168.123.18/24"',
            "scope global",
            "route show default dev",
            "disable_ipv6",
        ):
            self.assertIn(required, script)
        for mutator in ("nmcli", "netplan", "dhclient", " address add", " link set"):
            self.assertNotIn(mutator, script)

    def test_dockerfile_is_arm64_humble_multistage_blueprint(self) -> None:
        dockerfile = read("Dockerfile")
        self.assertGreaterEqual(dockerfile.count("FROM "), 3)
        for required in (
            "FROM --platform=linux/arm64 ${JETSON_READONLY_IMAGE} AS verified_base",
            "FROM verified_base AS navigation_builder",
            "FROM verified_base AS runtime_blueprint",
            'test "${TARGETARCH}" = "arm64"',
            "/opt/ros/humble/setup.bash",
            "ros-humble-nav2-bringup",
            "ros-humble-navigation2",
            "ros-humble-pointcloud-to-laserscan",
            "ros-humble-rtabmap-ros",
            'io.robonix.go2.runtime-complete="false"',
            'io.robonix.go2.motion="false"',
            "ENTRYPOINT [\"/opt/robonix/profile/entrypoint.sh\"]",
        ):
            self.assertIn(required, dockerfile)
        self.assertNotIn("--platform=linux/amd64", dockerfile)
        self.assertNotIn("nvidia/cuda", dockerfile.lower())

    def test_build_requires_exact_local_arm64_readonly_base(self) -> None:
        script = read("build.sh")
        for required in (
            'if [[ "$(uname -m)" != "aarch64" ]]',
            "JETSON_FULL_NOMOTION_BASE_IMAGE_ID",
            "robonix-local/jetson-readonly:sha256-",
            'profile" != "jetson-readonly"',
            'motion" != "disabled"',
            "--platform linux/arm64",
            "--pull=false",
            'io.robonix.go2.runtime-complete',
            "false false ${manifest_sha256}",
        ):
            self.assertIn(required, script)
        self.assertNotRegex(script, r"docker\s+(?:container\s+)?run")
        self.assertNotIn("docker pull", script)
        self.assertNotIn("--network host", script)

    def test_allowlisted_context_excludes_motion_sdk_and_secrets(self) -> None:
        ignore = read("Dockerfile.dockerignore")
        self.assertTrue(ignore.startswith("**\n"))
        for required in (
            "third_party/unitree_sdk2/**",
            "packages/go2_chassis/sdk_daemon/**",
            "**/go2_sport_daemon",
            "**/rbnx-build/**",
            "**/build/**",
            "**/install/**",
            "**/log/**",
            "**/.git/**",
            "**/.cache/**",
            "**/.ssh/**",
            "**/.env",
            "**/*.key",
            "**/*.pem",
        ):
            self.assertIn(required, ignore)

        dockerfile = read("Dockerfile")
        for forbidden_copy in (
            "COPY third_party/unitree_sdk2",
            "COPY packages/go2_chassis/sdk_daemon",
            "COPY packages/go2_chassis/rbnx-build",
        ):
            self.assertNotIn(forbidden_copy, dockerfile)

    def test_manifest_is_explicit_jetson_native_and_no_motion(self) -> None:
        manifest = yaml.safe_load(read("robonix_manifest.yaml"))
        self.assertEqual(manifest["env"]["GO2_ALLOW_MOTION"], "false")
        self.assertEqual(
            manifest["env"]["ROBONIX_VELOCITY_OUTPUT_TOPIC"],
            "/robonix/nomotion/cmd_vel",
        )
        self.assertNotIn("scene", manifest["system"])

        primitives = {item["name"]: item for item in manifest["primitive"]}
        chassis = primitives["go2_chassis"]["config"]
        self.assertIs(chassis["allow_motion"], False)
        self.assertIs(chassis["operator_present"], False)
        self.assertEqual(chassis["allowed_modes"], [255])
        self.assertEqual(
            chassis["twist_in_topic"],
            "/robonix/nomotion/chassis_input_disabled",
        )

        services = {item["name"]: item for item in manifest["service"]}
        self.assertEqual(
            services["mapping"]["manifest"],
            "package_manifest.jetson-native.yaml",
        )
        self.assertEqual(
            services["navigation"]["manifest"],
            "package_manifest.jetson-native.yaml",
        )
        self.assertEqual(services["navigation"]["capability_id"], "nav2")
        self.assertEqual(
            services["navigation"]["config"]["velocity_output_topic"],
            "/robonix/nomotion/cmd_vel",
        )
        mapping_config = services["mapping"]["config"]
        dashboard_config = services["go2_dashboard"]["config"]
        self.assertEqual(dashboard_config["pose_topic"], "/robonix/map/pose")
        self.assertEqual(dashboard_config["map_frame"], "map")
        self.assertEqual(
            dashboard_config["base_frame"], mapping_config["base_frame"]
        )

        scalar_strings: list[str] = []

        def collect(value: object) -> None:
            if isinstance(value, dict):
                for child in value.values():
                    collect(child)
            elif isinstance(value, list):
                for child in value:
                    collect(child)
            elif isinstance(value, str):
                scalar_strings.append(value)

        collect(manifest)
        self.assertNotIn("/cmd_vel", scalar_strings)
        self.assertNotIn("/api/sport/request", scalar_strings)
        self.assertNotIn("/lowcmd", scalar_strings)

    def test_profile_cannot_claim_runtime_completion(self) -> None:
        profile = yaml.safe_load(read("profile.yaml"))
        self.assertEqual(profile["platform"]["architecture"], "arm64")
        self.assertEqual(profile["platform"]["ros_distribution"], "humble")
        self.assertEqual(
            profile["runtime"],
            {
                "complete": False,
                "launch_allowed": False,
                "reason": "missing-verified-arm64-robonix-runtime",
                "required_artifacts": [
                    {"path": "/opt/robonix/source", "status": "missing"},
                    {
                        "path": "/opt/robonix/models/funasr-zh-online/model.pt",
                        "status": "missing",
                    },
                    {
                        "path": "/opt/robonix/deploy/third_party/service-map-rbnx/rbnx-build/codegen",
                        "status": "missing",
                    },
                    {
                        "path": "/opt/robonix/deploy/third_party/service-navigation-rbnx/rbnx-build/codegen",
                        "status": "missing",
                    },
                ],
            },
        )
        self.assertIs(profile["motion"]["enabled"], False)
        self.assertEqual(profile["motion"]["chassis_sdk_daemon"], "excluded")
        self.assertEqual(profile["motion"]["unitree_sport_client"], "excluded")
        self.assertEqual(
            profile["motion"]["navigation_velocity_output"],
            "/robonix/nomotion/cmd_vel",
        )
        self.assertGreaterEqual(len(profile["build_gates"]), 7)

    def test_launcher_has_hardened_oci_boundary(self) -> None:
        script = read("run.sh")
        for required in (
            "--runtime runc",
            "--network host",
            "--read-only",
            "--cap-drop ALL",
            "--security-opt no-new-privileges=true",
            "--restart no",
            "--user 10001:10001",
        ):
            self.assertIn(required, script)
        self.assertNotRegex(script, r"--pid(?:=|\s+)host")
        self.assertNotIn("docker.sock", script)
        self.assertNotIn("--privileged", script)
        self.assertNotIn("--device", script)
        self.assertNotIn("--cap-add", script)
        self.assertNotIn("--ipc host", script)
        self.assertNotIn("--volume", script)
        self.assertNotIn("--mount", script)

    def test_launcher_requires_separately_reviewed_complete_image(self) -> None:
        script = read("run.sh")
        self.assertIn('readonly image="${JETSON_FULL_NOMOTION_IMAGE:-}"', script)
        self.assertIn('if [[ -z "$image" ]]', script)
        self.assertIn("this launcher never pulls", script)
        self.assertIn(r"^sha256:[0-9a-f]{64}$", script)
        self.assertIn(
            "$expected_image_id arm64 linux jetson-full-nomotion false true",
            script,
        )
        self.assertIn(".Config.Healthcheck", script)
        self.assertIn('[\"/opt/robonix/profile/entrypoint.sh\"]', script)
        self.assertIn('"$expected_image_id"', script)

    def test_motion_and_velocity_gates_are_immutable(self) -> None:
        launcher = read("run.sh")
        entrypoint = read("entrypoint.sh")
        healthcheck = read("healthcheck.sh")
        combined = "\n".join((launcher, entrypoint, healthcheck))
        self.assertIn("--env ROBONIX_MOTION_ENABLED=false", launcher)
        self.assertIn("--env GO2_CHASSIS_ALLOW_MOTION=false", launcher)
        self.assertNotIn("ROBONIX_MOTION_ENABLED=true", combined)
        self.assertNotIn("GO2_CHASSIS_ALLOW_MOTION=true", combined)
        self.assertIn("/robonix/nomotion/cmd_vel", combined)
        self.assertNotRegex(combined, r'=["\']?/cmd_vel["\']?(?:\s|$)')

        for forbidden in (
            "ros2 topic pub",
            "/api/sport/request",
            "/lowcmd",
            "SportClient",
            "StopMove",
            "Move(",
        ):
            self.assertNotIn(forbidden, combined)

    def test_time_readiness_is_required_before_any_future_launch(self) -> None:
        launcher = read("run.sh")
        entrypoint = read("entrypoint.sh")
        healthcheck = read("healthcheck.sh")
        self.assertIn("deploy/jetson-time-sync/status.sh", launcher)
        self.assertIn("minimum_epoch=1704067200", launcher)
        self.assertIn("--env ROBONIX_TIME_READY=1", launcher)
        self.assertIn("${ROBONIX_TIME_READY:-}", entrypoint)
        self.assertIn("${ROBONIX_TIME_READY:-}", healthcheck)

    def test_entrypoint_and_healthcheck_remain_fail_closed(self) -> None:
        entrypoint = read("entrypoint.sh")
        healthcheck = read("healthcheck.sh")
        self.assertIn('if [[ "$$" -ne 1 ]]', entrypoint)
        self.assertIn("/var/run/docker.sock", entrypoint)
        self.assertIn("CapEff CapPrm CapInh CapAmb CapBnd", entrypoint)
        self.assertIn("${ROBONIX_RUNTIME_COMPLETE:-}", entrypoint)
        self.assertIn("incomplete ARM64 blueprint", entrypoint)
        self.assertRegex(entrypoint, r"exit 78\s*$")
        self.assertIn("${ROBONIX_RUNTIME_COMPLETE:-}", healthcheck)
        self.assertIn("intentionally unhealthy", healthcheck)
        self.assertRegex(healthcheck, r"exit 78\s*$")

    def test_rootfs_verifier_enforces_absence_and_static_contract(self) -> None:
        verifier = read("verify-image-rootfs.sh")
        for required in (
            "go2_sport_daemon",
            "packages/go2_chassis/sdk_daemon",
            "third_party/unitree_sdk2",
            'magic != b"\\x7fELF"',
            "forbidden Unitree motion token",
            "profile[\"runtime\"]",
            "chassis[\"config\"][\"allow_motion\"] is False",
            'services["navigation"]["config"]["velocity_output_topic"]',
            'dashboard_config["pose_topic"]',
            'dashboard_config["map_frame"]',
            'dashboard_config["base_frame"] == mapping_config["base_frame"]',
        ):
            self.assertIn(required, verifier)

    def test_manifest_assets_are_copied_or_explicitly_missing(self) -> None:
        dockerfile = read("Dockerfile")
        profile = yaml.safe_load(read("profile.yaml"))
        self.assertIn("COPY soma.yaml /opt/robonix/deploy/soma.yaml", dockerfile)
        self.assertIn("COPY config /opt/robonix/deploy/config", dockerfile)
        required = {
            item["path"]: item["status"]
            for item in profile["runtime"]["required_artifacts"]
        }
        self.assertEqual(required["/opt/robonix/source"], "missing")
        self.assertEqual(
            required["/opt/robonix/models/funasr-zh-online/model.pt"],
            "missing",
        )

    def test_stop_only_targets_recognized_nomotion_container(self) -> None:
        script = read("stop.sh")
        self.assertIn("jetson-full-nomotion false", script)
        self.assertIn("docker stop --time 20", script)
        self.assertNotIn("docker rm -f", script)


if __name__ == "__main__":
    unittest.main()
