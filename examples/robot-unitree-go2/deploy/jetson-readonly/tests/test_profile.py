#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import os
import posixpath
import re
import subprocess
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class JetsonReadonlyProfileTest(unittest.TestCase):
    def read(self, relative: str) -> str:
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_dockerfile_has_an_arm64_cpu_only_allowlist(self) -> None:
        dockerfile = self.read("Dockerfile")
        self.assertEqual(dockerfile.count("FROM --platform=linux/arm64"), 1)
        self.assertIn("AS verified_base", dockerfile)
        self.assertIn("FROM verified_base AS builder", dockerfile)
        self.assertIn("FROM verified_base AS runtime", dockerfile)
        self.assertIn("ARG ROS_IMAGE", dockerfile)
        self.assertNotRegex(dockerfile, r"ARG ROS_IMAGE\s*=")
        self.assertIn("NVIDIA_VISIBLE_DEVICES=void", dockerfile)
        self.assertNotIn("nvidia/cuda", dockerfile.lower())
        self.assertNotRegex(dockerfile, r"(?m)^COPY\s+\.\s")

        selected = re.search(
            r"--packages-select(?P<body>.*?)\n\s*# Build only",
            dockerfile,
            re.DOTALL,
        )
        self.assertIsNotNone(selected)
        for package in (
            "unitree_api",
            "unitree_go",
            "go2_chassis_adapter",
            "go2_sensors",
            "go2_description",
        ):
            self.assertRegex(selected.group("body"), rf"\b{package}\b")
        for forbidden in (
            "semantic_navigation",
            "go2_dashboard",
            "service-navigation-rbnx",
            "service-map-rbnx",
        ):
            self.assertNotIn(forbidden, dockerfile)

    def test_dockerfile_bounds_and_retries_package_downloads(self) -> None:
        dockerfile = self.read("Dockerfile")
        # Both builder and runtime stages perform an update and an install.
        # Every networked apt invocation must tolerate transient proxy errors
        # without waiting indefinitely.
        self.assertEqual(dockerfile.count("Acquire::Retries=5"), 4)
        self.assertEqual(dockerfile.count("Acquire::http::Timeout=30"), 4)
        self.assertEqual(dockerfile.count("Acquire::https::Timeout=30"), 4)
        self.assertEqual(
            dockerfile.count("APT::Update::Error-Mode=any"), 2
        )
        self.assertEqual(
            dockerfile.count(
                "id=robonix-go2-jammy-apt-archives-arm64,"
                "target=/var/cache/apt,sharing=locked"
            ),
            2,
        )
        self.assertEqual(
            dockerfile.count(
                "id=robonix-go2-jammy-apt-state-arm64,"
                "target=/var/lib/apt,sharing=locked"
            ),
            2,
        )
        self.assertEqual(
            dockerfile.count(
                "mv /etc/apt/apt.conf.d/docker-clean /tmp/docker-clean"
            ),
            2,
        )
        self.assertEqual(
            dockerfile.count(
                "mv /tmp/docker-clean /etc/apt/apt.conf.d/docker-clean"
            ),
            2,
        )
        self.assertIn(
            "https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/",
            dockerfile,
        )
        self.assertIn(
            "URIs: https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu",
            dockerfile,
        )
        self.assertIn("s|^Types: deb deb-src$|Types: deb|", dockerfile)

    def test_chassis_include_copy_matches_adapter_relative_include(self) -> None:
        dockerfile = self.read("Dockerfile")
        self.assertIn(
            "COPY packages/go2_chassis/include/go2_chassis "
            "/work/include/go2_chassis",
            dockerfile,
        )
        repository = ROOT.parents[1]
        cmake = (
            repository
            / "packages/go2_chassis/ros2_ws/src/go2_chassis_adapter/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertIn("../../../include", cmake)
        adapter_source = "/work/ros2/src/go2_chassis_adapter"
        resolved = posixpath.normpath(
            posixpath.join(adapter_source, "../../../include")
        )
        self.assertEqual(resolved, "/work/include")

    def test_context_excludes_vendor_examples_and_other_services(self) -> None:
        ignore = self.read("Dockerfile.dockerignore")
        self.assertTrue(ignore.startswith("**\n"))
        self.assertNotIn("!third_party/unitree_sdk2/example", ignore)
        for forbidden in (
            "!packages/semantic_navigation",
            "!packages/go2_dashboard",
            "!rbnx-boot",
        ):
            self.assertNotIn(forbidden, ignore)
        self.assertIn("!scripts/check_runtime_ownership.sh", ignore)
        self.assertIn("!scripts/runtime_lease.sh", ignore)
        self.assertIn("!deploy/jetson-readonly/camera-quality-healthcheck.py", ignore)

    def test_chassis_is_immutable_passive(self) -> None:
        config = yaml.safe_load(self.read("config/chassis-passive.yaml"))
        params = config["go2_chassis_adapter"]["ros__parameters"]
        self.assertIs(params["allow_motion"], False)
        self.assertEqual(params["allowed_modes"], [255])
        for control_parameter in (
            "cmd_vel_topic",
            "arm_service",
            "sdk_socket",
            "control_rate_hz",
        ):
            self.assertNotIn(control_parameter, params)

    def test_build_requires_digest_and_forwards_predefined_proxies(self) -> None:
        build = self.read("build.sh")
        self.assertIn("JETSON_READONLY_ROS_IMAGE_ID", build)
        self.assertIn("JETSON_READONLY_ROS_UPSTREAM_DIGEST", build)
        self.assertIn("robonix-local/ros-humble-ros-base-jammy:sha256-", build)
        self.assertIn("{{.Architecture}} {{.Os}} {{.Id}}", build)
        self.assertIn('"arm64"', build)
        self.assertIn('"linux"', build)
        self.assertIn('--build-arg "ROS_IMAGE=${base_image}"', build)
        self.assertIn('--build-arg "BASE_IMAGE_ID=${base_image_id}"', build)
        self.assertIn("BASE_IMAGE_UPSTREAM_DIGEST=${upstream_digest}", build)
        self.assertIn("--network host", build)
        self.assertIn("127\\.0\\.0\\.1|localhost", build)
        self.assertIn("unauthenticated loopback", build)
        for proxy in (
            "HTTP_PROXY",
            "HTTPS_PROXY",
            "NO_PROXY",
            "http_proxy",
            "https_proxy",
            "no_proxy",
        ):
            self.assertIn(proxy, build)
        dockerfile = self.read("Dockerfile")
        self.assertFalse(dockerfile.startswith("# syntax="))
        self.assertNotRegex(
            dockerfile,
            r"(?m)^ARG\s+(?:HTTP_PROXY|HTTPS_PROXY|NO_PROXY|http_proxy|https_proxy|no_proxy)",
        )

    def test_build_invocation_is_native_pinned_and_proxy_bounded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            fake_bin = temp / "bin"
            fake_bin.mkdir()
            arguments = temp / "docker-arguments"
            (fake_bin / "uname").write_text(
                "#!/usr/bin/env bash\nprintf 'aarch64\\n'\n", encoding="utf-8"
            )
            (fake_bin / "docker").write_text(
                "#!/usr/bin/env bash\n"
                "if [[ \"${1:-}\" == image && \"${2:-}\" == inspect ]]; then\n"
                "  printf 'arm64 linux %s\\n' \"$EXPECTED_IMAGE_ID\"\n"
                "  exit 0\n"
                "fi\n"
                "printf '%s\\n' \"$@\" > \"$DOCKER_ARGS\"\n",
                encoding="utf-8",
            )
            (fake_bin / "uname").chmod(0o755)
            (fake_bin / "docker").chmod(0o755)
            env = os.environ.copy()
            for name in (
                "HTTP_PROXY",
                "HTTPS_PROXY",
                "NO_PROXY",
                "http_proxy",
                "https_proxy",
                "no_proxy",
            ):
                env.pop(name, None)
            image_id = "sha256:" + "b" * 64
            env.update(
                {
                    "PATH": f"{fake_bin}:{env['PATH']}",
                    "DOCKER_ARGS": str(arguments),
                    "EXPECTED_IMAGE_ID": image_id,
                    "JETSON_READONLY_ROS_IMAGE": (
                        "robonix-local/ros-humble-ros-base-jammy:sha256-"
                        + "b" * 64
                    ),
                    "JETSON_READONLY_ROS_IMAGE_ID": image_id,
                    "JETSON_READONLY_ROS_UPSTREAM_DIGEST": "sha256:" + "a" * 64,
                    "HTTP_PROXY": "http://127.0.0.1:7897",
                    "HTTPS_PROXY": "http://localhost:7897",
                }
            )
            subprocess.run([str(ROOT / "build.sh")], check=True, env=env)
            invoked = arguments.read_text(encoding="utf-8")
            self.assertIn("build\n", invoked)
            self.assertIn("--platform\nlinux/arm64\n", invoked)
            self.assertIn("--network\nhost\n", invoked)
            self.assertIn(
                "ROS_IMAGE=robonix-local/ros-humble-ros-base-jammy:sha256-"
                + "b" * 64,
                invoked,
            )
            self.assertIn("BASE_IMAGE_ID=" + image_id, invoked)
            self.assertIn(
                "BASE_IMAGE_UPSTREAM_DIGEST=sha256:" + "a" * 64,
                invoked,
            )
            self.assertIn("HTTP_PROXY=http://127.0.0.1:7897", invoked)

            env["HTTP_PROXY"] = "http://user:password@127.0.0.1:7897"
            rejected = subprocess.run(
                [str(ROOT / "build.sh")],
                check=False,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertEqual(rejected.returncode, 44)
            self.assertIn("unauthenticated loopback", rejected.stderr)

    def test_entrypoint_has_no_control_or_network_mutation(self) -> None:
        runtime = "\n".join(
            self.read(path)
            for path in (
                "entrypoint.sh",
                "healthcheck.sh",
                "camera-quality-healthcheck.py",
                "../../scripts/check_runtime_ownership.sh",
                "../../scripts/runtime_lease.sh",
                "validate-network.sh",
                "config/chassis-passive.yaml",
                "config/sensors-readonly.yaml",
            )
        )
        forbidden = (
            "sport_mode_ctrl",
            "go2_sport_client",
            "go2_stand_example",
            "low_level_ctrl",
            "/api/sport/request",
            "/lowcmd",
            "ros2 topic pub",
            "ip address add",
            "ip addr add",
            "ip link set",
            "nmcli ",
            "netplan ",
            "dhclient ",
            "ifconfig ",
            "route add",
            "systemctl ",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, runtime)
        self.assertIn('expected_interface="eth0"', runtime)
        self.assertIn('expected_cidr="192.168.123.18/24"', runtime)
        entrypoint = self.read("entrypoint.sh")
        self.assertIn(
            "env LD_LIBRARY_PATH=/opt/robonix/camera/lib", entrypoint
        )
        self.assertNotRegex(entrypoint, r"(?m)^export LD_LIBRARY_PATH=")
        self.assertIn('"nx-${runtime_profile}"', entrypoint)
        self.assertIn("go2_runtime_lease_acquire", entrypoint)
        self.assertIn('"nx-${runtime_profile}" post', entrypoint)
        self.assertIn('if [[ "$runtime_profile" == full ]]', entrypoint)
        self.assertIn(
            "sensors-only: odom and tf_static publishers are absent",
            entrypoint,
        )

        verify = self.read("verify-runtime.sh")
        for daemon in ("go2_sport_daemon", "go2_chassis_sdk_daemon"):
            self.assertIn(daemon, verify)
        self.assertIn("overlay package allowlist mismatch", verify)
        self.assertIn("ROS executable allowlist mismatch", verify)
        self.assertIn("libcuda|libnvidia", verify)
        self.assertIn('bridge_dependencies="$(ldd "$bridge")"', verify)
        self.assertIn("libjpeg\\.so", verify)
        self.assertLess(
            verify.index('source "${overlay}/setup.bash"'),
            verify.index('bridge_dependencies="$(ldd "$bridge")"'),
        )

        dockerfile = self.read("Dockerfile")
        self.assertIn("libjpeg-turbo8", dockerfile)
        self.assertIn("ros-humble-rclpy", dockerfile)
        self.assertIn("camera-quality-healthcheck.py", dockerfile)

    def test_run_contract_is_hardened_and_nonpersistent(self) -> None:
        run = self.read("run.sh")
        required = (
            "--runtime runc",
            "--network host",
            "--read-only",
            "--tmpfs /tmp:rw,noexec,nosuid,nodev",
            "--cap-drop ALL",
            "--security-opt no-new-privileges=true",
            "--restart no",
            "--log-driver none",
            "--user 10001:10001",
        )
        for token in required:
            self.assertIn(token, run)
        forbidden = (
            "--privileged",
            "--device",
            "--volume",
            "--mount",
            "-v ",
            "/var/run/docker.sock",
            "--pid host",
            "--ipc host",
            "--gpus",
        )
        for token in forbidden:
            self.assertNotIn(token, run)
        self.assertIn("--sensors-only", run)
        self.assertIn('GO2_NX_RUNTIME_PROFILE=${runtime_profile}', run)

        health = self.read("healthcheck.sh")
        self.assertIn("runtime-profile", health)
        self.assertIn("required=(sensor-relay)", health)
        self.assertIn("camera-quality-healthcheck.py", health)
        self.assertIn("GO2_CAMERA_HEALTH_TIMEOUT_S", self.read("camera-quality-healthcheck.py"))
        self.assertIn('values.get("quality_ready") == "true"', self.read("camera-quality-healthcheck.py"))
        self.assertIn('values.get("healthy") == "true"', self.read("camera-quality-healthcheck.py"))

        dockerfile = self.read("Dockerfile")
        self.assertIn(
            "COPY scripts/check_runtime_ownership.sh "
            "/opt/robonix/profile/check-runtime-ownership.sh",
            dockerfile,
        )

    def test_no_compose_file_or_compose_invocation(self) -> None:
        self.assertFalse(any(ROOT.glob("*compose*")))
        for path in ROOT.iterdir():
            if path.is_file():
                self.assertNotIn("docker compose", path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
