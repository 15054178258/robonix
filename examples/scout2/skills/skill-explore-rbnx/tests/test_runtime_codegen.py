from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RuntimeCodegenTest(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, dict[str, str]]:
        package = root / "package"
        fake_bin = root / "bin"
        runtime_proto = root / "runtime-proto"
        package.mkdir()
        fake_bin.mkdir()
        runtime_proto.mkdir()
        (runtime_proto / "atlas.proto").write_text('syntax = "proto3";\n')
        staging = package / "rbnx-build" / "proto-staging"
        staging.mkdir(parents=True)
        (staging / "explore.proto").write_text('syntax = "proto3";\n')
        host_output = package / "rbnx-build" / "codegen" / "proto_gen"
        host_output.mkdir(parents=True)
        (host_output / "host-sentinel").touch()

        (fake_bin / "rbnx").write_text(
            "#!/usr/bin/env bash\n"
            'if [[ "${1:-}" == path && "${2:-}" == runtime-proto ]]; then\n'
            '  printf "%s\\n" "$RUNTIME_PROTO_DIR"\n'
            "else\n"
            '  printf "/tmp/robonix-api\\n"\n'
            "fi\n"
        )
        (fake_bin / "docker").write_text(
            "#!/usr/bin/env bash\n"
            'printf "docker" >> "$DOCKER_LOG"\n'
            'printf " %q" "$@" >> "$DOCKER_LOG"\n'
            'printf "\\n" >> "$DOCKER_LOG"\n'
            'for arg in "$@"; do\n'
            '  case "$arg" in\n'
            '    *:/proto-gen)\n'
            '      [[ "${FAKE_CODEGEN_FAIL:-0}" == 1 ]] && exit 41\n'
            '      out="${arg%:/proto-gen}"\n'
            '      touch "$out/atlas_pb2.py" "$out/atlas_pb2_grpc.py" '
            '"$out/explore_pb2.py" "$out/explore_pb2_grpc.py" '
            '"$out/robonix_contracts_pb2_grpc.py"\n'
            "      ;;\n"
            "  esac\n"
            "done\n"
        )
        for executable in fake_bin.iterdir():
            executable.chmod(0o755)

        wrapper = root / "start.sh"
        wrapper.write_text((ROOT / "scripts" / "start.sh").read_text())
        wrapper.chmod(0o755)
        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{fake_bin}:/usr/bin:/bin",
                "RBNX_PACKAGE_ROOT": str(package),
                "RUNTIME_PROTO_DIR": str(runtime_proto),
                "DOCKER_LOG": str(root / "docker.log"),
            }
        )
        return wrapper, env

    def test_wrapper_generates_offline_and_masks_only_runtime_stubs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wrapper, env = self._fixture(root)
            subprocess.run(["bash", str(wrapper)], check=True, env=env)
            codegen = root / "package" / "rbnx-build" / "codegen"
            self.assertTrue((codegen / "proto_gen" / "host-sentinel").exists())
            self.assertTrue((codegen / "explore_proto_gen" / "explore_pb2.py").exists())
            log = (root / "docker.log").read_text()
            self.assertIn("--network none", log)
            self.assertIn(
                f"{codegen}/explore_proto_gen:/explore/rbnx-build/codegen/proto_gen:ro",
                log,
            )

    def test_failed_codegen_preserves_previous_good_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wrapper, env = self._fixture(root)
            previous = (
                root
                / "package"
                / "rbnx-build"
                / "codegen"
                / "explore_proto_gen"
            )
            previous.mkdir()
            (previous / "known-good").touch()
            env["FAKE_CODEGEN_FAIL"] = "1"
            completed = subprocess.run(["bash", str(wrapper)], env=env)
            self.assertEqual(completed.returncode, 41)
            self.assertTrue((previous / "known-good").exists())


if __name__ == "__main__":
    unittest.main()
