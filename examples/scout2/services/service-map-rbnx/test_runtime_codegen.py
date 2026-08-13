#!/usr/bin/env python3
"""Regression coverage for Docker-local protobuf generation."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent


class RuntimeCodegenTests(unittest.TestCase):
    def test_all_runtime_images_include_the_generator(self) -> None:
        for name in ("Dockerfile", "Dockerfile.fastlio2_full", "Dockerfile.jetson"):
            dockerfile = (ROOT / "docker" / name).read_text(encoding="utf-8")
            with self.subTest(name=name):
                self.assertIn("grpcio==1.80.0", dockerfile)
                self.assertIn("grpcio-tools==1.76.0", dockerfile)
                self.assertIn("protobuf==6.33.6", dockerfile)

    def test_start_generates_validates_and_bind_masks_runtime_stubs(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        self.assertIn('runtime_proto="$(rbnx path runtime-proto)"', start)
        self.assertIn("--network none", start)
        self.assertIn("python3 -m grpc_tools.protoc", start)
        self.assertIn("[importlib.import_module(name) for name in modules]", start)
        self.assertIn("codegen/mapping_proto_gen", start)
        self.assertIn(
            "/mapping/rbnx-build/codegen/proto_gen:ro",
            start,
        )

    def test_native_path_exits_before_docker_codegen(self) -> None:
        start = (ROOT / "scripts" / "start.sh").read_text(encoding="utf-8")
        native_exit = start.index('exec bash "${PKG}/scripts/start_native.sh"')
        docker_codegen = start.index("prepare_runtime_proto_gen\n")
        self.assertLess(native_exit, docker_codegen)

    def test_host_codegen_uses_and_validates_native_runtime_python(self) -> None:
        build = (ROOT / "scripts" / "build.sh").read_text(encoding="utf-8")
        native = (ROOT / "scripts" / "start_native.sh").read_text(encoding="utf-8")
        self.assertIn('PYBIN="${MAPPING_NATIVE_PYTHON:-python3}"', build)
        self.assertIn('RBNX_CODEGEN_PYTHON="$PYBIN"', build)
        self.assertIn('"$PYBIN" - <<', build)
        self.assertIn("import atlas_pb2_grpc", build)
        self.assertIn("import map_mcp", build)
        self.assertIn('PYBIN="${MAPPING_NATIVE_PYTHON:-python3}"', native)

    def test_entrypoint_reports_missing_runtime_stubs(self) -> None:
        entrypoint = (ROOT / "docker" / "entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn("missing runtime-compatible protobuf stubs", entrypoint)
        self.assertIn("map_pb2.py", entrypoint)


if __name__ == "__main__":
    unittest.main()
