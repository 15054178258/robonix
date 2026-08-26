"""Command-line entry point for the Go2 read-only dashboard."""

from __future__ import annotations

import argparse
import os
from pathlib import Path


def _port(value: str) -> int:
    number = int(value)
    if number < 1 or number > 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return number


def _loopback_host(value: str) -> str:
    if str(value).strip() != "127.0.0.1":
        raise argparse.ArgumentTypeError(
            "dashboard host must be 127.0.0.1; use an SSH tunnel remotely"
        )
    return "127.0.0.1"


def _write_pid_file(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{os.getpid()}\n", encoding="ascii")


def _remove_own_pid_file(path: Path) -> None:
    try:
        if path.read_text(encoding="ascii").strip() == str(os.getpid()):
            path.unlink()
    except FileNotFoundError:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Robonix Go2 telemetry and optional Liaison voice dashboard"
    )
    parser.add_argument(
        "--host",
        type=_loopback_host,
        default=_loopback_host(
            os.environ.get("GO2_DASHBOARD_HOST", "127.0.0.1")
        ),
        help="HTTP bind address (fixed to 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=_port,
        default=_port(os.environ.get("GO2_DASHBOARD_PORT", "8092")),
    )
    parser.add_argument(
        "--log-level",
        choices=("critical", "error", "warning", "info", "debug"),
        default=os.environ.get("GO2_DASHBOARD_LOG_LEVEL", "info").lower(),
    )
    args = parser.parse_args()

    from uvicorn import run

    from .web import create_app

    pid_file = Path(
        os.environ.get("GO2_DASHBOARD_PID_FILE", "rbnx-build/run/dashboard.pid")
    ).resolve()
    _write_pid_file(pid_file)
    try:
        run(
            create_app(),
            host=args.host,
            port=args.port,
            log_level=args.log_level,
            access_log=False,
        )
    finally:
        _remove_own_pid_file(pid_file)


if __name__ == "__main__":
    main()
