#!/usr/bin/env python3
"""Read-only network topology gate for the bounded first-motion probe.

The validator changes no address, route, connection, DNS setting, or link
state.  It exists so the existing 10 cm commissioning envelope can use either
the original direct cable or one explicitly selected private Wi-Fi adapter
without silently accepting the laptop's Internet interface.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
from pathlib import Path
import re
import subprocess
from typing import Iterable


EXPECTED_ADDRESS = "192.168.123.99/24"
EXPECTED_SUBNET = "192.168.123.0/24"
INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]+$")
UUID_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$"
)


class NetworkGateError(RuntimeError):
    """The selected interface does not match the approved private topology."""


@dataclass(frozen=True)
class NetworkSnapshot:
    interface: str
    transport: str
    physical: bool
    wireless: bool
    operstate: str
    ipv4: tuple[str, ...]
    ipv6: tuple[str, ...]
    default_v4_interfaces: tuple[str, ...]
    default_v6_interfaces: tuple[str, ...]
    connection_name: str
    connection_uuid: str
    gateway_dns_values: tuple[str, ...]
    private_route: str
    robot_route: str
    orin_route: str


def _tokens(line: str) -> tuple[str, ...]:
    return tuple(token for token in line.strip().split() if token)


def _route_uses_private_interface(
    route: str,
    *,
    target: str,
    interface: str,
) -> bool:
    tokens = _tokens(route)
    if not tokens or tokens[0] != target:
        return False
    try:
        dev_index = tokens.index("dev")
        src_index = tokens.index("src")
    except ValueError:
        return False
    if dev_index + 1 >= len(tokens) or src_index + 1 >= len(tokens):
        return False
    return (
        tokens[dev_index + 1] == interface
        and tokens[src_index + 1] == EXPECTED_ADDRESS.split("/", 1)[0]
        and "via" not in tokens
    )


def validate_snapshot(
    snapshot: NetworkSnapshot,
    *,
    expected_connection_uuid: str | None,
    expected_connection_name: str | None,
    internet_interface: str,
    robot_ip: str,
    orin_ip: str,
) -> None:
    if snapshot.transport not in {"wired", "wireless-private"}:
        raise NetworkGateError("unsupported first-motion network transport")
    if not snapshot.physical:
        raise NetworkGateError("Go2 interface must be a physical device")
    if snapshot.operstate != "up":
        raise NetworkGateError("Go2 interface is not UP")
    if snapshot.ipv4 != (EXPECTED_ADDRESS,):
        raise NetworkGateError(
            f"Go2 interface must have exactly {EXPECTED_ADDRESS}"
        )
    if snapshot.ipv6:
        raise NetworkGateError("Go2 interface must have no IPv6 address")
    if snapshot.interface in snapshot.default_v4_interfaces:
        raise NetworkGateError("Go2 interface must have no IPv4 default route")
    if snapshot.interface in snapshot.default_v6_interfaces:
        raise NetworkGateError("Go2 interface must have no IPv6 default route")
    if any(value.strip() for value in snapshot.gateway_dns_values):
        raise NetworkGateError("Go2 interface must have no gateway or DNS")
    if internet_interface == snapshot.interface:
        raise NetworkGateError(
            "private Go2 and laptop Internet interfaces must be different"
        )
    if (
        not snapshot.default_v4_interfaces
        or set(snapshot.default_v4_interfaces) != {internet_interface}
    ):
        raise NetworkGateError(
            "all laptop IPv4 default routes must be owned by the Internet "
            "interface"
        )

    private_routes = tuple(
        line.strip()
        for line in snapshot.private_route.splitlines()
        if line.strip()
    )
    if len(private_routes) != 1:
        raise NetworkGateError(
            "private subnet must have exactly one direct route"
        )
    private_tokens = _tokens(private_routes[0])
    if (
        not private_tokens
        or private_tokens[0] != EXPECTED_SUBNET
        or "via" in private_tokens
        or "dev" not in private_tokens
        or private_tokens[private_tokens.index("dev") + 1]
        != snapshot.interface
        or "src" not in private_tokens
        or private_tokens[private_tokens.index("src") + 1]
        != EXPECTED_ADDRESS.split("/", 1)[0]
    ):
        raise NetworkGateError("private subnet route is not direct and exclusive")
    if not _route_uses_private_interface(
        snapshot.robot_route,
        target=robot_ip,
        interface=snapshot.interface,
    ):
        raise NetworkGateError("robot route does not use the private interface")
    if not _route_uses_private_interface(
        snapshot.orin_route,
        target=orin_ip,
        interface=snapshot.interface,
    ):
        raise NetworkGateError("Orin route does not use the private interface")

    if snapshot.transport == "wired":
        if snapshot.wireless:
            raise NetworkGateError("wired transport cannot use a wireless interface")
        return

    if not snapshot.wireless:
        raise NetworkGateError(
            "wireless-private transport requires a physical Wi-Fi interface"
        )
    if not expected_connection_uuid or UUID_RE.fullmatch(
        expected_connection_uuid
    ) is None:
        raise NetworkGateError(
            "wireless-private transport requires an exact connection UUID"
        )
    if snapshot.connection_uuid != expected_connection_uuid:
        raise NetworkGateError("active private Wi-Fi UUID does not match")
    if not expected_connection_name:
        raise NetworkGateError(
            "wireless-private transport requires an exact connection name"
        )
    if snapshot.connection_name != expected_connection_name:
        raise NetworkGateError("active private Wi-Fi connection name does not match")


def _run(arguments: Iterable[str]) -> str:
    result = subprocess.run(
        tuple(arguments),
        check=False,
        capture_output=True,
        text=True,
        timeout=3.0,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise NetworkGateError(
            f"read-only network query failed: {' '.join(arguments)}: {detail}"
        )
    return result.stdout.strip()


def _addresses(interface: str, family: str) -> tuple[str, ...]:
    output = _run(("ip", "-o", family, "addr", "show", "dev", interface))
    result: list[str] = []
    for line in output.splitlines():
        fields = line.split()
        marker = "inet" if family == "-4" else "inet6"
        if marker not in fields:
            continue
        index = fields.index(marker)
        if index + 1 < len(fields):
            result.append(fields[index + 1])
    return tuple(result)


def _default_interfaces(family: str) -> tuple[str, ...]:
    output = _run(("ip", family, "route", "show", "default"))
    result: list[str] = []
    for line in output.splitlines():
        fields = line.split()
        if "dev" in fields:
            index = fields.index("dev")
            if index + 1 < len(fields):
                result.append(fields[index + 1])
    return tuple(result)


def _nmcli_value(interface: str, field: str) -> str:
    return _run(("nmcli", "-g", field, "device", "show", interface))


def collect_snapshot(
    *,
    interface: str,
    transport: str,
    robot_ip: str,
    orin_ip: str,
) -> NetworkSnapshot:
    sysfs = Path("/sys/class/net") / interface
    if not sysfs.is_dir():
        raise NetworkGateError(f"network interface does not exist: {interface}")
    resolved = sysfs.resolve()
    physical = "/virtual/" not in str(resolved)
    operstate = (sysfs / "operstate").read_text(encoding="utf-8").strip()
    gateway_dns = tuple(
        _nmcli_value(interface, field)
        for field in (
            "IP4.GATEWAY",
            "IP4.DNS",
            "IP6.GATEWAY",
            "IP6.DNS",
        )
    )
    return NetworkSnapshot(
        interface=interface,
        transport=transport,
        physical=physical,
        wireless=(sysfs / "wireless").is_dir(),
        operstate=operstate,
        ipv4=_addresses(interface, "-4"),
        ipv6=_addresses(interface, "-6"),
        default_v4_interfaces=_default_interfaces("-4"),
        default_v6_interfaces=_default_interfaces("-6"),
        connection_name=_nmcli_value(interface, "GENERAL.CONNECTION"),
        connection_uuid=_nmcli_value(interface, "GENERAL.CON-UUID"),
        gateway_dns_values=gateway_dns,
        private_route=_run(
            ("ip", "-4", "route", "show", "exact", EXPECTED_SUBNET)
        ),
        robot_route=_run(("ip", "-4", "route", "get", robot_ip)).splitlines()[0],
        orin_route=_run(("ip", "-4", "route", "get", orin_ip)).splitlines()[0],
    )


def _ip(value: str) -> str:
    try:
        parsed = ipaddress.ip_address(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be an IP address") from error
    if parsed.version != 4:
        raise argparse.ArgumentTypeError("must be an IPv4 address")
    return str(parsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument(
        "--transport",
        choices=("wired", "wireless-private"),
        required=True,
    )
    parser.add_argument("--internet-interface", required=True)
    parser.add_argument("--robot-ip", type=_ip, required=True)
    parser.add_argument("--orin-ip", type=_ip, required=True)
    parser.add_argument("--connection-uuid")
    parser.add_argument("--connection-name")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for label, value in (
        ("interface", args.interface),
        ("internet interface", args.internet_interface),
    ):
        if INTERFACE_RE.fullmatch(value) is None:
            raise SystemExit(f"{label} contains unsupported characters")
    try:
        snapshot = collect_snapshot(
            interface=args.interface,
            transport=args.transport,
            robot_ip=args.robot_ip,
            orin_ip=args.orin_ip,
        )
        validate_snapshot(
            snapshot,
            expected_connection_uuid=args.connection_uuid,
            expected_connection_name=args.connection_name,
            internet_interface=args.internet_interface,
            robot_ip=args.robot_ip,
            orin_ip=args.orin_ip,
        )
    except (OSError, NetworkGateError, subprocess.SubprocessError) as error:
        print(f"first-motion network gate failed: {error}")
        return 1
    print(
        "PASS: first-motion network topology is "
        f"{args.transport} on {args.interface}; no configuration changed"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
