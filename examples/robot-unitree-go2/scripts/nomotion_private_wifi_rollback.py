#!/usr/bin/env python3
"""Temporarily validate the private Go2 Wi-Fi and restore the original links.

The default preflight mode is read-only.  Both execute modes are deliberately
named ``--execute-after-approval*`` because every use changes NetworkManager
state and therefore requires a fresh, separately explained operator approval.

This utility never starts ROS, Robonix, or a motion process.  In the interactive
mode, nmcli asks for the one-time Wi-Fi credential through the controlling
terminal; Python never reads, captures, persists, or prints it.  Connection
UUIDs are kept only in process memory and are intentionally omitted from
status/error messages.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import ipaddress
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Callable, Iterator, Mapping, Sequence


TARGET_SSID = "Robonix-Go2"
WIFI_INTERFACE = "wlo1"
WIRED_INTERFACE = "enp108s0"
WIRED_PROFILE_NAME = "go2-readonly"
HOST_ADDRESS = "192.168.123.99/24"
HOST_INTERFACE = ipaddress.IPv4Interface(HOST_ADDRESS)
HOST_IP = HOST_INTERFACE.ip
PRIVATE_NETWORK = HOST_INTERFACE.network
TARGET_HOSTS = ("192.168.123.1", "192.168.123.18", "192.168.123.161")
IPV4LL_NETWORK = ipaddress.IPv4Network("169.254.0.0/16")
ALLOWED_WIFI_IPV4LL_ROUTE_TOKENS = (
    str(IPV4LL_NETWORK),
    "dev",
    WIFI_INTERFACE,
    "scope",
    "link",
    "metric",
    "1000",
)
IPV6_LINK_LOCAL_NETWORK = ipaddress.IPv6Network("fe80::/64")
IPV6_MULTICAST_NETWORK = ipaddress.IPv6Network("ff00::/8")
ALLOWED_IPV6_LOOPBACK_MAIN_TOKENS = (
    "::1",
    "dev",
    "lo",
    "proto",
    "kernel",
    "metric",
    "256",
    "pref",
    "medium",
)
ALLOWED_IPV6_LOOPBACK_LOCAL_TOKENS = (
    "local",
    "::1",
    "dev",
    "lo",
    "table",
    "local",
    "proto",
    "kernel",
    "metric",
    "0",
    "pref",
    "medium",
)
REQUIRED_IPV6_LOOPBACK_ROUTE_KEYS = frozenset(
    {
        ("loopback-main", "lo"),
        ("loopback-local", "lo"),
    }
)
REQUIRED_IPV6_DOCKER_ROUTE_KEYS = frozenset(
    {
        ("bridge-main", "docker0"),
        ("bridge-local", "docker0"),
        ("bridge-multicast", "docker0"),
    }
)

NOMOTION_PROFILE = "workstation-full-nomotion-corrected-v1"
NOMOTION_PLACEMENT = "workstation-local"
NOMOTION_MOTION_FLAG = "false"

UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)

QUERY_TIMEOUT_S = 8.0
ACTIVATION_TIMEOUT_S = 25.0
PING_TIMEOUT_S = 8.0
MAX_HOLD_SECONDS = 300
ROLLBACK_ATTEMPTS = 3
STABLE_OBSERVATIONS = 4
STABILITY_SAMPLE_LIMIT = 10
STABILITY_INTERVAL_S = 0.25
RUNTIME_RECHECK_INTERVAL_S = 1.0

ROUTE_TYPES = {
    "anycast",
    "blackhole",
    "broadcast",
    "local",
    "multicast",
    "nat",
    "prohibit",
    "throw",
    "unreachable",
}


class SwitchError(RuntimeError):
    """A fixed, credential-free failure safe to show to the operator."""


class CommandTimedOut(SwitchError):
    """A bounded external command exceeded its deadline."""


class SwitchInterrupted(BaseException):
    """A handled process signal requested rollback at a command boundary."""

    def __init__(self, signum: int) -> None:
        self.signum = signum
        super().__init__(signum)


class SignalLatch:
    """Latch signals without throwing through an in-flight subprocess call."""

    def __init__(self) -> None:
        self.pending_signum: int | None = None
        self.defer_depth = 0

    def capture(self, signum: int) -> None:
        if self.pending_signum is None:
            self.pending_signum = signum

    def raise_if_pending(self) -> None:
        if self.pending_signum is not None and self.defer_depth == 0:
            raise SwitchInterrupted(self.pending_signum)

    @contextmanager
    def deferred(self) -> Iterator[None]:
        self.defer_depth += 1
        try:
            yield
        finally:
            self.defer_depth -= 1


class CommandRunner:
    """Run fixed argv and surface a latched signal only after the child exits."""

    def __init__(self, signal_latch: SignalLatch) -> None:
        self.signal_latch = signal_latch

    def run(
        self,
        argv: Sequence[str],
        *,
        timeout: float,
        operation: str,
    ) -> str:
        self.signal_latch.raise_if_pending()
        output = ""
        failure: SwitchError | None = None
        try:
            completed = subprocess.run(
                list(argv),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
                timeout=timeout,
                # Terminal INT/HUP is latched by the parent.  Keep it from
                # terminating nmcli half-way through an activation request;
                # rollback starts only after this bounded command boundary.
                start_new_session=True,
            )
            output = completed.stdout
            if completed.returncode != 0:
                # Never forward command stderr: NetworkManager/plugin
                # diagnostics are outside the credential-free output contract.
                failure = SwitchError(f"{operation} failed")
        except subprocess.TimeoutExpired:
            # subprocess.run kills and waits for the client process before this
            # boundary.  NetworkManager may still complete a D-Bus activation,
            # so rollback never trusts the timeout as a final network state.
            failure = CommandTimedOut(f"{operation} timed out")
        except OSError:
            failure = SwitchError(f"{operation} could not be started")

        self.signal_latch.raise_if_pending()
        if failure is not None:
            raise failure
        return output

    @staticmethod
    def require_interactive_terminal() -> None:
        streams = (sys.stdin, sys.stdout, sys.stderr)
        try:
            ready = all(stream.isatty() for stream in streams)
        except (AttributeError, OSError):
            ready = False
        if not ready:
            raise SwitchError(
                "interactive activation requires visible terminal stdin/stdout/stderr"
            )

    def run_interactive(
        self,
        argv: Sequence[str],
        *,
        timeout: float,
        operation: str,
    ) -> None:
        """Run nmcli with inherited terminal FDs; never handle secret bytes."""

        self.signal_latch.raise_if_pending()
        failure: SwitchError | None = None
        try:
            completed = subprocess.run(
                list(argv),
                # These explicit None values inherit the terminal directly.
                # Python never reads or captures the one-time secret.
                stdin=None,
                stdout=None,
                stderr=None,
                check=False,
                timeout=timeout,
                # Keep the controlling terminal for nmcli --ask.  The parent
                # only raises its latched signal after this bounded child exits.
                start_new_session=False,
            )
            if completed.returncode != 0:
                failure = SwitchError(f"{operation} failed")
        except subprocess.TimeoutExpired:
            failure = CommandTimedOut(f"{operation} timed out")
        except OSError:
            failure = SwitchError(f"{operation} could not be started")

        self.signal_latch.raise_if_pending()
        if failure is not None:
            raise failure


@dataclass(frozen=True)
class ConnectionState:
    campus_uuid: str
    target_uuid: str
    wired_uuid: str


@dataclass(frozen=True)
class AddressRecord:
    interface: str
    address: ipaddress.IPv4Interface


@dataclass(frozen=True)
class RouteRecord:
    family: int
    route_type: str
    network: ipaddress.IPv4Network | ipaddress.IPv6Network
    interface: str | None
    table: str
    has_gateway: bool
    tokens: tuple[str, ...]


class PrivateWifiRollback:
    def __init__(
        self,
        runner: CommandRunner,
        *,
        environ: Mapping[str, str],
        signal_latch: SignalLatch | None = None,
        sys_class_net: Path = Path("/sys/class/net"),
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.runner = runner
        self.environ = environ
        self.signal_latch = signal_latch or SignalLatch()
        self.sys_class_net = sys_class_net
        self.sleep = sleep
        self.monotonic = monotonic
        self.state: ConnectionState | None = None
        self.rollback_armed = False
        self.rollback_started = False

    def _run(
        self,
        argv: Sequence[str],
        *,
        operation: str,
        timeout: float = QUERY_TIMEOUT_S,
    ) -> str:
        self.signal_latch.raise_if_pending()
        try:
            output = self.runner.run(argv, timeout=timeout, operation=operation)
        except BaseException:
            self.signal_latch.raise_if_pending()
            raise
        self.signal_latch.raise_if_pending()
        return output

    def _run_interactive(
        self,
        argv: Sequence[str],
        *,
        operation: str,
        timeout: float,
    ) -> None:
        self.signal_latch.raise_if_pending()
        try:
            self.runner.run_interactive(
                argv,
                timeout=timeout,
                operation=operation,
            )
        except BaseException:
            self.signal_latch.raise_if_pending()
            raise
        self.signal_latch.raise_if_pending()

    def _nm_value(self, field: str, uuid: str, operation: str) -> str:
        return self._run(
            (
                "nmcli",
                "--get-values",
                field,
                "connection",
                "show",
                "uuid",
                uuid,
            ),
            operation=operation,
        ).strip()

    def _optional_active_uuid(self, interface: str, operation: str) -> str | None:
        value = self._run(
            (
                "nmcli",
                "--get-values",
                "GENERAL.CON-UUID",
                "device",
                "show",
                interface,
            ),
            operation=operation,
        ).strip()
        if not value:
            return None
        if not UUID_RE.fullmatch(value):
            raise SwitchError(f"{operation} did not return one active connection")
        return value

    def _active_uuid(self, interface: str, operation: str) -> str:
        value = self._optional_active_uuid(interface, operation)
        if value is None:
            raise SwitchError(f"{operation} did not return one active connection")
        return value

    def _all_ipv4_addresses(self) -> tuple[AddressRecord, ...]:
        output = self._run(
            ("ip", "-o", "-4", "addr", "show"),
            operation="read all IPv4 address ownership",
        )
        records: list[AddressRecord] = []
        for line in output.splitlines():
            fields = line.split()
            if len(fields) < 4 or "inet" not in fields:
                raise SwitchError("could not parse global IPv4 address ownership")
            index = fields.index("inet")
            if index + 1 >= len(fields):
                raise SwitchError("could not parse global IPv4 address ownership")
            interface = fields[1].split("@", 1)[0]
            try:
                address = ipaddress.IPv4Interface(fields[index + 1])
            except ValueError:
                raise SwitchError("could not parse global IPv4 address ownership") from None
            records.append(AddressRecord(interface, address))
        return tuple(records)

    def _optional_docker0_link_local_ip(self) -> ipaddress.IPv6Address | None:
        docker_path = self.sys_class_net / "docker0"
        if self._interface_kind("docker0") == "missing":
            return None
        if (
            not self._interface_is_local_bridge("docker0")
            or not (docker_path / "bridge").is_dir()
        ):
            raise SwitchError("docker0 must be a local virtual bridge")
        output = self._run(
            ("ip", "-o", "-6", "addr", "show", "dev", "docker0", "scope", "link"),
            operation="read docker0 IPv6 link-local address",
        )
        lines = tuple(line for line in output.splitlines() if line.strip())
        if not lines:
            return None
        if len(lines) != 1:
            raise SwitchError("docker0 must have at most one IPv6 link-local /64 address")
        fields = lines[0].split()
        if (
            len(fields) < 6
            or not fields[0].endswith(":")
            or not fields[0][:-1].isdigit()
            or fields[1] != "docker0"
            or fields[2] != "inet6"
            or fields[4:6] != ["scope", "link"]
            or fields.count("inet6") != 1
            or fields.count("scope") != 1
        ):
            raise SwitchError("could not parse docker0 IPv6 link-local address")
        try:
            address = ipaddress.IPv6Interface(fields[3])
        except ValueError:
            raise SwitchError("could not parse docker0 IPv6 link-local address") from None
        if address.network != IPV6_LINK_LOCAL_NETWORK:
            raise SwitchError("docker0 must have one IPv6 link-local /64 address")
        return address.ip

    @staticmethod
    def _host_owners(records: Sequence[AddressRecord]) -> tuple[AddressRecord, ...]:
        return tuple(record for record in records if record.address.ip == HOST_IP)

    @staticmethod
    def _interface_addresses(
        records: Sequence[AddressRecord], interface: str
    ) -> tuple[ipaddress.IPv4Interface, ...]:
        return tuple(
            record.address for record in records if record.interface == interface
        )

    def _require_exact_host_owner(
        self,
        records: Sequence[AddressRecord],
        interface: str,
        context: str,
    ) -> None:
        owners = self._host_owners(records)
        expected = AddressRecord(interface, HOST_INTERFACE)
        if owners != (expected,):
            raise SwitchError(
                f"{context} requires one global 192.168.123.99/24 owner on {interface}"
            )
        if self._interface_addresses(records, interface) != (HOST_INTERFACE,):
            raise SwitchError(f"{context} requires exactly one IPv4 address on {interface}")

    def _validate_runtime_gate(self) -> None:
        expected = {
            "GO2_FORCE_NOMOTION_PROFILE": NOMOTION_PROFILE,
            "GO2_RUNTIME_PLACEMENT": NOMOTION_PLACEMENT,
            "GO2_ALLOW_MOTION": NOMOTION_MOTION_FLAG,
        }
        for name, value in expected.items():
            if self.environ.get(name) != value:
                raise SwitchError(
                    "exact corrected workstation no-motion runtime gate is required"
                )

    def _interface_kind(self, interface: str) -> str:
        path = self.sys_class_net / interface
        try:
            resolved = path.resolve(strict=True)
        except OSError:
            return "missing"
        if interface == "lo" or "/virtual/" in str(resolved):
            return "virtual"
        if (path / "wireless").is_dir():
            return "wireless"
        return "wired"

    def _interface_is_local_bridge(self, interface: str) -> bool:
        path = self.sys_class_net / interface
        return self._interface_kind(interface) == "virtual" and (
            path / "bridge"
        ).is_dir()

    def _validate_interfaces(self) -> None:
        if self._interface_kind(WIFI_INTERFACE) != "wireless":
            raise SwitchError("wlo1 must be a physical wireless interface")
        if self._interface_kind(WIRED_INTERFACE) != "wired":
            raise SwitchError("enp108s0 must be a physical wired interface")

    def _connection_uuids(self) -> tuple[str, ...]:
        output = self._run(
            ("nmcli", "--get-values", "UUID", "connection", "show"),
            operation="list NetworkManager connection identities",
        )
        values = tuple(line.strip() for line in output.splitlines() if line.strip())
        if not values or any(not UUID_RE.fullmatch(value) for value in values):
            raise SwitchError("NetworkManager returned an invalid connection list")
        return values

    def _find_target_uuid(self) -> str:
        matches: list[str] = []
        for uuid in self._connection_uuids():
            connection_type = self._nm_value(
                "connection.type", uuid, "read candidate connection type"
            )
            if connection_type != "802-11-wireless":
                continue
            ssid = self._nm_value(
                "802-11-wireless.ssid", uuid, "read candidate Wi-Fi SSID"
            )
            if ssid == TARGET_SSID:
                matches.append(uuid)
        if len(matches) != 1:
            raise SwitchError(
                "exactly one pre-created Robonix-Go2 Wi-Fi profile is required"
            )
        return matches[0]

    def _require_profile_values(
        self, uuid: str, expected: Mapping[str, str], label: str
    ) -> None:
        for field, wanted in expected.items():
            actual = self._nm_value(field, uuid, f"validate {label} profile")
            if actual != wanted:
                raise SwitchError(f"{label} profile has an unsafe {field} setting")

    @staticmethod
    def _isolated_ip_settings() -> dict[str, str]:
        return {
            "connection.autoconnect": "no",
            "connection.master": "",
            "connection.slave-type": "",
            "connection.secondaries": "",
            "ipv4.method": "manual",
            "ipv4.addresses": HOST_ADDRESS,
            "ipv4.gateway": "",
            "ipv4.dns": "",
            "ipv4.dns-search": "",
            "ipv4.dns-options": "",
            "ipv4.routes": "",
            "ipv4.route-table": "254",
            "ipv4.routing-rules": "",
            "ipv4.never-default": "yes",
            "ipv4.ignore-auto-dns": "yes",
            "ipv6.method": "disabled",
            "ipv6.addresses": "",
            "ipv6.gateway": "",
            "ipv6.dns": "",
            "ipv6.dns-search": "",
            "ipv6.dns-options": "",
            "ipv6.routes": "",
            "ipv6.route-table": "254",
            "ipv6.routing-rules": "",
            "ipv6.never-default": "yes",
            "ipv6.ignore-auto-dns": "yes",
        }

    def _validate_target_profile(self, uuid: str) -> None:
        self._require_profile_values(
            uuid,
            {
                **self._isolated_ip_settings(),
                "connection.type": "802-11-wireless",
                "connection.interface-name": WIFI_INTERFACE,
                "802-11-wireless.mode": "infrastructure",
                "802-11-wireless.ssid": TARGET_SSID,
                "802-11-wireless-security.key-mgmt": "wpa-psk",
            },
            "private Wi-Fi",
        )
        psk_flags = self._nm_value(
            "802-11-wireless-security.psk-flags",
            uuid,
            "validate private Wi-Fi one-time secret policy",
        )
        if re.fullmatch(r"2(?:\s+\([^\r\n()]*\))?", psk_flags) is None:
            raise SwitchError(
                "private Wi-Fi profile must use psk-flags=2 (not saved)"
            )

    def _validate_wired_profile(self, uuid: str) -> None:
        self._require_profile_values(
            uuid,
            {
                **self._isolated_ip_settings(),
                "connection.id": WIRED_PROFILE_NAME,
                "connection.type": "802-3-ethernet",
                "connection.interface-name": WIRED_INTERFACE,
            },
            "go2-readonly wired",
        )

    def _parse_route(self, line: str, family: int) -> RouteRecord:
        tokens = tuple(line.split())
        if not tokens:
            raise SwitchError("could not parse all-table route state")
        offset = 0
        route_type = "unicast"
        if tokens[0] in ROUTE_TYPES:
            route_type = tokens[0]
            offset = 1
        if offset >= len(tokens):
            raise SwitchError("could not parse all-table route state")
        destination = tokens[offset]
        if destination == "default":
            destination = "0.0.0.0/0" if family == 4 else "::/0"
        try:
            network = ipaddress.ip_network(destination, strict=False)
        except ValueError:
            raise SwitchError("could not parse all-table route state") from None
        if network.version != family:
            raise SwitchError("route family did not match parsed destination")

        def token_value(name: str) -> str | None:
            if name not in tokens:
                return None
            index = tokens.index(name)
            if index + 1 >= len(tokens):
                raise SwitchError("could not parse all-table route state")
            return tokens[index + 1]

        table = token_value("table") or "main"
        interface = token_value("dev")
        return RouteRecord(
            family=family,
            route_type=route_type,
            network=network,
            interface=interface,
            table=table,
            has_gateway="via" in tokens or "nexthop" in tokens,
            tokens=tokens,
        )

    def _all_routes(self, family: int) -> tuple[RouteRecord, ...]:
        output = self._run(
            ("ip", f"-{family}", "route", "show", "table", "all"),
            operation=f"read all IPv{family} route tables",
        )
        return tuple(
            self._parse_route(line, family)
            for line in output.splitlines()
            if line.strip()
        )

    def _validate_policy_rules(self, family: int) -> None:
        output = self._run(
            ("ip", f"-{family}", "rule", "show"),
            operation=f"read all IPv{family} policy rules",
        )
        normalized = [" ".join(line.split()) for line in output.splitlines() if line.strip()]
        ipv4_allowed = {
            "0: from all lookup local",
            "0: from all lookup 255",
            "32766: from all lookup main",
            "32766: from all lookup 254",
            "32767: from all lookup default",
            "32767: from all lookup 253",
        }
        ipv6_allowed = ipv4_allowed
        required_priorities = {"0", "32766", "32767"} if family == 4 else {"0", "32766"}
        seen: set[str] = set()
        for rule in normalized:
            if rule not in (ipv4_allowed if family == 4 else ipv6_allowed):
                raise SwitchError(f"private validation found an unsafe IPv{family} policy rule")
            seen.add(rule.split(":", 1)[0])
        expected_lengths = {3} if family == 4 else {2, 3}
        if len(normalized) not in expected_lengths or len(seen) != len(normalized):
            raise SwitchError(f"private validation found duplicate IPv{family} policy rules")
        if seen != required_priorities and not (
            family == 6 and seen == {"0", "32766", "32767"}
        ):
            raise SwitchError(f"private validation found an incomplete IPv{family} rule baseline")

    def _allowed_ipv6_route_key(
        self,
        route: RouteRecord,
        docker0_link_local_ip: ipaddress.IPv6Address | None,
    ) -> tuple[str, str] | None:
        if route.tokens == ALLOWED_IPV6_LOOPBACK_MAIN_TOKENS:
            return ("loopback-main", "lo")
        if route.tokens == ALLOWED_IPV6_LOOPBACK_LOCAL_TOKENS:
            return ("loopback-local", "lo")

        interface = route.interface
        if (
            docker0_link_local_ip is None
            or interface != "docker0"
            or not self._interface_is_local_bridge(interface)
            or not (self.sys_class_net / interface / "bridge").is_dir()
        ):
            return None
        bridge_main = (
            str(IPV6_LINK_LOCAL_NETWORK),
            "dev",
            interface,
            "proto",
            "kernel",
            "metric",
            "256",
            "linkdown",
            "pref",
            "medium",
        )
        bridge_local = (
            "local",
            str(route.network.network_address),
            "dev",
            interface,
            "table",
            "local",
            "proto",
            "kernel",
            "metric",
            "0",
            "pref",
            "medium",
        )
        bridge_multicast = (
            "multicast",
            str(IPV6_MULTICAST_NETWORK),
            "dev",
            interface,
            "table",
            "local",
            "proto",
            "kernel",
            "metric",
            "256",
            "linkdown",
            "pref",
            "medium",
        )
        if route.tokens == bridge_main:
            return ("bridge-main", interface)
        if (
            route.route_type == "local"
            and route.network.prefixlen == 128
            and route.network.subnet_of(IPV6_LINK_LOCAL_NETWORK)
            and route.network.network_address == docker0_link_local_ip
            and route.tokens == bridge_local
        ):
            return ("bridge-local", interface)
        if route.tokens == bridge_multicast:
            return ("bridge-multicast", interface)
        return None

    def _validate_route_isolation(self) -> None:
        direct_routes = 0
        allowed_ipv4ll_routes = 0
        seen_ipv6_routes: set[tuple[str, str]] = set()
        docker0_link_local_ip = self._optional_docker0_link_local_ip()
        for family in (4, 6):
            self._validate_policy_rules(family)
            for route in self._all_routes(family):
                if route.route_type == "unicast" and route.network.prefixlen <= 7:
                    raise SwitchError("private validation found a default or split-default route")
                if route.table not in {"main", "254", "local", "255"}:
                    raise SwitchError("private validation found a non-baseline route table")
                if route.has_gateway:
                    raise SwitchError("private validation found a gateway route")

                if family == 6:
                    key = self._allowed_ipv6_route_key(route, docker0_link_local_ip)
                    if key is None:
                        raise SwitchError("private validation found an unexpected IPv6 route")
                    if key in seen_ipv6_routes:
                        raise SwitchError("private validation found a duplicate IPv6 route")
                    seen_ipv6_routes.add(key)
                    continue

                if route.route_type not in {"unicast", "local", "broadcast"}:
                    raise SwitchError("private validation found an unexpected route type")

                is_allowed_ipv4ll = (
                    route.route_type == "unicast"
                    and route.table == "main"
                    and route.tokens == ALLOWED_WIFI_IPV4LL_ROUTE_TOKENS
                )
                if (
                    route.network.overlaps(IPV4LL_NETWORK)
                    and not is_allowed_ipv4ll
                ):
                    raise SwitchError("private validation found an unexpected IPv4LL route")

                if route.interface == WIFI_INTERFACE:
                    def value_after(name: str) -> str | None:
                        if name not in route.tokens:
                            return None
                        index = route.tokens.index(name)
                        if index + 1 >= len(route.tokens):
                            return None
                        return route.tokens[index + 1]

                    is_direct = (
                        route.route_type == "unicast"
                        and route.network == PRIVATE_NETWORK
                        and route.table in {"main", "254"}
                        and value_after("scope") == "link"
                        and value_after("src") == str(HOST_IP)
                        and value_after("proto") == "kernel"
                    )
                    is_local = (
                        route.route_type in {"local", "broadcast"}
                        and route.table in {"local", "255"}
                        and route.network.network_address
                        in {
                            PRIVATE_NETWORK.network_address,
                            HOST_IP,
                            PRIVATE_NETWORK.broadcast_address,
                        }
                    )
                    if is_direct:
                        direct_routes += 1
                    elif is_allowed_ipv4ll:
                        allowed_ipv4ll_routes += 1
                    elif not is_local:
                        raise SwitchError("private Wi-Fi has a route outside 192.168.123.0/24")
                    continue

                if route.interface == WIRED_INTERFACE:
                    raise SwitchError(
                        "wired Go2 interface retained a route during Wi-Fi validation"
                    )
                if route.route_type == "unicast":
                    if route.network.overlaps(PRIVATE_NETWORK):
                        raise SwitchError("another interface route overlaps 192.168.123.0/24")
                    if route.interface is None:
                        raise SwitchError("private validation found an unbound unicast route")
                    if not self._interface_is_local_bridge(route.interface):
                        raise SwitchError("private validation found another physical egress route")
                    if not route.network.is_private:
                        raise SwitchError("private validation found a non-private bridge route")
        if direct_routes != 1:
            raise SwitchError(
                "private validation requires one direct 192.168.123.0/24 route on wlo1"
            )
        if allowed_ipv4ll_routes > 1:
            raise SwitchError("private validation found duplicate allowed IPv4LL routes")
        required_ipv6_route_keys = REQUIRED_IPV6_LOOPBACK_ROUTE_KEYS
        if docker0_link_local_ip is not None:
            required_ipv6_route_keys |= REQUIRED_IPV6_DOCKER_ROUTE_KEYS
        if seen_ipv6_routes != required_ipv6_route_keys:
            raise SwitchError("private validation found an incomplete IPv6 local baseline")

    def preflight(self) -> ConnectionState:
        self._validate_runtime_gate()
        self._validate_interfaces()

        campus_uuid = self._active_uuid(
            WIFI_INTERFACE, "read active campus Wi-Fi identity"
        )
        campus_type = self._nm_value(
            "connection.type", campus_uuid, "validate active campus connection type"
        )
        campus_ssid = self._nm_value(
            "802-11-wireless.ssid", campus_uuid, "validate active campus Wi-Fi"
        )
        if campus_type != "802-11-wireless" or campus_ssid == TARGET_SSID:
            raise SwitchError("wlo1 must initially hold a non-Robonix campus Wi-Fi")
        campus_default = self._run(
            ("ip", "-4", "route", "show", "default", "dev", WIFI_INTERFACE),
            operation="validate active campus default route",
        ).strip()
        if not campus_default:
            raise SwitchError("active campus Wi-Fi must initially own an IPv4 default route")

        wired_uuid = self._active_uuid(
            WIRED_INTERFACE, "read active go2-readonly wired identity"
        )
        target_uuid = self._find_target_uuid()
        if target_uuid == campus_uuid:
            raise SwitchError("private Wi-Fi must not already be active")
        self._validate_target_profile(target_uuid)
        self._validate_wired_profile(wired_uuid)

        records = self._all_ipv4_addresses()
        self._require_exact_host_owner(records, WIRED_INTERFACE, "preflight")
        wifi_addresses = self._interface_addresses(records, WIFI_INTERFACE)
        if not wifi_addresses or any(address.ip == HOST_IP for address in wifi_addresses):
            raise SwitchError(
                "wlo1 must initially have a campus address and not 192.168.123.99"
            )

        self.state = ConnectionState(campus_uuid, target_uuid, wired_uuid)
        return self.state

    def _activate(self, uuid: str, interface: str, operation: str) -> None:
        self._run(
            (
                "nmcli",
                "--wait",
                str(int(ACTIVATION_TIMEOUT_S)),
                "connection",
                "up",
                "uuid",
                uuid,
                "ifname",
                interface,
            ),
            operation=operation,
            timeout=ACTIVATION_TIMEOUT_S + 3.0,
        )

    def _activate_target_interactive(self, uuid: str) -> None:
        self._run_interactive(
            (
                "nmcli",
                "--ask",
                "--wait",
                str(int(ACTIVATION_TIMEOUT_S)),
                "connection",
                "up",
                "uuid",
                uuid,
                "ifname",
                WIFI_INTERFACE,
            ),
            operation="interactively activate private Wi-Fi",
            timeout=ACTIVATION_TIMEOUT_S + 3.0,
        )

    def _deactivate(self, uuid: str, operation: str) -> None:
        self._run(
            (
                "nmcli",
                "--wait",
                str(int(ACTIVATION_TIMEOUT_S)),
                "connection",
                "down",
                "uuid",
                uuid,
            ),
            operation=operation,
            timeout=ACTIVATION_TIMEOUT_S + 3.0,
        )

    def _validate_private_state(self) -> None:
        assert self.state is not None
        if (
            self._active_uuid(WIFI_INTERFACE, "verify private Wi-Fi activation")
            != self.state.target_uuid
        ):
            raise SwitchError("private Wi-Fi did not become the active wlo1 profile")
        records = self._all_ipv4_addresses()
        self._require_exact_host_owner(records, WIFI_INTERFACE, "private runtime")
        if self._interface_addresses(records, WIRED_INTERFACE):
            raise SwitchError("wired interface retained an IPv4 address")
        self._validate_route_isolation()

        dns = self._run(
            ("nmcli", "--get-values", "IP4.DNS,IP6.DNS", "device", "show"),
            operation="verify absence of NetworkManager DNS",
        ).strip()
        if dns:
            raise SwitchError("private validation requires no NetworkManager DNS")

    def _validate_private_runtime(self) -> None:
        self._validate_private_state()
        for address in TARGET_HOSTS:
            self._run(
                (
                    "ping",
                    "-n",
                    "-I",
                    WIFI_INTERFACE,
                    "-c",
                    "3",
                    "-W",
                    "1",
                    address,
                ),
                operation=f"reachability check for private host {address}",
                timeout=PING_TIMEOUT_S,
            )

    def _campus_active_sample(self) -> bool:
        assert self.state is not None
        if (
            self._active_uuid(WIFI_INTERFACE, "observe campus Wi-Fi restoration")
            != self.state.campus_uuid
        ):
            return False
        return bool(
            self._run(
                ("ip", "-4", "route", "show", "default", "dev", WIFI_INTERFACE),
                operation="observe campus default route restoration",
            ).strip()
        )

    def _campus_without_host_sample(self) -> bool:
        return self._campus_active_sample() and not self._host_owners(
            self._all_ipv4_addresses()
        )

    def _fully_restored_sample(self) -> bool:
        assert self.state is not None
        if not self._campus_active_sample():
            return False
        if (
            self._active_uuid(WIRED_INTERFACE, "observe wired restoration")
            != self.state.wired_uuid
        ):
            return False
        records = self._all_ipv4_addresses()
        try:
            self._require_exact_host_owner(records, WIRED_INTERFACE, "rollback")
        except SwitchError:
            return False
        return True

    def _no_host_owner_sample(self) -> bool:
        return not self._host_owners(self._all_ipv4_addresses())

    def _wired_unique_sample(self) -> bool:
        assert self.state is not None
        try:
            if (
                self._optional_active_uuid(
                    WIFI_INTERFACE, "observe fallback private Wi-Fi cancellation"
                )
                == self.state.target_uuid
            ):
                return False
            if (
                self._active_uuid(WIRED_INTERFACE, "observe fallback wired restoration")
                != self.state.wired_uuid
            ):
                return False
            self._require_exact_host_owner(
                self._all_ipv4_addresses(), WIRED_INTERFACE, "fallback rollback"
            )
        except SwitchError:
            return False
        return True

    def _wait_for_stable(self, sample: Callable[[], bool]) -> bool:
        consecutive = 0
        for index in range(STABILITY_SAMPLE_LIMIT):
            try:
                valid = sample()
            except Exception:
                # Rollback observations are advisory.  Treat malformed output
                # or another ordinary runtime error as an unstable sample so a
                # later recovery round can still restore a safe topology.
                valid = False
            consecutive = consecutive + 1 if valid else 0
            if consecutive >= STABLE_OBSERVATIONS:
                return True
            if index + 1 < STABILITY_SAMPLE_LIMIT:
                self.sleep(STABILITY_INTERVAL_S)
        return False

    def _best_effort(self, action: Callable[[], None]) -> None:
        try:
            action()
        except Exception:
            # Every ordinary command/query failure must leave rollback running.
            # Process-control BaseExceptions remain outside this catch.
            pass

    def rollback(self) -> None:
        if not self.rollback_armed or self.rollback_started or self.state is None:
            return
        self.rollback_started = True
        restored = False
        wired_safe_fallback = False
        with self.signal_latch.deferred():
            for _attempt in range(ROLLBACK_ATTEMPTS):
                # The first operation always requests the original campus Wi-Fi.
                self._best_effort(
                    lambda: self._activate(
                        self.state.campus_uuid,
                        WIFI_INTERFACE,
                        "restore campus Wi-Fi",
                    )
                )
                # A timed-out activation client does not prove that NM stopped
                # the request.  Reconcile with an explicit target deactivation
                # followed by a newer campus activation request.
                self._best_effort(
                    lambda: self._deactivate(
                        self.state.target_uuid,
                        "cancel private Wi-Fi activation during rollback",
                    )
                )
                self._best_effort(
                    lambda: self._activate(
                        self.state.campus_uuid,
                        WIFI_INTERFACE,
                        "reassert campus Wi-Fi during rollback",
                    )
                )
                if not self._wait_for_stable(self._campus_without_host_sample):
                    continue

                self._best_effort(
                    lambda: self._activate(
                        self.state.wired_uuid,
                        WIRED_INTERFACE,
                        "restore go2-readonly wired profile",
                    )
                )
                if self._wait_for_stable(self._fully_restored_sample):
                    restored = True
                    break
                self._best_effort(
                    lambda: self._deactivate(
                        self.state.wired_uuid,
                        "remove unstable wired address during rollback",
                    )
                )

            if not restored:
                # Leave the campus request newer than any target request, and
                # remove wired .99 unless a stable, unique restoration was
                # proven.  This cannot guarantee recovery after SIGKILL/power
                # loss, but it never knowingly leaves duplicate host addresses.
                self._best_effort(
                    lambda: self._activate(
                        self.state.campus_uuid,
                        WIFI_INTERFACE,
                        "final campus Wi-Fi recovery attempt",
                    )
                )
                self._best_effort(
                    lambda: self._deactivate(
                        self.state.target_uuid,
                        "final private Wi-Fi cancellation",
                    )
                )
                self._best_effort(
                    lambda: self._activate(
                        self.state.campus_uuid,
                        WIFI_INTERFACE,
                        "final campus Wi-Fi reassertion",
                    )
                )
                if self._wait_for_stable(self._no_host_owner_sample):
                    self._best_effort(
                        lambda: self._activate(
                            self.state.wired_uuid,
                            WIRED_INTERFACE,
                            "fallback go2-readonly wired restoration",
                        )
                    )
                    wired_safe_fallback = self._wait_for_stable(
                        self._wired_unique_sample
                    )
                if not wired_safe_fallback:
                    self._best_effort(
                        lambda: self._deactivate(
                            self.state.wired_uuid,
                            "remove unverified wired address",
                        )
                    )
        self.rollback_armed = False
        if not restored:
            raise SwitchError(
                "rollback did not restore stable campus Wi-Fi; "
                + (
                    "wired go2-readonly was restored with unique address ownership"
                    if wired_safe_fallback
                    else "wired restoration was blocked to prevent duplicate addresses"
                )
            )

    def execute(self, hold_seconds: int, *, interactive_target: bool = False) -> None:
        if interactive_target:
            self.runner.require_interactive_terminal()
        self.preflight()
        assert self.state is not None
        self.rollback_armed = True
        primary_error: BaseException | None = None
        try:
            self._deactivate(
                self.state.wired_uuid,
                "temporarily disconnect go2-readonly wired profile",
            )
            if interactive_target:
                self._activate_target_interactive(self.state.target_uuid)
            else:
                self._activate(
                    self.state.target_uuid,
                    WIFI_INTERFACE,
                    "temporarily activate private Wi-Fi",
                )
            self._validate_private_runtime()
            deadline = self.monotonic() + hold_seconds
            while self.monotonic() < deadline:
                self.sleep(
                    min(
                        RUNTIME_RECHECK_INTERVAL_S,
                        max(0.0, deadline - self.monotonic()),
                    )
                )
                self.signal_latch.raise_if_pending()
                self._validate_private_state()
        except BaseException as exc:
            primary_error = exc
        try:
            self.rollback()
        except SwitchError as rollback_error:
            if primary_error is None:
                raise
            raise SwitchError(
                f"private validation failed and rollback was incomplete: {rollback_error}"
            ) from None
        if primary_error is not None:
            raise primary_error
        self.signal_latch.raise_if_pending()


def _positive_bounded_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("must be an integer") from None
    if not 0 <= parsed <= MAX_HOLD_SECONDS:
        raise argparse.ArgumentTypeError(
            f"must be between 0 and {MAX_HOLD_SECONDS} seconds"
        )
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preflight or temporarily validate Robonix-Go2 Wi-Fi with ordered rollback. "
            "No ROS or motion command is run."
        )
    )
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument(
        "--preflight",
        action="store_true",
        help="read configuration and current state only; make no network change",
    )
    modes.add_argument(
        "--execute-after-approval",
        action="store_true",
        help="change NetworkManager temporarily; requires fresh operator approval",
    )
    modes.add_argument(
        "--execute-after-approval-interactive",
        action="store_true",
        help=(
            "let nmcli read one unsaved Wi-Fi secret directly from a visible terminal; "
            "requires fresh operator approval"
        ),
    )
    parser.add_argument(
        "--hold-seconds",
        type=_positive_bounded_int,
        default=30,
        help=f"private validation hold, 0..{MAX_HOLD_SECONDS} seconds (default: 30)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.preflight and args.hold_seconds != 30:
        print(
            "error: --hold-seconds is valid only with an execute-after-approval mode",
            file=sys.stderr,
        )
        return 2

    signal_latch = SignalLatch()
    workflow = PrivateWifiRollback(
        CommandRunner(signal_latch),
        environ=os.environ,
        signal_latch=signal_latch,
    )

    def on_signal(signum: int, _frame: object) -> None:
        signal_latch.capture(signum)

    previous_handlers: dict[int, object] = {}
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signum] = signal.signal(signum, on_signal)

    try:
        if args.preflight:
            workflow.preflight()
            signal_latch.raise_if_pending()
            print("PASS: private Wi-Fi configuration and rollback prerequisites are valid")
            print("READ-ONLY: no NetworkManager connection was changed")
        else:
            print("Starting separately approved private Wi-Fi validation; rollback is armed")
            if args.execute_after_approval_interactive:
                print(
                    "nmcli will read the one-time Wi-Fi secret directly; "
                    "Python will not receive or capture it"
                )
            workflow.execute(
                args.hold_seconds,
                interactive_target=args.execute_after_approval_interactive,
            )
            print("PASS: private Wi-Fi reachability validated and original network restored")
        return 0
    except SwitchInterrupted as exc:
        try:
            workflow.rollback()
        except SwitchError as rollback_error:
            print(f"ERROR: signal rollback incomplete: {rollback_error}", file=sys.stderr)
            return 70
        print("INTERRUPTED: original network restored", file=sys.stderr)
        return 128 + exc.signum
    except SwitchError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
