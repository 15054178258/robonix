#!/usr/bin/env python3
"""Temporarily validate the router-backed Go2 Wi-Fi, then restore campus Wi-Fi.

The preflight mode is read-only.  The execute mode creates a temporary
NetworkManager clone, gives only that clone a router/default route, validates
the router, robot, lidar and direct Internet path, and always restores the
original campus connection.  The saved ``Robonix-Go2`` profile is never
modified.  ``nmcli --ask`` reads the one-time Wi-Fi secret directly from the
visible terminal; this process never reads, captures, prints or persists it.

This utility does not start ROS, Robonix or a motion process.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import ipaddress
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import time
from typing import Mapping, Sequence


TARGET_SSID = "Robonix-Go2"
WIFI_INTERFACE = "wlo1"
WIRED_INTERFACE = "enp108s0"
HOST_ADDRESS = "192.168.123.99/24"
HOST_IP = ipaddress.IPv4Interface(HOST_ADDRESS).ip
PRIVATE_NETWORK = ipaddress.IPv4Interface(HOST_ADDRESS).network
ROUTER_IP = "192.168.123.1"
TARGET_HOSTS = (ROUTER_IP, "192.168.123.18", "192.168.123.161")
DASHSCOPE_PROBE = "https://dashscope.aliyuncs.com/compatible-mode/v1/models"
EXPECTED_DASHSCOPE_STATUS = "401"
RUNTIME_GATE = {
    "GO2_FORCE_NOMOTION_PROFILE": "workstation-full-nomotion-corrected-v1",
    "GO2_RUNTIME_PLACEMENT": "workstation-local",
    "GO2_ALLOW_MOTION": "false",
}
UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)
QUERY_TIMEOUT_S = 8.0
ACTIVATION_TIMEOUT_S = 25.0
MAX_HOLD_SECONDS = 120
RECHECK_INTERVAL_S = 1.0
STACK_PORTS = frozenset({50051, 50107, 7860, 8091, 8092})


class CutoverError(RuntimeError):
    """A fixed, credential-free failure safe to show to the operator."""


class CutoverInterrupted(BaseException):
    """A handled signal requested immediate rollback at a command boundary."""

    def __init__(self, signum: int) -> None:
        self.signum = signum
        super().__init__(signum)


class SignalLatch:
    def __init__(self) -> None:
        self.pending: int | None = None
        self.defer_depth = 0

    def capture(self, signum: int) -> None:
        if self.pending is None:
            self.pending = signum

    def check(self) -> None:
        if self.pending is not None and self.defer_depth == 0:
            raise CutoverInterrupted(self.pending)

    @contextmanager
    def deferred(self):  # type: ignore[no-untyped-def]
        self.defer_depth += 1
        try:
            yield
        finally:
            self.defer_depth -= 1


class Runner:
    """Execute fixed argv without forwarding potentially sensitive stderr."""

    def __init__(self, latch: SignalLatch) -> None:
        self.latch = latch

    def run(
        self,
        argv: Sequence[str],
        *,
        operation: str,
        timeout: float = QUERY_TIMEOUT_S,
        environ: Mapping[str, str] | None = None,
    ) -> str:
        self.latch.check()
        try:
            result = subprocess.run(
                list(argv),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
                timeout=timeout,
                env=dict(environ) if environ is not None else None,
                start_new_session=True,
            )
        except subprocess.TimeoutExpired:
            raise CutoverError(f"{operation} timed out") from None
        except OSError:
            raise CutoverError(f"{operation} could not be started") from None
        self.latch.check()
        if result.returncode != 0:
            raise CutoverError(f"{operation} failed")
        return result.stdout

    def interactive(self, argv: Sequence[str], *, operation: str) -> None:
        self.latch.check()
        if not all(stream.isatty() for stream in (sys.stdin, sys.stdout, sys.stderr)):
            raise CutoverError("interactive activation requires a visible terminal")
        try:
            result = subprocess.run(
                list(argv),
                stdin=None,
                stdout=None,
                stderr=None,
                check=False,
                timeout=ACTIVATION_TIMEOUT_S + 3.0,
                start_new_session=False,
            )
        except subprocess.TimeoutExpired:
            raise CutoverError(f"{operation} timed out") from None
        except OSError:
            raise CutoverError(f"{operation} could not be started") from None
        self.latch.check()
        if result.returncode != 0:
            raise CutoverError(f"{operation} failed")


@dataclass(frozen=True)
class Baseline:
    campus_uuid: str
    source_uuid: str
    initial_uuids: frozenset[str]
    campus_addresses: tuple[str, ...]
    campus_default_routes: tuple[str, ...]
    campus_dns: str


class RouterGatewayAudit:
    def __init__(self, runner: Runner, latch: SignalLatch) -> None:
        self.runner = runner
        self.latch = latch
        self.baseline: Baseline | None = None
        self.audit_uuid: str | None = None
        self.audit_name: str | None = None
        self.rollback_armed = False

    def _run(
        self,
        argv: Sequence[str],
        operation: str,
        *,
        timeout: float = QUERY_TIMEOUT_S,
        environ: Mapping[str, str] | None = None,
    ) -> str:
        return self.runner.run(
            argv, operation=operation, timeout=timeout, environ=environ
        )

    def _nm_value(self, field: str, uuid: str, operation: str) -> str:
        return self._run(
            ("nmcli", "--get-values", field, "connection", "show", "uuid", uuid),
            operation,
        ).strip()

    def _active_uuid(self) -> str:
        value = self._run(
            (
                "nmcli",
                "--get-values",
                "GENERAL.CON-UUID",
                "device",
                "show",
                WIFI_INTERFACE,
            ),
            "read active Wi-Fi identity",
        ).strip()
        if not UUID_RE.fullmatch(value):
            raise CutoverError("wlo1 must have exactly one active Wi-Fi connection")
        return value

    def _connection_uuids(self) -> tuple[str, ...]:
        output = self._run(
            ("nmcli", "--get-values", "UUID", "connection", "show"),
            "list NetworkManager connection identities",
        )
        values = tuple(line.strip() for line in output.splitlines() if line.strip())
        if not values or any(not UUID_RE.fullmatch(value) for value in values):
            raise CutoverError("NetworkManager returned an invalid connection list")
        return values

    def _find_by_ssid(self, ssid: str) -> tuple[str, ...]:
        matches: list[str] = []
        for uuid in self._connection_uuids():
            if self._nm_value("connection.type", uuid, "read connection type") != "802-11-wireless":
                continue
            if self._nm_value("802-11-wireless.ssid", uuid, "read Wi-Fi SSID") == ssid:
                matches.append(uuid)
        return tuple(matches)

    @staticmethod
    def _carrier() -> str:
        try:
            return Path(f"/sys/class/net/{WIRED_INTERFACE}/carrier").read_text(
                encoding="ascii"
            ).strip()
        except OSError:
            return ""

    def _require_no_stack_listeners(self) -> None:
        output = self._run(("ss", "-H", "-lnt"), "inspect local TCP listeners")
        found: set[int] = set()
        for line in output.splitlines():
            fields = line.split()
            if len(fields) < 4:
                continue
            match = re.search(r":(\d+)$", fields[3])
            if match and int(match.group(1)) in STACK_PORTS:
                found.add(int(match.group(1)))
        if found:
            raise CutoverError("Robonix/UI listeners must be stopped before Wi-Fi audit")

    def _require_source_profile(self, uuid: str) -> None:
        expected = {
            "connection.type": "802-11-wireless",
            "connection.interface-name": WIFI_INTERFACE,
            "connection.autoconnect": "no",
            "connection.secondaries": "",
            "802-11-wireless.mode": "infrastructure",
            "802-11-wireless.ssid": TARGET_SSID,
            "802-11-wireless-security.key-mgmt": "wpa-psk",
            "proxy.method": "none",
            "ipv4.method": "manual",
            "ipv4.addresses": HOST_ADDRESS,
            "ipv4.gateway": "",
            "ipv4.dns": "",
            "ipv4.never-default": "yes",
            "ipv4.ignore-auto-dns": "yes",
            "ipv4.ignore-auto-routes": "no",
            "ipv4.route-table": "254",
            "ipv4.routes": "",
            "ipv4.routing-rules": "",
            "ipv6.method": "disabled",
            "ipv6.never-default": "yes",
        }
        for field, wanted in expected.items():
            if self._nm_value(field, uuid, "validate saved private Wi-Fi profile") != wanted:
                raise CutoverError(f"saved Robonix-Go2 profile has unexpected {field}")
        flags = self._nm_value(
            "802-11-wireless-security.psk-flags",
            uuid,
            "validate one-time Wi-Fi secret policy",
        )
        if re.fullmatch(r"2(?:\s+\([^\r\n()]*\))?", flags) is None:
            raise CutoverError("saved Robonix-Go2 profile must keep psk-flags=2")

    def _ipv4_addresses(self) -> tuple[tuple[str, ipaddress.IPv4Interface], ...]:
        output = self._run(("ip", "-o", "-4", "addr", "show"), "read IPv4 ownership")
        records: list[tuple[str, ipaddress.IPv4Interface]] = []
        for line in output.splitlines():
            fields = line.split()
            if "inet" not in fields or len(fields) < 4:
                raise CutoverError("could not parse IPv4 ownership")
            index = fields.index("inet")
            try:
                address = ipaddress.IPv4Interface(fields[index + 1])
            except (IndexError, ValueError):
                raise CutoverError("could not parse IPv4 ownership") from None
            records.append((fields[1].split("@", 1)[0], address))
        return tuple(records)

    def _host_owners(self) -> tuple[tuple[str, ipaddress.IPv4Interface], ...]:
        return tuple(
            record for record in self._ipv4_addresses() if record[1].ip == HOST_IP
        )

    def preflight(self) -> Baseline:
        for name, expected in RUNTIME_GATE.items():
            if os.environ.get(name) != expected:
                raise CutoverError("exact corrected workstation no-motion runtime gate is required")
        if self._carrier() != "0":
            raise CutoverError("enp108s0 must have no physical carrier for wireless audit")
        self._require_no_stack_listeners()

        campus_uuid = self._active_uuid()
        campus_ssid = self._nm_value(
            "802-11-wireless.ssid", campus_uuid, "validate active campus Wi-Fi"
        )
        if campus_ssid == TARGET_SSID:
            raise CutoverError("wlo1 must initially be on the campus Wi-Fi")
        campus_default_routes = tuple(
            " ".join(line.split())
            for line in self._run(
                ("ip", "-4", "route", "show", "default"),
                "validate campus default route",
            ).splitlines()
            if line.strip()
        )
        if (
            len(campus_default_routes) != 1
            or f" dev {WIFI_INTERFACE} " not in f" {campus_default_routes[0]} "
        ):
            raise CutoverError("campus Wi-Fi must initially own the only IPv4 default route")
        campus_addresses = tuple(
            str(address)
            for interface, address in self._ipv4_addresses()
            if interface == WIFI_INTERFACE
        )
        if not campus_addresses:
            raise CutoverError("campus Wi-Fi must initially own an IPv4 address")
        campus_dns = self._run(
            (
                "nmcli",
                "--get-values",
                "IP4.DNS,IP6.DNS",
                "device",
                "show",
                WIFI_INTERFACE,
            ),
            "snapshot campus DNS",
        ).strip()
        if not campus_dns:
            raise CutoverError("campus Wi-Fi must initially expose NetworkManager DNS")

        initial_uuids = frozenset(self._connection_uuids())
        sources = self._find_by_ssid(TARGET_SSID)
        if len(sources) != 1:
            raise CutoverError("exactly one saved Robonix-Go2 profile is required")
        source_uuid = sources[0]
        if source_uuid == campus_uuid:
            raise CutoverError("saved Robonix-Go2 profile must not already be active")
        self._require_source_profile(source_uuid)
        if self._host_owners():
            raise CutoverError("192.168.123.99 must be unowned before wireless audit")
        self.baseline = Baseline(
            campus_uuid=campus_uuid,
            source_uuid=source_uuid,
            initial_uuids=initial_uuids,
            campus_addresses=campus_addresses,
            campus_default_routes=campus_default_routes,
            campus_dns=campus_dns,
        )
        return self.baseline

    def _recover_audit_uuid(self) -> str | None:
        assert self.baseline is not None
        if self.audit_name is None:
            return None
        current = frozenset(self._connection_uuids())
        if self.audit_uuid is not None:
            if self.audit_uuid in current:
                return self.audit_uuid
            self.audit_uuid = None
        matches = tuple(
            uuid
            for uuid in current - self.baseline.initial_uuids
            if self._nm_value(
                "connection.id", uuid, "recover temporary audit identity"
            )
            == self.audit_name
        )
        if len(matches) > 1:
            raise CutoverError("multiple temporary Wi-Fi audit profiles require cleanup")
        if matches:
            self.audit_uuid = matches[0]
        return self.audit_uuid

    def _audit_profile_is_nonpersistent(self, uuid: str) -> bool:
        output = self._run(
            ("nmcli", "--terse", "--fields", "UUID,FILENAME", "connection", "show"),
            "inspect temporary audit persistence",
        )
        matches: list[str] = []
        for line in output.splitlines():
            if ":" not in line:
                continue
            candidate_uuid, filename = line.split(":", 1)
            if candidate_uuid == uuid:
                matches.append(filename)
        return len(matches) == 1 and (
            matches[0] == ""
            or matches[0].startswith("/run/NetworkManager/system-connections/")
        )

    def _create_audit_profile(self) -> str:
        assert self.baseline is not None
        audit_name = f"Robonix-Go2-gateway-audit-{os.getpid()}-{secrets.token_hex(4)}"
        self.audit_name = audit_name
        before_uuids = frozenset(self._connection_uuids())
        if before_uuids != self.baseline.initial_uuids:
            raise CutoverError("NetworkManager connection set changed after preflight")
        self._run(
            (
                "nmcli",
                "--wait",
                "10",
                "connection",
                "clone",
                "--temporary",
                "uuid",
                self.baseline.source_uuid,
                audit_name,
            ),
            "create temporary router Wi-Fi audit profile",
            timeout=13.0,
        )
        after_uuids = frozenset(self._connection_uuids())
        new_uuids = after_uuids - before_uuids
        audit_matches = tuple(
            uuid
            for uuid in new_uuids
            if self._nm_value(
                "connection.id", uuid, "locate temporary audit identity"
            )
            == audit_name
        )
        if len(audit_matches) == 1:
            # Record the clone before rejecting any concurrent unrelated
            # NetworkManager change so rollback can still delete our profile.
            self.audit_uuid = audit_matches[0]
        if len(new_uuids) != 1 or len(audit_matches) != 1:
            raise CutoverError("temporary Wi-Fi audit profile was not created uniquely")
        self._run(
            (
                "nmcli",
                "connection",
                "modify",
                "--temporary",
                "uuid",
                self.audit_uuid,
                "connection.autoconnect",
                "no",
                "connection.interface-name",
                WIFI_INTERFACE,
                "connection.secondaries",
                "",
                "proxy.method",
                "none",
                "ipv4.method",
                "manual",
                "ipv4.addresses",
                HOST_ADDRESS,
                "ipv4.gateway",
                ROUTER_IP,
                "ipv4.never-default",
                "no",
                "ipv4.dns",
                ROUTER_IP,
                "ipv4.ignore-auto-dns",
                "yes",
                "ipv4.ignore-auto-routes",
                "yes",
                "ipv4.route-table",
                "254",
                "ipv4.routes",
                "",
                "ipv4.routing-rules",
                "",
                "ipv6.method",
                "disabled",
                "ipv6.never-default",
                "yes",
            ),
            "configure temporary router Wi-Fi audit profile",
        )
        if not self._audit_profile_is_nonpersistent(self.audit_uuid):
            raise CutoverError("temporary Wi-Fi audit profile became persistent")
        flags = self._nm_value(
            "802-11-wireless-security.psk-flags",
            self.audit_uuid,
            "validate temporary one-time Wi-Fi secret policy",
        )
        if re.fullmatch(r"2(?:\s+\([^\r\n()]*\))?", flags) is None:
            raise CutoverError("temporary Wi-Fi profile must keep psk-flags=2")
        return self.audit_uuid

    def _activate_audit_profile(self) -> None:
        assert self.audit_uuid is not None
        self.runner.interactive(
            (
                "nmcli",
                "--ask",
                "--wait",
                str(int(ACTIVATION_TIMEOUT_S)),
                "connection",
                "up",
                "uuid",
                self.audit_uuid,
                "ifname",
                WIFI_INTERFACE,
            ),
            operation="activate temporary router Wi-Fi audit profile",
        )

    def _validate_runtime_state(self) -> None:
        assert self.audit_uuid is not None
        if self._active_uuid() != self.audit_uuid:
            raise CutoverError("temporary router Wi-Fi is not the active wlo1 profile")
        owners = self._host_owners()
        expected_owner = (WIFI_INTERFACE, ipaddress.IPv4Interface(HOST_ADDRESS))
        if owners != (expected_owner,):
            raise CutoverError("wlo1 must uniquely own 192.168.123.99/24")
        if any(interface == WIRED_INTERFACE for interface, _address in self._ipv4_addresses()):
            raise CutoverError("enp108s0 retained an IPv4 address during wireless audit")
        global_ipv6 = self._run(
            ("ip", "-o", "-6", "addr", "show", "dev", WIFI_INTERFACE, "scope", "global"),
            "verify absence of global Wi-Fi IPv6",
        ).strip()
        if global_ipv6:
            raise CutoverError("wlo1 retained a global IPv6 address during wireless audit")
        dns = self._run(
            (
                "nmcli",
                "--get-values",
                "IP4.DNS,IP6.DNS",
                "device",
                "show",
                WIFI_INTERFACE,
            ),
            "validate router Wi-Fi DNS",
        ).strip()
        dns_values = tuple(
            value for value in re.split(r"[\s|]+", dns) if value
        )
        if dns_values != (ROUTER_IP,):
            raise CutoverError("router Wi-Fi DNS must be exactly 192.168.123.1")
        routes = tuple(
            line.split()
            for line in self._run(
                ("ip", "-4", "route", "show", "default"),
                "validate router default route",
            ).splitlines()
            if line.strip()
        )
        if len(routes) != 1:
            raise CutoverError("wireless audit requires exactly one IPv4 default route")
        tokens = routes[0]
        if not (
            len(tokens) >= 5
            and tokens[0] == "default"
            and tokens[1:5] == ["via", ROUTER_IP, "dev", WIFI_INTERFACE]
        ):
            raise CutoverError("IPv4 default route must use 192.168.123.1 on wlo1")
        route_get = self._run(
            ("ip", "-4", "route", "get", "1.1.1.1"),
            "validate public route selection",
        ).split()
        required = (("via", ROUTER_IP), ("dev", WIFI_INTERFACE), ("src", str(HOST_IP)))
        for key, value in required:
            if key not in route_get or route_get.index(key) + 1 >= len(route_get):
                raise CutoverError("public route selection is incomplete")
            if route_get[route_get.index(key) + 1] != value:
                raise CutoverError("public route selection does not use the router Wi-Fi")

    def _validate_reachability(self) -> None:
        self._validate_runtime_state()
        for host in TARGET_HOSTS:
            ping_env = dict(os.environ)
            ping_env["LC_ALL"] = "C"
            warmed = False
            for _attempt in range(3):
                try:
                    self._run(
                        (
                            "ping",
                            "-n",
                            "-I",
                            WIFI_INTERFACE,
                            "-c",
                            "1",
                            "-W",
                            "1",
                            host,
                        ),
                        f"neighbor warm-up for {host}",
                        timeout=3.0,
                        environ=ping_env,
                    )
                    warmed = True
                    break
                except CutoverError:
                    continue
            if not warmed:
                raise CutoverError(f"neighbor warm-up for {host} failed")
            output = self._run(
                ("ping", "-n", "-I", WIFI_INTERFACE, "-c", "3", "-W", "1", host),
                f"reachability check for {host}",
                timeout=6.0,
                environ=ping_env,
            )
            if not re.search(
                r"3 packets transmitted, 3 received, (?:\+?0(?:\.0+)?)% packet loss",
                output,
            ):
                raise CutoverError(f"reachability check for {host} had packet loss")
        direct_env = dict(os.environ)
        for name in (
            "http_proxy",
            "https_proxy",
            "HTTP_PROXY",
            "HTTPS_PROXY",
            "ALL_PROXY",
            "all_proxy",
            "NO_PROXY",
            "no_proxy",
        ):
            direct_env.pop(name, None)
        metadata = self._run(
            (
                "/usr/bin/curl",
                "--disable",
                "--ipv4",
                "--interface",
                str(HOST_IP),
                "--proxy",
                "",
                "--noproxy",
                "*",
                "--connect-timeout",
                "5",
                "--max-time",
                "12",
                "--silent",
                "--show-error",
                "--output",
                "/dev/null",
                "--write-out",
                "%{http_code}\t%{local_ip}\t%{remote_ip}\t"
                "%{ssl_verify_result}\t%{num_redirects}\t%{url_effective}",
                DASHSCOPE_PROBE,
            ),
            "direct DashScope Internet probe",
            timeout=15.0,
            environ=direct_env,
        ).strip().split("\t")
        if len(metadata) != 6:
            raise CutoverError("direct DashScope probe returned incomplete metadata")
        status, local_ip, remote_ip_text, tls_result, redirects, effective_url = metadata
        try:
            remote_ip = ipaddress.IPv4Address(remote_ip_text)
        except ValueError:
            raise CutoverError("direct DashScope probe returned an invalid remote IPv4") from None
        if not (
            status == EXPECTED_DASHSCOPE_STATUS
            and local_ip == str(HOST_IP)
            and tls_result == "0"
            and redirects == "0"
            and effective_url == DASHSCOPE_PROBE
            and remote_ip.is_global
        ):
            raise CutoverError("direct DashScope probe did not prove a clean HTTP 401 path")
        route_get = self._run(
            (
                "ip",
                "-4",
                "route",
                "get",
                str(remote_ip),
                "from",
                str(HOST_IP),
                "oif",
                WIFI_INTERFACE,
            ),
            "validate actual DashScope route selection",
        ).split()
        for key, value in (
            ("via", ROUTER_IP),
            ("dev", WIFI_INTERFACE),
            ("src", str(HOST_IP)),
        ):
            if key not in route_get or route_get.index(key) + 1 >= len(route_get):
                raise CutoverError("actual DashScope route selection is incomplete")
            if route_get[route_get.index(key) + 1] != value:
                raise CutoverError("actual DashScope route did not use router Wi-Fi")

    def _activate_campus(self) -> None:
        assert self.baseline is not None
        self._run(
            (
                "nmcli",
                "--wait",
                str(int(ACTIVATION_TIMEOUT_S)),
                "connection",
                "up",
                "uuid",
                self.baseline.campus_uuid,
                "ifname",
                WIFI_INTERFACE,
            ),
            "restore campus Wi-Fi",
            timeout=ACTIVATION_TIMEOUT_S + 3.0,
        )

    def _audit_candidates(self) -> tuple[str, ...]:
        assert self.baseline is not None
        if self.audit_name is None:
            return ()
        current = frozenset(self._connection_uuids())
        return tuple(
            uuid
            for uuid in current - self.baseline.initial_uuids
            if self._nm_value(
                "connection.id", uuid, "inspect temporary audit cleanup"
            )
            == self.audit_name
        )

    def _campus_restored_sample(self) -> bool:
        assert self.baseline is not None
        if self._active_uuid() != self.baseline.campus_uuid:
            return False
        addresses = tuple(
            str(address)
            for interface, address in self._ipv4_addresses()
            if interface == WIFI_INTERFACE
        )
        if addresses != self.baseline.campus_addresses or self._host_owners():
            return False
        default_routes = tuple(
            " ".join(line.split())
            for line in self._run(
                ("ip", "-4", "route", "show", "default"),
                "observe campus default route restoration",
            ).splitlines()
            if line.strip()
        )
        if default_routes != self.baseline.campus_default_routes:
            return False
        dns = self._run(
            (
                "nmcli",
                "--get-values",
                "IP4.DNS,IP6.DNS",
                "device",
                "show",
                WIFI_INTERFACE,
            ),
            "observe campus DNS restoration",
        ).strip()
        if dns != self.baseline.campus_dns or self._audit_candidates():
            return False
        self._require_source_profile(self.baseline.source_uuid)
        return True

    def _wait_for_campus_stability(self) -> bool:
        consecutive = 0
        for index in range(12):
            try:
                valid = self._campus_restored_sample()
            except CutoverError:
                valid = False
            consecutive = consecutive + 1 if valid else 0
            if consecutive >= 4:
                return True
            if index + 1 < 12:
                time.sleep(0.25)
        return False

    def rollback(self) -> None:
        if not self.rollback_armed or self.baseline is None:
            return
        errors: list[str] = []
        with self.latch.deferred():
            for _attempt in range(3):
                try:
                    self._activate_campus()
                except CutoverError:
                    errors.append("campus activation")
                try:
                    recovered = self._recover_audit_uuid()
                except CutoverError:
                    recovered = None
                    errors.append("recover temporary router Wi-Fi identity")
                if recovered is not None:
                    deleting_uuid = recovered
                    try:
                        if self._active_uuid() == deleting_uuid:
                            self._run(
                                (
                                    "nmcli",
                                    "--wait",
                                    "10",
                                    "connection",
                                    "down",
                                    "uuid",
                                    deleting_uuid,
                                ),
                                "deactivate temporary router Wi-Fi",
                                timeout=13.0,
                            )
                    except CutoverError:
                        errors.append("deactivate temporary router Wi-Fi")
                    try:
                        self._activate_campus()
                    except CutoverError:
                        errors.append("campus reactivation before cleanup")
                    try:
                        self._run(
                            (
                                "nmcli",
                                "--wait",
                                "10",
                                "connection",
                                "delete",
                                "uuid",
                                deleting_uuid,
                            ),
                            "delete temporary router Wi-Fi profile",
                            timeout=13.0,
                        )
                        if deleting_uuid in self._connection_uuids() or self._audit_candidates():
                            raise CutoverError("temporary profile still exists after deletion")
                        self.audit_uuid = None
                    except CutoverError:
                        errors.append("delete temporary router Wi-Fi profile")
                try:
                    self._activate_campus()
                    if not self._wait_for_campus_stability():
                        raise CutoverError("campus restoration did not become stable")
                    self.rollback_armed = False
                    return
                except CutoverError:
                    errors.append("campus restoration verification")
                time.sleep(0.5)
        raise CutoverError(
            "rollback did not prove stable campus Wi-Fi restoration: "
            + ", ".join(dict.fromkeys(errors))
        )

    def execute(self, hold_seconds: int) -> None:
        self.preflight()
        self.rollback_armed = True
        primary: BaseException | None = None
        try:
            self._create_audit_profile()
            self._activate_audit_profile()
            self._validate_reachability()
            deadline = time.monotonic() + hold_seconds
            while time.monotonic() < deadline:
                time.sleep(min(RECHECK_INTERVAL_S, deadline - time.monotonic()))
                self.latch.check()
                self._validate_runtime_state()
            self._validate_reachability()
        except BaseException as exc:
            primary = exc
        try:
            self.rollback()
        except CutoverError as rollback_error:
            raise CutoverError(
                f"wireless audit failed and rollback was incomplete: {rollback_error}"
            ) from None
        if primary is not None:
            raise primary
        self.latch.check()


def bounded_hold(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("must be an integer") from None
    if parsed < 0 or parsed > MAX_HOLD_SECONDS:
        raise argparse.ArgumentTypeError(f"must be between 0 and {MAX_HOLD_SECONDS}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate router-backed Go2 Wi-Fi and always restore campus Wi-Fi."
    )
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--preflight", action="store_true")
    modes.add_argument("--execute-after-approval-interactive", action="store_true")
    parser.add_argument("--hold-seconds", type=bounded_hold, default=30)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.preflight and args.hold_seconds != 30:
        print("error: --hold-seconds applies only to execute mode", file=sys.stderr)
        return 2
    latch = SignalLatch()
    workflow = RouterGatewayAudit(Runner(latch), latch)
    lock_handle = Path(__file__).resolve().open("rb")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_handle.close()
        print("ERROR: another router Wi-Fi audit is already running", file=sys.stderr)
        return 1

    def capture(signum: int, _frame: object) -> None:
        latch.capture(signum)

    old_handlers: dict[int, object] = {}
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        old_handlers[signum] = signal.signal(signum, capture)
    try:
        if args.preflight:
            workflow.preflight()
            print("PASS: router Wi-Fi temporary-audit prerequisites are valid")
            print("READ-ONLY: no NetworkManager connection was changed")
        else:
            print("Starting separately approved router Wi-Fi audit; rollback is armed")
            print("nmcli reads the one-time Wi-Fi secret directly; Python never receives it")
            workflow.execute(args.hold_seconds)
            print("PASS: router, Go2, lidar and direct Internet validated")
            print("RESTORED: original campus Wi-Fi is active; saved profile was untouched")
        return 0
    except CutoverInterrupted as exc:
        try:
            workflow.rollback()
        except CutoverError as rollback_error:
            print(f"ERROR: signal rollback incomplete: {rollback_error}", file=sys.stderr)
            return 70
        print("INTERRUPTED: original campus Wi-Fi restored", file=sys.stderr)
        return 128 + exc.signum
    except CutoverError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        lock_handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
