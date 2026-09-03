from __future__ import annotations

import ast
from contextlib import redirect_stderr
import importlib.util
import io
import ipaddress
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "nomotion_private_wifi_rollback.py"
SPEC = importlib.util.spec_from_file_location("nomotion_private_wifi_rollback", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


CAMPUS_UUID = "11111111-1111-4111-8111-111111111111"
TARGET_UUID = "22222222-2222-4222-8222-222222222222"
WIRED_UUID = "33333333-3333-4333-8333-333333333333"
TARGET_IPV6_ROUTES = (
    "::1 dev lo proto kernel metric 256 pref medium",
    "fe80::/64 dev docker0 proto kernel metric 256 linkdown pref medium",
    "local ::1 dev lo table local proto kernel metric 0 pref medium",
    "local fe80::454:87ff:fe24:1966 dev docker0 table local "
    "proto kernel metric 0 pref medium",
    "multicast ff00::/8 dev docker0 table local proto kernel "
    "metric 256 linkdown pref medium",
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0
        self.on_sleep = None

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds
        if self.on_sleep is not None:
            self.on_sleep()


class FakeRunner:
    def __init__(self, signal_latch: object) -> None:
        isolated = {
            "connection.autoconnect": "no",
            "connection.master": "",
            "connection.slave-type": "",
            "connection.secondaries": "",
            "ipv4.method": "manual",
            "ipv4.addresses": MODULE.HOST_ADDRESS,
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
        self.profiles = {
            CAMPUS_UUID: {
                "connection.id": "campus-profile",
                "connection.type": "802-11-wireless",
                "connection.interface-name": MODULE.WIFI_INTERFACE,
                "802-11-wireless.ssid": "NEU",
            },
            TARGET_UUID: {
                **isolated,
                "connection.id": "private-profile",
                "connection.type": "802-11-wireless",
                "connection.interface-name": MODULE.WIFI_INTERFACE,
                "802-11-wireless.mode": "infrastructure",
                "802-11-wireless.ssid": MODULE.TARGET_SSID,
                "802-11-wireless-security.key-mgmt": "wpa-psk",
                "802-11-wireless-security.psk-flags": "2 (not saved)",
            },
            WIRED_UUID: {
                **isolated,
                "connection.id": MODULE.WIRED_PROFILE_NAME,
                "connection.type": "802-3-ethernet",
                "connection.interface-name": MODULE.WIRED_INTERFACE,
            },
        }
        self.active = {
            MODULE.WIFI_INTERFACE: CAMPUS_UUID,
            MODULE.WIRED_INTERFACE: WIRED_UUID,
        }
        self.addresses = {
            MODULE.WIFI_INTERFACE: ("172.22.123.232/16",),
            MODULE.WIRED_INTERFACE: (MODULE.HOST_ADDRESS,),
        }
        self.signal_latch = signal_latch
        self.calls: list[tuple[tuple[str, ...], str]] = []
        self.fail_once: dict[str, BaseException] = {}
        self.ignore_activation: set[str] = set()
        self.capture_signal_on_operation: str | None = None
        self.late_target_timeout = False
        self.late_target_countdown: int | None = None
        self.late_target_fired = False
        self.late_target_on_wired_restore = False
        self.runtime_third_owner: str | None = None
        self.target_address = MODULE.HOST_ADDRESS
        self.extra_routes_v4: list[str] = []
        self.extra_routes_v6: list[str] = []
        self.omitted_routes_v6: set[str] = set()
        self.docker0_link_address_lines = [
            "5: docker0 inet6 fe80::454:87ff:fe24:1966/64 scope link "
            "valid_lft forever preferred_lft forever"
        ]
        self.extra_rules_v4: list[str] = []
        self.extra_rules_v6: list[str] = []
        self.interactive_terminal_ready = True
        self.interactive_timeout = False

    def require_interactive_terminal(self) -> None:
        if not self.interactive_terminal_ready:
            raise MODULE.SwitchError("simulated non-interactive terminal")

    def run_interactive(self, argv, *, timeout, operation):  # noqa: ANN001, ANN201
        del timeout
        command = tuple(argv)
        self.calls.append((command, operation))
        self.assert_interactive_command(command)
        if operation in self.fail_once:
            raise self.fail_once.pop(operation)
        if self.interactive_timeout:
            self.interactive_timeout = False
            self.late_target_countdown = 3
            raise MODULE.CommandTimedOut("simulated interactive activation timeout")
        self._activate_state(TARGET_UUID, MODULE.WIFI_INTERFACE)
        self._capture_after_boundary(operation)

    @staticmethod
    def assert_interactive_command(command: tuple[str, ...]) -> None:
        expected = (
            "nmcli",
            "--ask",
            "--wait",
            "25",
            "connection",
            "up",
            "uuid",
            TARGET_UUID,
            "ifname",
            MODULE.WIFI_INTERFACE,
        )
        if command != expected:
            raise AssertionError(f"unexpected interactive command: {command!r}")

    def _activate_state(self, uuid: str, interface: str) -> None:
        self.active[interface] = uuid
        if uuid == TARGET_UUID:
            self.addresses[interface] = (self.target_address,)
            if self.runtime_third_owner is not None:
                self.addresses["tun-test"] = (self.runtime_third_owner,)
        elif uuid == CAMPUS_UUID:
            self.addresses[interface] = ("172.22.123.232/16",)
        elif uuid == WIRED_UUID:
            self.addresses[interface] = (MODULE.HOST_ADDRESS,)

    def _deactivate_state(self, uuid: str) -> None:
        for interface, active_uuid in tuple(self.active.items()):
            if active_uuid == uuid:
                del self.active[interface]
                self.addresses[interface] = ()

    def _tick_late_target(self) -> None:
        if self.late_target_countdown is None:
            return
        self.late_target_countdown -= 1
        if self.late_target_countdown <= 0:
            self.late_target_countdown = None
            self.late_target_fired = True
            self._activate_state(TARGET_UUID, MODULE.WIFI_INTERFACE)

    def _capture_after_boundary(self, operation: str) -> None:
        if operation == self.capture_signal_on_operation:
            self.capture_signal_on_operation = None
            self.signal_latch.capture(signal.SIGTERM)

    def _all_address_output(self) -> str:
        index = 2
        lines: list[str] = []
        for interface, addresses in self.addresses.items():
            for address in addresses:
                lines.append(
                    f"{index}: {interface}    inet {address} scope global {interface}"
                )
                index += 1
        return "\n".join(lines) + ("\n" if lines else "")

    def _private_routes_v4(self) -> str:
        lines: list[str] = []
        if self.active.get(MODULE.WIFI_INTERFACE) == TARGET_UUID:
            lines.extend(
                (
                    "192.168.123.0/24 dev wlo1 proto kernel scope link src 192.168.123.99",
                    "local 192.168.123.99 dev wlo1 table local proto kernel "
                    "scope host src 192.168.123.99",
                    "broadcast 192.168.123.0 dev wlo1 table local proto kernel "
                    "scope link src 192.168.123.99",
                    "broadcast 192.168.123.255 dev wlo1 table local proto kernel "
                    "scope link src 192.168.123.99",
                )
            )
        lines.extend(self.extra_routes_v4)
        return "\n".join(lines) + ("\n" if lines else "")

    def _private_routes_v6(self) -> str:
        lines: list[str] = []
        if self.active.get(MODULE.WIFI_INTERFACE) == TARGET_UUID:
            lines.extend(
                route
                for route in TARGET_IPV6_ROUTES
                if route not in self.omitted_routes_v6
            )
        lines.extend(self.extra_routes_v6)
        return "\n".join(lines) + ("\n" if lines else "")

    def run(self, argv, *, timeout, operation):  # noqa: ANN001, ANN201
        del timeout
        command = tuple(argv)
        self.calls.append((command, operation))
        if operation in self.fail_once:
            raise self.fail_once.pop(operation)

        if command[:3] == ("nmcli", "--get-values", "GENERAL.CON-UUID"):
            if command[-1] == MODULE.WIFI_INTERFACE:
                self._tick_late_target()
            return self.active.get(command[-1], "") + "\n"

        if command[:3] == ("nmcli", "--get-values", "UUID"):
            return "\n".join(self.profiles) + "\n"

        if (
            command[:2] == ("nmcli", "--get-values")
            and command[3:6] == ("connection", "show", "uuid")
        ):
            return self.profiles[command[6]].get(command[2], "") + "\n"

        if command[:3] == ("nmcli", "--get-values", "IP4.DNS,IP6.DNS"):
            return (
                "172.22.0.1\n"
                if self.active.get(MODULE.WIFI_INTERFACE) == CAMPUS_UUID
                else ""
            )

        if command[:4] == ("nmcli", "--wait", "25", "connection"):
            action = command[4]
            uuid = command[6]
            if action == "down":
                self._deactivate_state(uuid)
                self._capture_after_boundary(operation)
                return ""
            if action == "up":
                interface = command[8]
                if (
                    uuid == TARGET_UUID
                    and operation == "temporarily activate private Wi-Fi"
                    and self.late_target_timeout
                ):
                    self.late_target_timeout = False
                    self.late_target_countdown = 3
                    raise MODULE.CommandTimedOut("simulated activation client timeout")
                if uuid not in self.ignore_activation:
                    self._activate_state(uuid, interface)
                if (
                    uuid == WIRED_UUID
                    and operation == "restore go2-readonly wired profile"
                    and self.late_target_on_wired_restore
                ):
                    self.late_target_on_wired_restore = False
                    self.late_target_countdown = 2
                self._capture_after_boundary(operation)
                return ""

        if command == ("ip", "-o", "-4", "addr", "show"):
            return self._all_address_output()

        if command == (
            "ip",
            "-o",
            "-6",
            "addr",
            "show",
            "dev",
            "docker0",
            "scope",
            "link",
        ):
            return "\n".join(self.docker0_link_address_lines) + (
                "\n" if self.docker0_link_address_lines else ""
            )

        if command[:5] == ("ip", "-4", "route", "show", "default"):
            return (
                "default via 172.22.255.254 dev wlo1\n"
                if self.active.get(MODULE.WIFI_INTERFACE) == CAMPUS_UUID
                else ""
            )

        if command == ("ip", "-4", "route", "show", "table", "all"):
            return self._private_routes_v4()
        if command == ("ip", "-6", "route", "show", "table", "all"):
            return self._private_routes_v6()
        if command == ("ip", "-4", "rule", "show"):
            baseline = [
                "0: from all lookup local",
                "32766: from all lookup main",
                "32767: from all lookup default",
            ]
            return "\n".join(baseline + self.extra_rules_v4) + "\n"
        if command == ("ip", "-6", "rule", "show"):
            baseline = ["0: from all lookup local", "32766: from all lookup main"]
            return "\n".join(baseline + self.extra_rules_v6) + "\n"
        if command and command[0] == "ping":
            return ""
        raise AssertionError(f"unexpected simulated command for {operation}: {command!r}")

    def mutations(self) -> list[tuple[str, str, str]]:
        result = []
        for command, _operation in self.calls:
            if command[:4] == ("nmcli", "--wait", "25", "connection"):
                result.append(
                    (command[4], command[6], command[8] if len(command) > 8 else "")
                )
            elif command[:5] == (
                "nmcli",
                "--ask",
                "--wait",
                "25",
                "connection",
            ):
                result.append((command[5], command[7], command[9]))
        return result


class WorkflowFixture:
    def __init__(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        sys_class_net = Path(self.temp.name)
        (sys_class_net / MODULE.WIFI_INTERFACE / "wireless").mkdir(parents=True)
        (sys_class_net / MODULE.WIRED_INTERFACE).mkdir()
        (sys_class_net / "lo").mkdir()
        docker0 = sys_class_net / "devices" / "virtual" / "net" / "docker0"
        (docker0 / "bridge").mkdir(parents=True)
        (sys_class_net / "docker0").symlink_to(docker0, target_is_directory=True)
        self.signal_latch = MODULE.SignalLatch()
        self.runner = FakeRunner(self.signal_latch)
        self.clock = FakeClock()
        self.workflow = MODULE.PrivateWifiRollback(
            self.runner,
            environ={
                "GO2_FORCE_NOMOTION_PROFILE": MODULE.NOMOTION_PROFILE,
                "GO2_RUNTIME_PLACEMENT": MODULE.NOMOTION_PLACEMENT,
                "GO2_ALLOW_MOTION": MODULE.NOMOTION_MOTION_FLAG,
            },
            signal_latch=self.signal_latch,
            sys_class_net=sys_class_net,
            sleep=self.clock.sleep,
            monotonic=self.clock.monotonic,
        )

    def close(self) -> None:
        self.temp.cleanup()


class NoMotionPrivateWifiRollbackTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = WorkflowFixture()

    def tearDown(self) -> None:
        self.fixture.close()

    def assert_restored(self) -> None:
        self.assert_runner_restored(self.fixture.runner)

    def assert_runner_restored(self, runner: FakeRunner) -> None:
        self.assertEqual(runner.active.get(MODULE.WIFI_INTERFACE), CAMPUS_UUID)
        self.assertEqual(runner.active.get(MODULE.WIRED_INTERFACE), WIRED_UUID)
        owners = [
            (interface, ipaddress.ip_interface(address))
            for interface, addresses in runner.addresses.items()
            for address in addresses
            if ipaddress.ip_interface(address).ip == MODULE.HOST_IP
        ]
        self.assertEqual(
            owners,
            [(MODULE.WIRED_INTERFACE, MODULE.HOST_INTERFACE)],
        )

    def test_preflight_is_read_only_and_uses_fixed_profiles(self) -> None:
        state = self.fixture.workflow.preflight()
        self.assertEqual(state, MODULE.ConnectionState(CAMPUS_UUID, TARGET_UUID, WIRED_UUID))
        self.assertEqual(self.fixture.runner.mutations(), [])

    def test_preflight_accepts_numeric_not_saved_secret_flag(self) -> None:
        self.fixture.runner.profiles[TARGET_UUID][
            "802-11-wireless-security.psk-flags"
        ] = "2"
        self.fixture.workflow.preflight()
        self.assertEqual(self.fixture.runner.mutations(), [])

    def test_success_validates_explicit_direct_route_and_rolls_back_in_order(self) -> None:
        self.fixture.workflow.execute(hold_seconds=0)
        mutations = self.fixture.runner.mutations()
        self.assertEqual(mutations[:2], [("down", WIRED_UUID, ""), ("up", TARGET_UUID, "wlo1")])
        rollback = mutations[2:]
        self.assertEqual(rollback[0], ("up", CAMPUS_UUID, "wlo1"))
        self.assertIn(("down", TARGET_UUID, ""), rollback)
        self.assertLess(
            rollback.index(("up", CAMPUS_UUID, "wlo1")),
            rollback.index(("up", WIRED_UUID, "enp108s0")),
        )
        self.assert_restored()

    def test_preflight_rejects_same_ip_with_wrong_prefix(self) -> None:
        self.fixture.runner.addresses[MODULE.WIRED_INTERFACE] = ("192.168.123.99/32",)
        with self.assertRaisesRegex(MODULE.SwitchError, "one global"):
            self.fixture.workflow.preflight()

    def test_preflight_rejects_third_interface_owner_with_any_prefix(self) -> None:
        self.fixture.runner.addresses["tun-test"] = ("192.168.123.99/32",)
        with self.assertRaisesRegex(MODULE.SwitchError, "one global"):
            self.fixture.workflow.preflight()

    def test_runtime_third_interface_owner_fails_and_blocks_unverified_wired(self) -> None:
        self.fixture.runner.runtime_third_owner = "192.168.123.99/32"
        with self.assertRaisesRegex(MODULE.SwitchError, "rollback was incomplete"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assertNotEqual(
            self.fixture.runner.active.get(MODULE.WIRED_INTERFACE), WIRED_UUID
        )

    def test_runtime_rejects_target_address_with_wrong_prefix(self) -> None:
        self.fixture.runner.target_address = "192.168.123.99/32"
        with self.assertRaisesRegex(MODULE.SwitchError, "one global"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_profile_rejects_routes_rules_secondaries_mode_and_key_mgmt(self) -> None:
        cases = {
            "ipv4.routes": "10.0.0.0/8 192.168.123.1",
            "ipv4.route-table": "100",
            "ipv4.routing-rules": "priority 100 from all table 100",
            "connection.secondaries": "44444444-4444-4444-8444-444444444444",
            "802-11-wireless.mode": "ap",
            "802-11-wireless-security.key-mgmt": "sae",
            "802-11-wireless-security.psk-flags": "0 (none)",
        }
        for field, value in cases.items():
            with self.subTest(field=field):
                fixture = WorkflowFixture()
                try:
                    fixture.runner.profiles[TARGET_UUID][field] = value
                    with self.assertRaisesRegex(MODULE.SwitchError, "unsafe|psk-flags"):
                        fixture.workflow.preflight()
                    self.assertEqual(fixture.runner.mutations(), [])
                finally:
                    fixture.close()

    def test_profile_rejects_every_non_not_saved_secret_flag(self) -> None:
        for value in (
            "0 (none)",
            "1 (agent-owned)",
            "3 (agent-owned, not saved)",
            "4 (not required)",
            "6 (not saved, not required)",
            "2\n4",
        ):
            with self.subTest(value=value):
                fixture = WorkflowFixture()
                try:
                    fixture.runner.profiles[TARGET_UUID][
                        "802-11-wireless-security.psk-flags"
                    ] = value
                    with self.assertRaisesRegex(MODULE.SwitchError, "psk-flags"):
                        fixture.workflow.preflight()
                    self.assertEqual(fixture.runner.mutations(), [])
                finally:
                    fixture.close()

    def test_wired_profile_rejects_nonempty_routes_and_secondaries(self) -> None:
        for field, value in {
            "ipv6.routes": "fd00::/64",
            "connection.secondaries": "44444444-4444-4444-8444-444444444444",
        }.items():
            with self.subTest(field=field):
                fixture = WorkflowFixture()
                try:
                    fixture.runner.profiles[WIRED_UUID][field] = value
                    with self.assertRaisesRegex(MODULE.SwitchError, "unsafe"):
                        fixture.workflow.preflight()
                finally:
                    fixture.close()

    def test_runtime_rejects_split_default_in_non_main_table(self) -> None:
        self.fixture.runner.extra_routes_v4.append("0.0.0.0/1 dev tun-test table 100")
        with self.assertRaisesRegex(MODULE.SwitchError, "split-default|rollback"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_rejects_policy_routing_rule(self) -> None:
        self.fixture.runner.extra_rules_v4.append("100: from all lookup 100")
        with self.assertRaisesRegex(MODULE.SwitchError, "policy rule"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_rejects_gateway_route(self) -> None:
        self.fixture.runner.extra_routes_v4.append(
            "10.0.0.0/8 via 192.168.123.1 dev wlo1 table main"
        )
        with self.assertRaisesRegex(MODULE.SwitchError, "gateway route"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_accepts_exact_networkmanager_ipv4ll_route(self) -> None:
        self.fixture.runner.extra_routes_v4.append(
            "169.254.0.0/16 dev wlo1 scope link metric 1000"
        )
        self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_rejects_every_nonexact_wifi_ipv4ll_route(self) -> None:
        exact = "169.254.0.0/16 dev wlo1 scope link metric 1000"
        cases = (
            ("default", ("default dev wlo1 scope link metric 1000",), ()),
            (
                "gateway",
                (
                    "169.254.0.0/16 via 192.168.123.1 dev wlo1 "
                    "scope link metric 1000",
                ),
                (),
            ),
            (
                "different network",
                ("169.254.1.0/24 dev wlo1 scope link metric 1000",),
                (),
            ),
            ("missing metric", ("169.254.0.0/16 dev wlo1 scope link",), ()),
            (
                "wrong metric",
                ("169.254.0.0/16 dev wlo1 scope link metric 1001",),
                (),
            ),
            (
                "wrong scope",
                ("169.254.0.0/16 dev wlo1 scope host metric 1000",),
                (),
            ),
            (
                "wrong interface",
                ("169.254.0.0/16 dev docker0 scope link metric 1000",),
                (),
            ),
            (
                "explicit table",
                ("169.254.0.0/16 dev wlo1 table main scope link metric 1000",),
                (),
            ),
            (
                "extra proto",
                ("169.254.0.0/16 dev wlo1 proto kernel scope link metric 1000",),
                (),
            ),
            (
                "extra source",
                (
                    "169.254.0.0/16 dev wlo1 scope link "
                    "src 169.254.1.2 metric 1000",
                ),
                (),
            ),
            (
                "reordered attributes",
                ("169.254.0.0/16 dev wlo1 metric 1000 scope link",),
                (),
            ),
            ("duplicate exact route", (exact, exact), ()),
            (
                "exact route plus gateway route",
                (exact, "10.0.0.0/8 via 192.168.123.1 dev wlo1"),
                (),
            ),
            ("IPv6 link-local route", (), ("fe80::/64 dev wlo1 scope link",)),
        )
        for label, ipv4_routes, ipv6_routes in cases:
            with self.subTest(label=label):
                fixture = WorkflowFixture()
                try:
                    fixture.runner.extra_routes_v4.extend(ipv4_routes)
                    fixture.runner.extra_routes_v6.extend(ipv6_routes)
                    with self.assertRaises(MODULE.SwitchError):
                        fixture.workflow.execute(hold_seconds=0)
                    self.assert_runner_restored(fixture.runner)
                finally:
                    fixture.close()

    def test_runtime_rejects_every_nonexact_ipv6_local_baseline(self) -> None:
        loopback_main, bridge_main, loopback_local, bridge_local, bridge_multicast = (
            TARGET_IPV6_ROUTES
        )
        cases = (
            ("missing loopback main", (), (loopback_main,)),
            ("missing bridge main", (), (bridge_main,)),
            ("missing bridge local", (), (bridge_local,)),
            ("missing bridge multicast", (), (bridge_multicast,)),
            ("duplicate loopback", (loopback_main,), ()),
            ("duplicate bridge route", (bridge_main,), ()),
            ("duplicate bridge local kind", (bridge_local,), ()),
            ("bridge local different address", (
                "local fe80::1 dev docker0 table local proto kernel metric 0 pref medium",
            ), ()),
            ("duplicate multicast", (bridge_multicast,), ()),
            ("loopback wrong interface", (
                "::1 dev wlo1 proto kernel metric 256 pref medium",
            ), ()),
            ("loopback wrong proto", (
                "::1 dev lo proto static metric 256 pref medium",
            ), ()),
            ("loopback wrong metric", (
                "::1 dev lo proto kernel metric 257 pref medium",
            ), ()),
            ("loopback extra attribute", (
                "::1 dev lo proto kernel metric 256 pref medium onlink",
            ), ()),
            ("loopback duplicate attribute", (
                "::1 dev lo proto kernel metric 256 metric 256 pref medium",
            ), ()),
            ("physical unicast", (
                "fe80::/64 dev enp-test proto kernel metric 256 linkdown pref medium",
            ), ()),
            ("physical local", (
                "local fe80::1 dev enp-test table local proto kernel metric 0 pref medium",
            ), ()),
            ("wlo1 IPv6", (
                "fe80::/64 dev wlo1 proto kernel metric 256 pref medium",
            ), ()),
            ("non-bridge virtual interface", (
                "multicast ff00::/8 dev tun-test table local proto kernel "
                "metric 256 linkdown pref medium",
            ), ()),
            ("different local bridge", (
                "multicast ff00::/8 dev br-test table local proto kernel "
                "metric 256 linkdown pref medium",
            ), ()),
            ("multicast wrong proto", (
                "multicast ff00::/8 dev docker0 table local proto static "
                "metric 256 linkdown pref medium",
            ), ()),
            ("multicast wrong metric", (
                "multicast ff00::/8 dev docker0 table local proto kernel "
                "metric 257 linkdown pref medium",
            ), ()),
            ("multicast wrong table", (
                "multicast ff00::/8 dev docker0 table main proto kernel "
                "metric 256 linkdown pref medium",
            ), ()),
            ("multicast wrong type", (
                "local ff00::/8 dev docker0 table local proto kernel "
                "metric 256 linkdown pref medium",
            ), ()),
            ("multicast wrong prefix", (
                "multicast ff02::/16 dev docker0 table local proto kernel "
                "metric 256 linkdown pref medium",
            ), ()),
            ("multicast missing linkdown", (
                "multicast ff00::/8 dev docker0 table local proto kernel "
                "metric 256 pref medium",
            ), ()),
            ("multicast extra attribute", (
                "multicast ff00::/8 dev docker0 table local proto kernel "
                "metric 256 linkdown pref medium expires 30sec",
            ), ()),
            ("bridge main wrong prefix", (
                "fe80::/10 dev docker0 proto kernel metric 256 linkdown pref medium",
            ), ()),
            ("bridge local outside exact prefix", (
                "local febf::1 dev docker0 table local proto kernel metric 0 pref medium",
            ), ()),
            ("default", ("default dev docker0 metric 256",), ()),
            ("gateway", (
                "2001:db8::/64 via fe80::1 dev docker0 metric 256",
            ), ()),
            ("unexpected route type", (
                "broadcast fe80::1 dev enp-test table local",
            ), ()),
        )
        for label, extra_routes, omitted_routes in cases:
            with self.subTest(label=label):
                fixture = WorkflowFixture()
                try:
                    sys_class_net = Path(fixture.temp.name)
                    (sys_class_net / "enp-test").mkdir()
                    virtual_net = sys_class_net / "devices" / "virtual" / "net"
                    (virtual_net / "tun-test").mkdir()
                    (virtual_net / "br-test" / "bridge").mkdir(parents=True)
                    (sys_class_net / "tun-test").symlink_to(
                        virtual_net / "tun-test", target_is_directory=True
                    )
                    (sys_class_net / "br-test").symlink_to(
                        virtual_net / "br-test", target_is_directory=True
                    )
                    fixture.runner.extra_routes_v6.extend(extra_routes)
                    fixture.runner.omitted_routes_v6.update(omitted_routes)
                    with self.assertRaises(MODULE.SwitchError):
                        fixture.workflow.execute(hold_seconds=0)
                    self.assert_runner_restored(fixture.runner)
                finally:
                    fixture.close()

    def test_runtime_accepts_absent_docker0_ipv6_state_and_rolls_back(self) -> None:
        self.fixture.runner.docker0_link_address_lines = []
        self.fixture.runner.omitted_routes_v6.update(TARGET_IPV6_ROUTES[1:2])
        self.fixture.runner.omitted_routes_v6.update(TARGET_IPV6_ROUTES[3:])
        self.fixture.workflow.execute(hold_seconds=2)
        self.assertGreaterEqual(self.fixture.clock.now, 2.0)
        self.assert_restored()

    def test_runtime_accepts_missing_docker0_interface_without_routes(self) -> None:
        docker0 = Path(self.fixture.temp.name) / "docker0"
        docker0.unlink()
        self.fixture.runner.docker0_link_address_lines = []
        self.fixture.runner.omitted_routes_v6.update(TARGET_IPV6_ROUTES[1:2])
        self.fixture.runner.omitted_routes_v6.update(TARGET_IPV6_ROUTES[3:])
        self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_rejects_nonbridge_docker0_interface(self) -> None:
        sys_class_net = Path(self.fixture.temp.name)
        (sys_class_net / "docker0").unlink()
        docker0 = sys_class_net / "devices" / "virtual" / "net" / "docker0-not-bridge"
        docker0.mkdir(parents=True)
        (sys_class_net / "docker0").symlink_to(docker0, target_is_directory=True)
        with self.assertRaisesRegex(MODULE.SwitchError, "local virtual bridge"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_rejects_docker0_routes_without_link_local_address(self) -> None:
        self.fixture.runner.docker0_link_address_lines = []
        with self.assertRaisesRegex(MODULE.SwitchError, "unexpected IPv6 route"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_hold_rejects_late_docker0_address_route_mismatch(self) -> None:
        changed: list[bool] = []

        def remove_link_local_only() -> None:
            if not changed:
                changed.append(True)
                self.fixture.runner.docker0_link_address_lines = []

        self.fixture.clock.on_sleep = remove_link_local_only
        with self.assertRaisesRegex(MODULE.SwitchError, "unexpected IPv6 route"):
            self.fixture.workflow.execute(hold_seconds=1)
        self.assert_restored()

    def test_runtime_rejects_ambiguous_docker0_link_local_address(self) -> None:
        exact = (
            "5: docker0 inet6 fe80::454:87ff:fe24:1966/64 scope link "
            "valid_lft forever preferred_lft forever"
        )
        cases = {
            "multiple": [
                exact,
                "5: docker0 inet6 fe80::1/64 scope link valid_lft forever",
            ],
            "wrong interface": [
                "5: wlo1 inet6 fe80::454:87ff:fe24:1966/64 scope link"
            ],
            "wrong prefix": [
                "5: docker0 inet6 fe80::454:87ff:fe24:1966/128 scope link"
            ],
            "non-link-local": [
                "5: docker0 inet6 2001:db8::1/64 scope link"
            ],
            "wrong scope": [
                "5: docker0 inet6 fe80::454:87ff:fe24:1966/64 scope global"
            ],
            "duplicate scope field": [exact + " scope link"],
            "malformed": ["not-an-ip-address-record"],
        }
        for label, lines in cases.items():
            with self.subTest(label=label):
                fixture = WorkflowFixture()
                try:
                    fixture.runner.docker0_link_address_lines = lines
                    with self.assertRaises(MODULE.SwitchError):
                        fixture.workflow.execute(hold_seconds=0)
                    self.assert_runner_restored(fixture.runner)
                finally:
                    fixture.close()

    def test_runtime_rejects_overlapping_route_on_virtual_bridge(self) -> None:
        bridge = Path(self.fixture.temp.name) / "docker0" / "bridge"
        bridge.mkdir(parents=True, exist_ok=True)
        self.fixture.runner.extra_routes_v4.append(
            "192.168.123.0/24 dev docker0 proto kernel scope link src 192.168.123.1"
        )
        with self.assertRaisesRegex(MODULE.SwitchError, "overlaps"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_hold_rechecks_all_routes_and_rolls_back_on_late_change(self) -> None:
        injected: list[bool] = []

        def inject_route() -> None:
            if (
                not injected
                and self.fixture.runner.active.get(MODULE.WIFI_INTERFACE) == TARGET_UUID
            ):
                injected.append(True)
                self.fixture.runner.extra_routes_v4.append(
                    "0.0.0.0/1 dev tun-test table 100"
                )

        self.fixture.clock.on_sleep = inject_route
        with self.assertRaisesRegex(MODULE.SwitchError, "split-default"):
            self.fixture.workflow.execute(hold_seconds=2)
        self.assertEqual(injected, [True])
        self.assert_restored()

    def test_ping_timeout_rolls_back_before_propagating_failure(self) -> None:
        self.fixture.runner.fail_once[
            "reachability check for private host 192.168.123.18"
        ] = MODULE.CommandTimedOut("simulated ping timeout")
        with self.assertRaisesRegex(MODULE.CommandTimedOut, "simulated ping timeout"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_interactive_mode_uses_nmcli_ask_and_restores_normally(self) -> None:
        self.fixture.workflow.execute(hold_seconds=0, interactive_target=True)
        interactive_calls = [
            command
            for command, operation in self.fixture.runner.calls
            if operation == "interactively activate private Wi-Fi"
        ]
        self.assertEqual(len(interactive_calls), 1)
        self.assertEqual(interactive_calls[0][0:2], ("nmcli", "--ask"))
        self.assert_restored()

    def test_interactive_mode_rejects_non_tty_before_any_mutation(self) -> None:
        self.fixture.runner.interactive_terminal_ready = False
        with self.assertRaisesRegex(MODULE.SwitchError, "non-interactive"):
            self.fixture.workflow.execute(hold_seconds=0, interactive_target=True)
        self.assertEqual(self.fixture.runner.mutations(), [])

    def test_interactive_activation_timeout_rolls_back_late_nm_completion(self) -> None:
        self.fixture.runner.interactive_timeout = True
        with self.assertRaisesRegex(MODULE.CommandTimedOut, "interactive activation"):
            self.fixture.workflow.execute(hold_seconds=0, interactive_target=True)
        self.assertTrue(self.fixture.runner.late_target_fired)
        self.assert_restored()

    def test_interactive_wrong_secret_failure_rolls_back(self) -> None:
        self.fixture.runner.fail_once[
            "interactively activate private Wi-Fi"
        ] = MODULE.SwitchError("simulated rejected one-time secret")
        with self.assertRaisesRegex(MODULE.SwitchError, "rejected one-time secret"):
            self.fixture.workflow.execute(hold_seconds=0, interactive_target=True)
        self.assert_restored()

    def test_interactive_signal_is_deferred_until_nmcli_boundary(self) -> None:
        self.fixture.runner.capture_signal_on_operation = (
            "interactively activate private Wi-Fi"
        )
        with self.assertRaises(MODULE.SwitchInterrupted):
            self.fixture.workflow.execute(hold_seconds=0, interactive_target=True)
        self.assert_restored()

    def test_signal_is_raised_only_after_activation_command_boundary(self) -> None:
        self.fixture.runner.capture_signal_on_operation = (
            "temporarily activate private Wi-Fi"
        )
        with self.assertRaises(MODULE.SwitchInterrupted):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assertIn(("up", TARGET_UUID, MODULE.WIFI_INTERFACE), self.fixture.runner.mutations())
        self.assert_restored()

    def test_command_runner_latches_signal_until_subprocess_returns(self) -> None:
        latch = MODULE.SignalLatch()
        runner = MODULE.CommandRunner(latch)
        completed_boundary: list[bool] = []

        def fake_run(*_args, **kwargs):  # noqa: ANN002, ANN003, ANN202
            self.assertTrue(kwargs["start_new_session"])
            latch.capture(signal.SIGTERM)
            completed_boundary.append(True)
            return subprocess.CompletedProcess(["nmcli"], 0, stdout="", stderr="")

        with mock.patch.object(MODULE.subprocess, "run", side_effect=fake_run):
            with self.assertRaises(MODULE.SwitchInterrupted):
                runner.run(("nmcli", "device", "show"), timeout=1, operation="test")
        self.assertEqual(completed_boundary, [True])

    def test_interactive_runner_inherits_terminal_fds_and_captures_nothing(self) -> None:
        latch = MODULE.SignalLatch()
        runner = MODULE.CommandRunner(latch)
        observed: list[dict[str, object]] = []

        def fake_run(*_args, **kwargs):  # noqa: ANN002, ANN003, ANN202
            observed.append(kwargs)
            return subprocess.CompletedProcess(["nmcli"], 0)

        command = (
            "nmcli",
            "--ask",
            "--wait",
            "25",
            "connection",
            "up",
            "uuid",
            TARGET_UUID,
            "ifname",
            MODULE.WIFI_INTERFACE,
        )
        with mock.patch.object(MODULE.subprocess, "run", side_effect=fake_run):
            runner.run_interactive(command, timeout=28, operation="interactive test")
        self.assertEqual(len(observed), 1)
        self.assertIsNone(observed[0]["stdin"])
        self.assertIsNone(observed[0]["stdout"])
        self.assertIsNone(observed[0]["stderr"])
        self.assertFalse(observed[0]["start_new_session"])

    def test_interactive_runner_maps_nonzero_without_forwarding_output(self) -> None:
        runner = MODULE.CommandRunner(MODULE.SignalLatch())
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            return_value=subprocess.CompletedProcess(["nmcli"], 10),
        ) as run:
            with self.assertRaisesRegex(MODULE.SwitchError, "interactive test failed"):
                runner.run_interactive(
                    ("nmcli", "--ask"), timeout=28, operation="interactive test"
                )
        self.assertIsNone(run.call_args.kwargs["stdout"])
        self.assertIsNone(run.call_args.kwargs["stderr"])

    def test_interactive_runner_maps_timeout_after_child_boundary(self) -> None:
        runner = MODULE.CommandRunner(MODULE.SignalLatch())
        with mock.patch.object(
            MODULE.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired(("nmcli", "--ask"), 28),
        ):
            with self.assertRaisesRegex(MODULE.CommandTimedOut, "interactive test"):
                runner.run_interactive(
                    ("nmcli", "--ask"), timeout=28, operation="interactive test"
                )

    def test_activation_client_timeout_with_late_nm_completion_is_reconciled(self) -> None:
        self.fixture.runner.late_target_timeout = True
        with self.assertRaisesRegex(MODULE.CommandTimedOut, "activation client timeout"):
            self.fixture.workflow.execute(hold_seconds=0)
        self.assertTrue(self.fixture.runner.late_target_fired)
        campus_ups = [
            mutation
            for mutation in self.fixture.runner.mutations()
            if mutation == ("up", CAMPUS_UUID, MODULE.WIFI_INTERFACE)
        ]
        self.assertGreaterEqual(len(campus_ups), 3)
        self.assert_restored()

    def test_late_target_completion_after_wired_restore_forces_another_round(self) -> None:
        self.fixture.runner.late_target_on_wired_restore = True
        self.fixture.workflow.execute(hold_seconds=0)
        self.assertTrue(self.fixture.runner.late_target_fired)
        mutations = self.fixture.runner.mutations()
        self.assertIn(("down", WIRED_UUID, ""), mutations[2:])
        self.assertGreaterEqual(
            mutations.count(("up", CAMPUS_UUID, MODULE.WIFI_INTERFACE)), 4
        )
        self.assert_restored()

    def test_failed_campus_restore_never_restores_duplicate_wired_address(self) -> None:
        self.fixture.runner.ignore_activation.add(CAMPUS_UUID)
        with self.assertRaisesRegex(MODULE.SwitchError, "rollback"):
            self.fixture.workflow.execute(hold_seconds=0)
        owners = [
            ipaddress.ip_interface(address)
            for addresses in self.fixture.runner.addresses.values()
            for address in addresses
            if ipaddress.ip_interface(address).ip == MODULE.HOST_IP
        ]
        self.assertLessEqual(len(owners), 1)
        self.assertEqual(
            self.fixture.runner.active.get(MODULE.WIRED_INTERFACE), WIRED_UUID
        )

    def test_unexpected_rollback_query_error_does_not_abort_recovery(self) -> None:
        self.fixture.runner.fail_once["observe campus Wi-Fi restoration"] = ValueError(
            "simulated malformed output"
        )
        self.fixture.workflow.execute(hold_seconds=0)
        self.assert_restored()

    def test_runtime_gate_is_exact(self) -> None:
        self.fixture.workflow.environ["GO2_ALLOW_MOTION"] = "true"
        with self.assertRaisesRegex(MODULE.SwitchError, "exact corrected"):
            self.fixture.workflow.preflight()
        self.assertEqual(self.fixture.runner.mutations(), [])

    def test_source_has_no_credential_value_access_or_motion_command(self) -> None:
        original_source = SCRIPT.read_text(encoding="utf-8")
        source = original_source.lower()
        self.assertIn('target_ssid = "robonix-go2"', source)
        self.assertIn('wifi_interface = "wlo1"', source)
        self.assertIn("802-11-wireless-security.key-mgmt", source)
        literals = {
            node.value
            for node in ast.walk(ast.parse(original_source))
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertNotIn("802-11-wireless-security.psk", literals)
        self.assertIn("802-11-wireless-security.psk-flags", literals)
        self.assertIn("--ask", source)
        for forbidden in (
            "--show-secrets",
            "getpass(",
            "input(",
            "ros2",
            "/cmd_vel",
            "/lowcmd",
            "/api/sport/request",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)

    def test_cli_requires_an_explicit_fixed_mode_and_has_no_target_override(self) -> None:
        parser = MODULE.build_parser()
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parser.parse_args([])
        option_strings = {
            option
            for action in parser._actions
            for option in action.option_strings
        }
        self.assertNotIn("--ssid", option_strings)
        self.assertNotIn("--wifi-interface", option_strings)
        self.assertNotIn("--wired-interface", option_strings)
        self.assertIn("--execute-after-approval-interactive", option_strings)


if __name__ == "__main__":
    unittest.main()
