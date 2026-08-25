from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "validate_first_motion_network.py"
SPEC = importlib.util.spec_from_file_location("first_motion_network", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
network = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = network
SPEC.loader.exec_module(network)


class FirstMotionNetworkTest(unittest.TestCase):
    def snapshot(self, **changes):
        values = {
            "interface": "wlx500ff54809b8",
            "transport": "wireless-private",
            "physical": True,
            "wireless": True,
            "operstate": "up",
            "ipv4": ("192.168.123.99/24",),
            "ipv6": (),
            "default_v4_interfaces": ("wlo1",),
            "default_v6_interfaces": (),
            "connection_name": "Robonix-Go2",
            "connection_uuid": "ce767234-9037-4a53-a5f4-aa7b6cbf743f",
            "gateway_dns_values": ("", "", "", ""),
            "private_route": (
                "192.168.123.0/24 dev wlx500ff54809b8 "
                "proto kernel scope link src 192.168.123.99 metric 601"
            ),
            "robot_route": (
                "192.168.123.161 dev wlx500ff54809b8 "
                "src 192.168.123.99 uid 1000"
            ),
            "orin_route": (
                "192.168.123.18 dev wlx500ff54809b8 "
                "src 192.168.123.99 uid 1000"
            ),
        }
        values.update(changes)
        return network.NetworkSnapshot(**values)

    def validate(self, snapshot=None, **changes):
        network.validate_snapshot(
            snapshot or self.snapshot(**changes),
            expected_connection_uuid=(
                "ce767234-9037-4a53-a5f4-aa7b6cbf743f"
            ),
            expected_connection_name="Robonix-Go2",
            internet_interface="wlo1",
            robot_ip="192.168.123.161",
            orin_ip="192.168.123.18",
        )

    def test_exact_private_wireless_topology_passes(self):
        self.validate()

    def test_private_adapter_must_not_own_default_route(self):
        with self.assertRaisesRegex(network.NetworkGateError, "default route"):
            self.validate(
                default_v4_interfaces=("wlo1", "wlx500ff54809b8")
            )
        with self.assertRaisesRegex(
            network.NetworkGateError, "all laptop IPv4 default routes"
        ):
            self.validate(default_v4_interfaces=("wlo1", "usb0"))

    def test_private_subnet_route_must_be_unique(self):
        with self.assertRaisesRegex(
            network.NetworkGateError, "exactly one direct route"
        ):
            self.validate(
                private_route=(
                    "192.168.123.0/24 dev wlx500ff54809b8 "
                    "src 192.168.123.99\n"
                    "192.168.123.0/24 dev enp108s0 "
                    "src 192.168.123.99 metric 900"
                )
            )

    def test_robot_and_orin_routes_must_not_escape_through_campus(self):
        for route_name in ("robot_route", "orin_route"):
            with self.subTest(route=route_name):
                with self.assertRaisesRegex(
                    network.NetworkGateError, "private interface"
                ):
                    self.validate(
                        **{
                            route_name: (
                                "192.168.123.161 via 172.22.255.254 "
                                "dev wlo1 src 172.22.123.232"
                            )
                        }
                    )

    def test_connection_identity_is_exact(self):
        with self.assertRaisesRegex(network.NetworkGateError, "UUID"):
            self.validate(
                connection_uuid="11111111-1111-1111-1111-111111111111"
            )
        with self.assertRaisesRegex(network.NetworkGateError, "name"):
            self.validate(connection_name="OtherNetwork")

    def test_gateway_dns_ipv6_and_extra_ipv4_fail_closed(self):
        cases = (
            ({"gateway_dns_values": ("192.168.123.1", "", "", "")}, "gateway"),
            ({"ipv6": ("fe80::1/64",)}, "IPv6"),
            (
                {
                    "ipv4": (
                        "192.168.123.99/24",
                        "192.168.123.100/24",
                    )
                },
                "exactly",
            ),
        )
        for changes, message in cases:
            with self.subTest(changes=changes):
                with self.assertRaisesRegex(network.NetworkGateError, message):
                    self.validate(**changes)

    def test_transport_is_physical_and_explicit(self):
        with self.assertRaisesRegex(network.NetworkGateError, "physical"):
            self.validate(physical=False)
        with self.assertRaisesRegex(network.NetworkGateError, "Wi-Fi"):
            self.validate(wireless=False)
        wired = self.snapshot(
            interface="enp108s0",
            transport="wired",
            wireless=False,
            connection_name="go2-readonly",
            connection_uuid="7e9da516-35c4-419d-b791-be4f424ad437",
            private_route=(
                "192.168.123.0/24 dev enp108s0 "
                "proto kernel scope link src 192.168.123.99"
            ),
            robot_route=(
                "192.168.123.161 dev enp108s0 src 192.168.123.99"
            ),
            orin_route="192.168.123.18 dev enp108s0 src 192.168.123.99",
        )
        network.validate_snapshot(
            wired,
            expected_connection_uuid=None,
            expected_connection_name=None,
            internet_interface="wlo1",
            robot_ip="192.168.123.161",
            orin_ip="192.168.123.18",
        )


if __name__ == "__main__":
    unittest.main()
