from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
START = ROOT / "start.sh"
NOMOTION = ROOT / "scripts" / "start_workstation_full_nomotion_corrected.sh"
FIRST_MOTION = ROOT / "scripts" / "start_first_motion_corrected.sh"
PROFILE = "workstation-full-nomotion-corrected-v1"
PRIVATE_WIFI_SSID = "Robonix-Go2"


def wireless_gate(source: str) -> str:
    start = source.index(
        'if [[ -d "/sys/class/net/$GO2_NETWORK_INTERFACE/wireless" ]]'
    )
    end = source.index(
        '[[ "$(cat -- "/sys/class/net/$GO2_NETWORK_INTERFACE/operstate")"',
        start,
    )
    return source[start:end]


class NoMotionPrivateWifiContractTest(unittest.TestCase):
    def test_shared_start_allows_wifi_only_for_exact_nomotion_profile(self) -> None:
        source = START.read_text(encoding="utf-8")
        gate = wireless_gate(source)

        self.assertIn(PROFILE, gate)
        self.assertIn('"$GO2_RUNTIME_PLACEMENT" == workstation-local', gate)
        self.assertIn('"$GO2_ALLOW_MOTION" == false', gate)
        self.assertIn("must be wired outside the corrected full no-motion profile", gate)
        self.assertIn(
            "nmcli --get-values GENERAL.CON-UUID", gate
        )
        self.assertIn(
            "nmcli --get-values 802-11-wireless.ssid", gate
        )
        self.assertIn(
            '"$active_wifi_ssid" == "$NOMOTION_PRIVATE_WIFI_SSID"', gate
        )
        self.assertIn(
            f"readonly NOMOTION_PRIVATE_WIFI_SSID={PRIVATE_WIFI_SSID}", source
        )
        self.assertLess(
            source.index("unset NOMOTION_PRIVATE_WIFI_SSID"),
            source.index(f"readonly NOMOTION_PRIVATE_WIFI_SSID={PRIVATE_WIFI_SSID}"),
        )
        self.assertIn("if ! active_connection_uuid=\"$(", gate)
        self.assertIn("^[[:xdigit:]]{8}-", gate)
        self.assertIn("if ! active_wifi_ssid=\"$(", gate)

        # The only profile named by the exception is a one-way safety marker:
        # start.sh forces it back to local, no-motion operation before reaching
        # the network gate. Any other non-empty marker is rejected outright.
        profile_case = source[
            source.index('case "$INHERITED_FORCE_NOMOTION_PROFILE" in') :
            source.index("# Python primitives must use", 1)
        ]
        self.assertIn(PROFILE, profile_case)
        self.assertIn("export GO2_RUNTIME_PLACEMENT=workstation-local", profile_case)
        self.assertIn("export GO2_ALLOW_MOTION=false", profile_case)
        self.assertIn('echo "unsupported GO2_FORCE_NOMOTION_PROFILE"', profile_case)

    def test_corrected_wrapper_preserves_all_isolation_checks(self) -> None:
        source = NOMOTION.read_text(encoding="utf-8")
        gate = wireless_gate(source)
        forced = source[: source.index('[[ -x "$PYTHON" ]]')]

        self.assertIn(f"PROFILE={PROFILE}", forced)
        self.assertIn('export GO2_FORCE_NOMOTION_PROFILE="$PROFILE"', forced)
        self.assertIn("export GO2_RUNTIME_PLACEMENT=workstation-local", forced)
        self.assertIn("export GO2_ALLOW_MOTION=false", forced)
        self.assertIn('"$GO2_FORCE_NOMOTION_PROFILE" == "$PROFILE"', gate)
        self.assertIn('"$GO2_RUNTIME_PLACEMENT" == workstation-local', gate)
        self.assertIn('"$GO2_ALLOW_MOTION" == false', gate)
        self.assertIn("nmcli --get-values GENERAL.CON-UUID", gate)
        self.assertIn("nmcli --get-values 802-11-wireless.ssid", gate)
        self.assertIn(
            '"$active_wifi_ssid" == "$NOMOTION_PRIVATE_WIFI_SSID"', gate
        )
        self.assertIn(
            f"readonly NOMOTION_PRIVATE_WIFI_SSID={PRIVATE_WIFI_SSID}", source
        )
        self.assertLess(
            source.index("unset NOMOTION_PRIVATE_WIFI_SSID"),
            source.index(f"readonly NOMOTION_PRIVATE_WIFI_SSID={PRIVATE_WIFI_SSID}"),
        )
        self.assertIn("if ! active_connection_uuid=\"$(", gate)
        self.assertIn("^[[:xdigit:]]{8}-", gate)
        self.assertIn("if ! active_wifi_ssid=\"$(", gate)

        for required in (
            "192.168.123.99/24",
            'ip -o -6 addr show dev "$GO2_NETWORK_INTERFACE"',
            'ip -4 route show default dev "$GO2_NETWORK_INTERFACE"',
            'ip -6 route show default dev "$GO2_NETWORK_INTERFACE"',
            "IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS",
        ):
            with self.subTest(required=required):
                self.assertIn(required, source)

    def test_one_shot_interface_override_survives_both_env_loads_only_for_nomotion(self) -> None:
        start = START.read_text(encoding="utf-8")
        nomotion = NOMOTION.read_text(encoding="utf-8")
        first_motion = FIRST_MOTION.read_text(encoding="utf-8")

        wrapper_capture = nomotion.index(
            'INHERITED_NETWORK_INTERFACE="${GO2_NETWORK_INTERFACE:-}"'
        )
        wrapper_env = nomotion.index('source "$ROOT/.env"')
        wrapper_restore = nomotion.index(
            'export GO2_NETWORK_INTERFACE="$INHERITED_NETWORK_INTERFACE"'
        )
        wrapper_internal = nomotion.index(
            'export GO2_NOMOTION_NETWORK_INTERFACE="$GO2_NETWORK_INTERFACE"'
        )
        self.assertLess(wrapper_capture, wrapper_env)
        self.assertLess(wrapper_env, wrapper_restore)
        self.assertLess(wrapper_restore, wrapper_internal)

        shared_capture = start.index(
            'INHERITED_NOMOTION_NETWORK_INTERFACE="${GO2_NOMOTION_NETWORK_INTERFACE:-}"'
        )
        shared_env = start.index('source "$DEPLOY_DIR/.env"')
        profile_case = start.index('case "$INHERITED_FORCE_NOMOTION_PROFILE" in')
        shared_restore = start.index(
            'export GO2_NETWORK_INTERFACE="$INHERITED_NOMOTION_NETWORK_INTERFACE"'
        )
        self.assertLess(shared_capture, shared_env)
        self.assertLess(shared_env, profile_case)
        self.assertLess(profile_case, shared_restore)
        self.assertIn(
            "corrected no-motion wrapper did not provide its validated network interface",
            start,
        )
        self.assertNotIn("GO2_NOMOTION_NETWORK_INTERFACE", first_motion)

    def test_no_generic_wifi_override_and_first_motion_is_explicit_opt_in(
        self,
    ) -> None:
        start = START.read_text(encoding="utf-8")
        nomotion = NOMOTION.read_text(encoding="utf-8")
        first_motion = FIRST_MOTION.read_text(encoding="utf-8")

        for forbidden in (
            "GO2_ALLOW_WIFI",
            "GO2_NETWORK_ALLOW_WIFI",
            "GO2_WIFI_SSID",
            "GO2_NETWORK_SSID",
            "GO2_NOMOTION_WIFI_SSID",
        ):
            self.assertNotIn(forbidden, start)
            self.assertNotIn(forbidden, nomotion)
            self.assertNotIn(forbidden, first_motion)
        self.assertNotRegex(start + nomotion + first_motion, r"\bGO2_[A-Z0-9_]*SSID\b")

        self.assertNotIn(PROFILE, first_motion)
        self.assertIn(
            'TRANSPORT="${GO2_FIRST_MOTION_TRANSPORT:-wired}"',
            first_motion,
        )
        self.assertIn("wired)", first_motion)
        self.assertIn("wireless-private)", first_motion)
        self.assertIn(
            "I_APPROVE_GO2_WIRELESS_PRIVATE_LAN_FIRST_10CM",
            first_motion,
        )
        self.assertIn("wlx500ff54809b8", first_motion)
        self.assertIn("Robonix-Go2", first_motion)
        self.assertIn(
            "ce767234-9037-4a53-a5f4-aa7b6cbf743f",
            first_motion,
        )
        self.assertIn(
            'if [[ "$TRANSPORT" == wired ]]; then',
            first_motion,
        )
        self.assertIn("Go2 interface must be a physical wired device", first_motion)
        self.assertIn(
            "wireless-private transport requires a physical Wi-Fi device",
            first_motion,
        )
        self.assertIn("validate_first_motion_network.py", first_motion)
        self.assertIn("--transport wireless-private", first_motion)
        self.assertIn("192.168.123.99/24", first_motion)
        self.assertIn(
            "IP4.GATEWAY,IP4.DNS,IP6.GATEWAY,IP6.DNS", first_motion
        )
        for forbidden in (
            "nmcli connection up",
            "nmcli connection modify",
            "nmcli device connect",
            "ip route add",
            "ip addr add",
        ):
            self.assertNotIn(forbidden, first_motion)


if __name__ == "__main__":
    unittest.main()
