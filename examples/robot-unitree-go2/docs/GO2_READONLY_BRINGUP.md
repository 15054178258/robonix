# Go2 read-only network and sensor bring-up

This procedure is subscription-only. It never starts a Unitree motion example,
publishes a velocity/control topic, changes posture, or moves the robot. Every
network-changing or `sudo` command below is a separately approved operator
step; do not combine or run those steps without explicit approval.

The address and CycloneDDS pattern follow the pinned official
`third_party/unitree_ros2/README.md` at commit `668d1ec5`. Its example NIC name
must always be replaced with the interface physically identified on this host.

## 1. Identify the dedicated physical NIC (read-only)

Keep the Go2 stationary using its official app/remote. From the repository
root, capture a baseline with the cable disconnected, connect only the direct
Go2 Ethernet cable, then repeat:

```bash
bash scripts/check_go2_network.sh
ip -br link
ip -br addr
nmcli device status
```

The candidate is the physical Ethernet interface whose carrier changes. Reject
loopback, Wi-Fi, Docker/bridge, VPN and other virtual devices. Record its exact
name and the original NetworkManager profile:

```bash
IFACE='<CONFIRMED_GO2_WIRED_INTERFACE>'
ORIGINAL_PROFILE='<ORIGINAL_PROFILE_OR_EMPTY>'
nmcli --fields GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show "$IFACE"
nmcli --fields NAME,UUID,TYPE,DEVICE,AUTOCONNECT connection show
ip -4 addr show dev "$IFACE"
ip -4 route show
```

Send only `IFACE` and `ORIGINAL_PROFILE` to the maintainer. Do not send a
password, token, private key, device handover document or API credential.

## 2. Separately approved NetworkManager changes

Operation A creates, but does not activate, a dedicated non-default profile.
It writes NetworkManager configuration and therefore needs its own approval:

```bash
sudo nmcli connection add type ethernet ifname "$IFACE" con-name go2-readonly connection.autoconnect no ipv4.method manual ipv4.addresses 192.168.123.99/24 ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled
```

Review it without changing state:

```bash
nmcli connection show go2-readonly
```

Operation B activates that profile, temporarily replacing any connection on
this one NIC. It needs a second approval:

```bash
sudo nmcli connection up go2-readonly ifname "$IFACE"
```

Verify the exact fail-closed shape expected by `start.sh`:

```bash
ip -br addr show dev "$IFACE"
ip -4 route show
nmcli --fields GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show "$IFACE"
```

The NIC must be UP with exactly `192.168.123.99/24`, no other IPv4 address,
IPv6 disabled, and no gateway or DNS for either address family. Stop if another
address, gateway or DNS appears. This procedure never
bridges or routes the Go2 link to Wi-Fi, a VPN or another LAN.

## 3. Bind the current shell to the NIC (no network change)

```bash
source /opt/ros/humble/setup.bash
source rbnx-build/unitree_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
ros2 pkg prefix rmw_cyclonedds_cpp
```

Do not put this XML in `~/.bashrc`. The deployment `start.sh` later discards any
inherited value and generates its own binding from `GO2_NETWORK_INTERFACE`.

## 4. Discover and sample state/sensors (read-only)

```bash
bash scripts/list_go2_topics.sh
bash scripts/check_sensors.sh
bash scripts/check_tf.sh
```

The scripts inventory topic names, types, endpoint QoS, bounded rates and
`frame_id`; every `ros2 topic echo` is timeout-bounded. For an independently
confirmed state topic, a manual sample must remain bounded too:

```bash
timeout --signal=INT --kill-after=2s 6s ros2 topic echo --once --no-arr /sportmodestate
timeout --signal=INT --kill-after=2s 8s ros2 topic hz --window 20 /sportmodestate
```

If only `/lf/sportmodestate` exists, substitute that exact discovered name.
Never replace these commands with `sport_mode_ctrl`, `go2_sport_client`,
`go2_stand_example`, `low_level_ctrl`, `ros2 topic pub`, or any posture API.

Record whether consecutive `SportModeState.stamp` values are non-zero and
strictly increasing, and which stationary `mode` value is actually observed.
Zero or repeated stamps are tolerated during this read-only audit, but the
motion gate will reject them and they therefore block later supervised motion
until the source/firmware behavior is understood. Never guess an allowed mode.

A bounded evidence bag is optional:

```bash
bash scripts/record_go2_readonly.sh --duration 30 --label first-link
```

It records only the discovered state/sensor/TF allowlist under ignored
`logs/`; it never uses all-topic recording.

## 5. Optional onboard-computer SSH check

First obtain the IP, user and trusted host fingerprint from the operator. Do
not guess them, use `sshpass`, or save a password. A public-key-only read-only
probe is:

```bash
ONBOARD_IP='<OPERATOR_CONFIRMED_IP>'
ONBOARD_USER='<OPERATOR_CONFIRMED_USER>'
mkdir -p logs/go2-readonly/ssh
timeout --signal=INT --kill-after=2s 10s ssh -o BatchMode=yes -o PasswordAuthentication=no -o KbdInteractiveAuthentication=no -o StrictHostKeyChecking=ask -o UserKnownHostsFile="$PWD/logs/go2-readonly/ssh/known_hosts" "$ONBOARD_USER@$ONBOARD_IP" 'uname -a; ip -br addr; uptime'
```

After the operator confirms the account and host fingerprint and public-key
authentication succeeds, stream the repository's bounded audit script over
standard input. The script is not installed on the onboard computer and does
not use `sudo`, change time/network/services, inspect credentials, or include
process arguments:

```bash
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
timeout --signal=INT --kill-after=2s 180s ssh -o BatchMode=yes -o PasswordAuthentication=no -o KbdInteractiveAuthentication=no -o UserKnownHostsFile="$PWD/logs/go2-readonly/ssh/known_hosts" "$ONBOARD_USER@$ONBOARD_IP" 'bash -s' < scripts/audit_nx_readonly.sh > "logs/go2-readonly/ssh/${STAMP}-nx-audit.txt"
```

If interactive password authentication is the only option, the operator must
enter it directly in their own terminal after approving the connection. It
must never be copied into chat, `.env`, logs or this repository.

## 6. Separately approved recovery

Record current state first:

```bash
nmcli device status
nmcli connection show --active
ip -4 route show
```

Each operation below changes network state and requires a fresh approval:

Operation C disconnects the Go2 profile:

```bash
sudo nmcli connection down go2-readonly
```

Operation D removes the temporary profile:

```bash
sudo nmcli connection delete go2-readonly
```

Operation E is needed only when `ORIGINAL_PROFILE` was non-empty:

```bash
sudo nmcli connection up "$ORIGINAL_PROFILE" ifname "$IFACE"
```

Finally verify that addresses, DNS and default route match the baseline, then
clear only this shell's DDS environment:

```bash
ip -br addr
ip -4 route show
nmcli device status
nmcli --fields GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY,IP4.DNS device show "$IFACE"
unset CYCLONEDDS_URI RMW_IMPLEMENTATION
```

If recovery differs from the baseline, stop and report the read-only diff. Do
not improvise additional network changes.
