#!/usr/bin/env python3
"""Correlate a ROS 2 writer GID with source addresses in an offline PCAP.

``ros2 topic info --verbose`` exposes the DDS writer GID, but it does not
expose the publisher's IP address.  This tool performs the missing, explicit
evidence step: the first 12 bytes of the GID are matched to the RTPS GUID
prefix and the next four bytes are matched to DATA/DATA_FRAG writerEntityId.

The parser is deliberately small and read-only.  It supports classic PCAP
files containing Ethernet, Linux cooked, Linux cooked v2, raw IPv4, or BSD
loopback IPv4 frames.  It never opens a capture interface or a ROS graph.
"""

from __future__ import annotations

import argparse
from collections import Counter
import ipaddress
import json
import os
from pathlib import Path
import struct
import sys
from typing import Any, Iterator


MAX_CAPTURED_PACKET_BYTES = 16 * 1024 * 1024
RTPS_DATA = 0x15
RTPS_DATA_FRAG = 0x16


def parse_writer_gid(value: str) -> tuple[bytes, bytes, bytes]:
    """Return participant prefix, entity ID, and full rmw GID bytes."""

    compact = value.replace(".", "").replace(":", "").replace("-", "")
    compact = "".join(compact.split())
    if len(compact) < 32 or len(compact) % 2:
        raise ValueError("writer GID must contain at least 16 hexadecimal bytes")
    try:
        gid = bytes.fromhex(compact)
    except ValueError as error:
        raise ValueError("writer GID contains non-hexadecimal characters") from error
    return gid[:12], gid[12:16], gid


def _pcap_layout(stream: Any) -> tuple[str, int]:
    header = stream.read(24)
    if len(header) != 24:
        raise ValueError("truncated classic PCAP global header")
    magic = header[:4]
    layouts = {
        b"\xd4\xc3\xb2\xa1": "<",
        b"\xa1\xb2\xc3\xd4": ">",
        b"\x4d\x3c\xb2\xa1": "<",
        b"\xa1\xb2\x3c\x4d": ">",
    }
    if magic == b"\x0a\x0d\x0d\x0a":
        raise ValueError("PCAPNG is not supported; capture with tcpdump -w classic.pcap")
    if magic not in layouts:
        raise ValueError("unrecognized PCAP magic")
    endian = layouts[magic]
    _, major, minor, _, _, _, link_type = struct.unpack(
        f"{endian}IHHIIII", header
    )
    if (major, minor) != (2, 4):
        raise ValueError(f"unsupported PCAP version {major}.{minor}")
    return endian, link_type


def iter_pcap_packets(path: Path) -> Iterator[tuple[int, int, bytes]]:
    """Yield ``(packet_number, link_type, captured_bytes)``."""

    with path.open("rb") as stream:
        endian, link_type = _pcap_layout(stream)
        packet_number = 0
        while True:
            record_header = stream.read(16)
            if not record_header:
                return
            if len(record_header) != 16:
                raise ValueError("truncated PCAP packet header")
            _, _, captured_length, original_length = struct.unpack(
                f"{endian}IIII", record_header
            )
            if captured_length > MAX_CAPTURED_PACKET_BYTES:
                raise ValueError(
                    f"captured packet exceeds {MAX_CAPTURED_PACKET_BYTES} bytes"
                )
            if original_length < captured_length:
                raise ValueError("PCAP original length is smaller than captured length")
            packet = stream.read(captured_length)
            if len(packet) != captured_length:
                raise ValueError("truncated PCAP packet body")
            packet_number += 1
            yield packet_number, link_type, packet


def _ipv4_payload(link_type: int, packet: bytes) -> bytes | None:
    # LINKTYPE_ETHERNET
    if link_type == 1:
        if len(packet) < 14:
            return None
        offset = 14
        ether_type = int.from_bytes(packet[12:14], "big")
        while ether_type in (0x8100, 0x88A8, 0x9100):
            if len(packet) < offset + 4:
                return None
            ether_type = int.from_bytes(packet[offset + 2 : offset + 4], "big")
            offset += 4
        return packet[offset:] if ether_type == 0x0800 else None

    # LINKTYPE_LINUX_SLL and LINKTYPE_LINUX_SLL2
    if link_type == 113:
        if len(packet) < 16 or packet[14:16] != b"\x08\x00":
            return None
        return packet[16:]
    if link_type == 276:
        if len(packet) < 20 or packet[0:2] != b"\x08\x00":
            return None
        return packet[20:]

    # LINKTYPE_RAW and LINKTYPE_IPV4
    if link_type in (101, 228):
        return packet

    # LINKTYPE_NULL: host-endian AF_INET is normally 2.  Accept either byte
    # order because the enclosing PCAP byte order does not define this field.
    if link_type == 0 and len(packet) >= 4:
        if packet[:4] in (b"\x02\x00\x00\x00", b"\x00\x00\x00\x02"):
            return packet[4:]
    return None


def _udp_datagram(ip_packet: bytes) -> dict[str, Any] | None:
    if len(ip_packet) < 20 or ip_packet[0] >> 4 != 4:
        return None
    header_length = (ip_packet[0] & 0x0F) * 4
    if header_length < 20 or len(ip_packet) < header_length + 8:
        return None
    total_length = int.from_bytes(ip_packet[2:4], "big")
    if total_length < header_length + 8:
        return None
    total_length = min(total_length, len(ip_packet))
    fragment = int.from_bytes(ip_packet[6:8], "big")
    if fragment & 0x1FFF or ip_packet[9] != 17:
        return None
    udp = ip_packet[header_length:total_length]
    udp_length = int.from_bytes(udp[4:6], "big")
    if udp_length < 8:
        return None
    udp = udp[: min(udp_length, len(udp))]
    return {
        "source_ip": str(ipaddress.IPv4Address(ip_packet[12:16])),
        "destination_ip": str(ipaddress.IPv4Address(ip_packet[16:20])),
        "source_port": int.from_bytes(udp[0:2], "big"),
        "destination_port": int.from_bytes(udp[2:4], "big"),
        "payload": udp[8:],
    }


def _rtps_data_writer_ids(payload: bytes) -> list[bytes]:
    """Return writer entity IDs from RTPS DATA and DATA_FRAG submessages."""

    if len(payload) < 20 or payload[:4] != b"RTPS":
        return []
    writers: list[bytes] = []
    offset = 20
    while offset + 4 <= len(payload):
        submessage_id = payload[offset]
        flags = payload[offset + 1]
        byte_order = "little" if flags & 0x01 else "big"
        octets_to_next = int.from_bytes(payload[offset + 2 : offset + 4], byte_order)
        body_start = offset + 4
        body_end = len(payload) if octets_to_next == 0 else body_start + octets_to_next
        is_data = submessage_id in (RTPS_DATA, RTPS_DATA_FRAG)
        declared_writer_present = octets_to_next == 0 or octets_to_next >= 12
        captured_writer_present = len(payload) >= body_start + 12
        if is_data and declared_writer_present and captured_writer_present:
            writers.append(payload[body_start + 8 : body_start + 12])
        # A classic PCAP snaplen can truncate a valid UDP/RTPS datagram after
        # the fixed DATA/DATA_FRAG fields.  The writerEntityId above is still
        # usable when both the declared and captured body cover it, but no
        # later submessage boundary can be trusted once the current body is
        # incomplete.
        if body_end > len(payload):
            break
        if octets_to_next == 0:
            break
        offset = body_end
    return writers


def correlate(path: Path, writer_gid: str) -> dict[str, Any]:
    prefix, entity_id, gid = parse_writer_gid(writer_gid)
    prefix_sources: Counter[str] = Counter()
    writer_sources: Counter[str] = Counter()
    prefix_packets: list[dict[str, Any]] = []
    writer_packets: list[dict[str, Any]] = []
    total_packets = 0
    decoded_udp_packets = 0

    for packet_number, link_type, frame in iter_pcap_packets(path):
        total_packets += 1
        ip_packet = _ipv4_payload(link_type, frame)
        if ip_packet is None:
            continue
        datagram = _udp_datagram(ip_packet)
        if datagram is None:
            continue
        decoded_udp_packets += 1
        payload = datagram.pop("payload")
        if len(payload) < 20 or payload[:4] != b"RTPS":
            continue
        if payload[8:20] != prefix:
            continue
        source = datagram["source_ip"]
        prefix_sources[source] += 1
        packet_record = {"packet_number": packet_number, **datagram}
        if len(prefix_packets) < 32:
            prefix_packets.append(packet_record)
        if entity_id in _rtps_data_writer_ids(payload):
            writer_sources[source] += 1
            if len(writer_packets) < 32:
                writer_packets.append(packet_record)

    proven_sources = sorted(writer_sources)
    if len(proven_sources) == 1:
        conclusion = "single_source_proven_by_rtps_data_writer"
    elif len(proven_sources) > 1:
        conclusion = "multiple_sources_observed_for_writer_gid"
    elif prefix_sources:
        conclusion = "participant_prefix_seen_but_writer_data_not_proven"
    else:
        conclusion = "writer_participant_prefix_not_seen"
    return {
        "schema_version": 1,
        "method": "pcap_rtps_guid_prefix_and_data_writer_entity_correlation",
        "input_pcap": str(path),
        "writer_gid": gid.hex("."),
        "rtps_participant_guid_prefix": prefix.hex(),
        "rtps_writer_entity_id": entity_id.hex(),
        "ros_topic_info_provides_ip": False,
        "total_packets": total_packets,
        "decoded_ipv4_udp_packets": decoded_udp_packets,
        "participant_prefix_sources": dict(sorted(prefix_sources.items())),
        "writer_data_sources": dict(sorted(writer_sources.items())),
        "participant_prefix_packet_examples": prefix_packets,
        "writer_data_packet_examples": writer_packets,
        "proven_source_ips": proven_sources,
        "conclusion": conclusion,
        "caveat": (
            "A ROS 2 writer GID can change after publisher or robot restart; "
            "collect topic-info and packet evidence in the same session."
        ),
    }


def _write_private_json(path: Path, value: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(f"refusing to overwrite existing evidence: {path}")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.chmod(path, 0o600)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline ROS writer-GID to RTPS source-IP correlation"
    )
    parser.add_argument("--pcap", required=True, type=Path)
    parser.add_argument("--writer-gid", required=True)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        result = correlate(arguments.pcap.expanduser().resolve(), arguments.writer_gid)
        if arguments.output:
            _write_private_json(arguments.output, result)
        else:
            json.dump(result, sys.stdout, ensure_ascii=False, indent=2, sort_keys=True)
            sys.stdout.write("\n")
    except (OSError, ValueError) as error:
        print(f"correlation failed: {error}", file=sys.stderr)
        return 2
    return 0 if result["conclusion"] == "single_source_proven_by_rtps_data_writer" else 3


if __name__ == "__main__":
    raise SystemExit(main())
