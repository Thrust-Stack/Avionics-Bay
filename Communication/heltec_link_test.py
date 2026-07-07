"""Capture and compare Heltec V4 point-to-point telemetry test results.

Run ``tx`` on the computer connected to the avionics transmitter and ``rx``
on the computer connected to the ground receiver. Copy both JSON reports to
one computer and run ``compare`` to calculate end-to-end packet delivery.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: install it with 'py -m pip install pyserial'", file=sys.stderr)
    raise SystemExit(2)


TX_SUCCESS_RE = re.compile(
    r"^TX id=(?P<id>\d+) time=(?P<time_ms>\d+) "
    r"alt=(?P<alt>-?[\d.]+)m batt=(?P<batt>[\d.]+)V "
    r"temp=(?P<temp>-?[\d.]+)C gps=(?P<gps>\d+)$"
)
TX_FAILURE_RE = re.compile(r"^Transmit failed for id=(?P<id>\d+): (?P<error>-?\d+)$")
RX_RE = re.compile(
    r"^(?P<id>\d+),(?P<time_ms>\d+),(?P<alt>-?[\d.]+),"
    r"(?P<batt>[\d.]+),(?P<temp>-?[\d.]+),(?P<gps>\d+),"
    r"(?P<rssi>-?[\d.]+),(?P<snr>-?[\d.]+),(?P<lost>\d+)$"
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def available_ports() -> str:
    ports = [port.device for port in list_ports.comports()]
    return ", ".join(ports) if ports else "none"


def parse_line(role: str, line: str) -> dict[str, Any] | None:
    if role == "tx":
        match = TX_SUCCESS_RE.match(line)
        if match:
            values = match.groupdict()
            return {
                "status": "sent",
                "id": int(values["id"]),
                "time_ms": int(values["time_ms"]),
                "altitude_m": float(values["alt"]),
                "battery_v": float(values["batt"]),
                "temperature_c": float(values["temp"]),
                "gps": int(values["gps"]),
            }
        match = TX_FAILURE_RE.match(line)
        if match:
            return {
                "status": "failed",
                "id": int(match.group("id")),
                "error": int(match.group("error")),
            }
        return None

    match = RX_RE.match(line)
    if not match:
        return None
    values = match.groupdict()
    return {
        "status": "received",
        "id": int(values["id"]),
        "time_ms": int(values["time_ms"]),
        "altitude_m": float(values["alt"]),
        "battery_v": float(values["batt"]),
        "temperature_c": float(values["temp"]),
        "gps": int(values["gps"]),
        "rssi_dbm": float(values["rssi"]),
        "snr_db": float(values["snr"]),
        "receiver_reported_lost": int(values["lost"]),
    }


def packet_rate(records: list[dict[str, Any]]) -> float | None:
    timed = [record for record in records if "time_ms" in record]
    if len(timed) < 2:
        return None
    elapsed_ms = timed[-1]["time_ms"] - timed[0]["time_ms"]
    if elapsed_ms <= 0:
        return None
    return (len(timed) - 1) * 1000.0 / elapsed_ms


def summarize(role: str, records: list[dict[str, Any]]) -> dict[str, Any]:
    successful = [record for record in records if record["status"] != "failed"]
    failed = [record for record in records if record["status"] == "failed"]
    ids = [record["id"] for record in successful]
    unique_ids = sorted(set(ids))
    summary: dict[str, Any] = {
        "packets": len(successful),
        "unique_packets": len(unique_ids),
        "duplicates": len(ids) - len(unique_ids),
        "first_packet_id": unique_ids[0] if unique_ids else None,
        "last_packet_id": unique_ids[-1] if unique_ids else None,
        "observed_rate_hz": packet_rate(successful),
        "failed_transmits": len(failed),
    }
    if role == "rx" and successful:
        summary.update(
            {
                "receiver_reported_lost": sum(
                    record["receiver_reported_lost"] for record in successful
                ),
                "average_rssi_dbm": statistics.fmean(
                    record["rssi_dbm"] for record in successful
                ),
                "minimum_rssi_dbm": min(record["rssi_dbm"] for record in successful),
                "maximum_rssi_dbm": max(record["rssi_dbm"] for record in successful),
                "average_snr_db": statistics.fmean(
                    record["snr_db"] for record in successful
                ),
            }
        )
    return summary


def capture(args: argparse.Namespace) -> int:
    started_utc = utc_now()
    records: list[dict[str, Any]] = []
    raw_unparsed = 0

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"Could not open {args.port}: {exc}", file=sys.stderr)
        print(f"Available ports: {available_ports()}", file=sys.stderr)
        return 2

    # Avoid resetting ESP32-S3 native USB devices when the host opens the port.
    port.dtr = False
    port.rts = False
    port.reset_input_buffer()

    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    print(
        f"Capturing {args.role.upper()} on {args.port} at {args.baud} baud "
        f"for {'until Ctrl+C' if deadline is None else f'{args.duration:g} seconds'}..."
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                line = port.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as exc:
                print(f"Serial read failed: {exc}", file=sys.stderr)
                return 2
            if not line:
                continue
            record = parse_line(args.role, line)
            if record is None:
                raw_unparsed += 1
                if args.verbose:
                    print(f"unparsed: {line}")
                continue
            records.append(record)
            if args.verbose:
                print(line)
    except KeyboardInterrupt:
        print("\nCapture stopped.")
    finally:
        port.close()

    report = {
        "format_version": 1,
        "role": args.role,
        "port": args.port,
        "baud": args.baud,
        "started_utc": started_utc,
        "finished_utc": utc_now(),
        "unparsed_lines": raw_unparsed,
        "summary": summarize(args.role, records),
        "records": records,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    summary = report["summary"]
    label = "sent" if args.role == "tx" else "received"
    print(f"Packets {label}: {summary['packets']}")
    print(f"Unique packet IDs: {summary['unique_packets']}")
    if summary["observed_rate_hz"] is not None:
        print(f"Observed rate: {summary['observed_rate_hz']:.3f} Hz")
    if args.role == "tx":
        print(f"Failed transmits: {summary['failed_transmits']}")
    elif summary["packets"]:
        print(f"Receiver-reported lost: {summary['receiver_reported_lost']}")
        print(f"Average RSSI: {summary['average_rssi_dbm']:.1f} dBm")
        print(f"Average SNR: {summary['average_snr_db']:.1f} dB")
    print(f"Report written to: {args.output.resolve()}")
    return 0


def load_report(path: Path, expected_role: str) -> dict[str, Any]:
    report = json.loads(path.read_text(encoding="utf-8"))
    if report.get("format_version") != 1 or report.get("role") != expected_role:
        raise ValueError(f"{path} is not a {expected_role.upper()} format-version 1 report")
    return report


def compare_reports(args: argparse.Namespace) -> int:
    try:
        tx = load_report(args.tx_report, "tx")
        rx = load_report(args.rx_report, "rx")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"Cannot compare reports: {exc}", file=sys.stderr)
        return 2

    sent_ids = {
        record["id"] for record in tx["records"] if record["status"] == "sent"
    }
    received_ids = {record["id"] for record in rx["records"]}
    if not sent_ids or not received_ids:
        print("Both reports need at least one successful packet.", file=sys.stderr)
        return 2

    # Use only the shared packet-ID window. This removes start/stop skew between
    # the two computers without requiring their wall clocks to be synchronized.
    window_start = max(min(sent_ids), min(received_ids))
    window_end = min(max(sent_ids), max(received_ids))
    if window_end < window_start:
        print("Reports do not contain an overlapping packet-ID range.", file=sys.stderr)
        return 2

    sent_in_window = {packet_id for packet_id in sent_ids if window_start <= packet_id <= window_end}
    received_in_window = {
        packet_id for packet_id in received_ids if window_start <= packet_id <= window_end
    }
    matched = sent_in_window & received_in_window
    missing = sorted(sent_in_window - received_in_window)
    delivery = 100.0 * len(matched) / len(sent_in_window) if sent_in_window else 0.0

    print(f"Compared packet IDs: {window_start} through {window_end}")
    print(f"Packets sent successfully: {len(sent_in_window)}")
    print(f"Packets received: {len(matched)}")
    print(f"Packets missing: {len(missing)}")
    print(f"Delivery rate: {delivery:.2f}%")
    if missing:
        preview = ", ".join(str(packet_id) for packet_id in missing[:20])
        suffix = " ..." if len(missing) > 20 else ""
        print(f"Missing packet IDs: {preview}{suffix}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    for role in ("tx", "rx"):
        capture_parser = subparsers.add_parser(role, help=f"capture {role.upper()} serial output")
        capture_parser.add_argument("--port", required=True, help="serial port, for example COM7")
        capture_parser.add_argument("--baud", type=int, default=115200)
        capture_parser.add_argument(
            "--duration",
            type=float,
            default=120.0,
            help="capture seconds; use 0 to run until Ctrl+C (default: 120)",
        )
        capture_parser.add_argument(
            "--output", type=Path, default=Path(f"{role}_report.json")
        )
        capture_parser.add_argument("--verbose", action="store_true")
        capture_parser.set_defaults(handler=capture, role=role)

    compare_parser = subparsers.add_parser("compare", help="compare TX and RX JSON reports")
    compare_parser.add_argument("tx_report", type=Path)
    compare_parser.add_argument("rx_report", type=Path)
    compare_parser.set_defaults(handler=compare_reports)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if hasattr(args, "duration") and args.duration < 0:
        print("--duration cannot be negative", file=sys.stderr)
        return 2
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
