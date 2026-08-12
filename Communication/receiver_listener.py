"""Read ground-station Heltec telemetry from serial and append it to a log.

This listener supports dual logging:
 - Raw text log (optional, for debugging) -- mirrors existing behavior.
 - Structured CSV log (recommended) -- each row contains timestamp, packet
     type, raw packet, and a JSON-encoded `parsed` field with extracted values.

The paired Heltec receiver sketch now decodes a compact binary LoRa frame on
the radio and prints a human- and machine-parseable ASCII line over USB
containing the decoded fields. This listener also accepts the older text-based
packets (GPS NMEA, $IMU, $ALT, $CTRL) and will extract basic fields from them.

CLI flags `--csv` and `--raw` control output paths. Keep reconnect handling
so the listener continues retrying if the USB device disconnects.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import time
import csv
import json
from typing import Any, Dict

import serial


DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
DEFAULT_LOG_FILE = Path(__file__).with_name("telemetry_log.txt")
DEFAULT_CSV_FILE = Path(__file__).with_name("telemetry_log.csv")
DEFAULT_RECONNECT_DELAY_S = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Listen to a ground-station Heltec serial port and log telemetry."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port to open, for example COM7 or COM9. Default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Default: {DEFAULT_BAUD}",
    )
    parser.add_argument(
        "--log",
        default=DEFAULT_LOG_FILE,
        type=Path,
        help=f"Path to append raw received lines. Default: {DEFAULT_LOG_FILE}",
    )
    parser.add_argument(
        "--csv",
        default=DEFAULT_CSV_FILE,
        type=Path,
        help=(
            f"Path to append structured CSV rows. Default: {DEFAULT_CSV_FILE}"
        ),
    )
    parser.add_argument(
        "--no-raw",
        action="store_true",
        help="Do not write the raw text log; useful when only CSV is desired.",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="Write raw received lines to the log without a timestamp prefix.",
    )
    parser.add_argument(
        "--reconnect-delay",
        default=DEFAULT_RECONNECT_DELAY_S,
        type=float,
        help=(
            "Seconds to wait before reopening the serial port after a USB "
            f"disconnect. Default: {DEFAULT_RECONNECT_DELAY_S}"
        ),
    )
    return parser.parse_args()


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def open_serial_port(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 1
    ser.rtscts = False
    ser.dsrdtr = False
    ser.rts = False
    ser.dtr = True
    ser.open()
    return ser


def _nmea_coord_to_deg(coord: str, hemi: str) -> float | None:
    # Convert NMEA lat/lon in ddmm.mmmm (lat) or dddmm.mmmm (lon) to degrees.
    try:
        if not coord:
            return None
        dot = coord.find('.')
        if dot == -1:
            return None
        degrees_len = dot - 2
        degrees = float(coord[:degrees_len])
        minutes = float(coord[degrees_len:])
        deg = degrees + minutes / 60.0
        if hemi in ('S', 'W'):
            deg = -deg
        return deg
    except Exception:
        return None


def parse_gprmc(fields: list[str]) -> Dict[str, Any]:
    # GPRMC: time, status, lat, N/S, lon, E/W, speed(knots), track, date, ...
    out: Dict[str, Any] = {}
    try:
        out['nmea_time'] = fields[1]
        out['status'] = fields[2]
        lat = _nmea_coord_to_deg(fields[3], fields[4])
        lon = _nmea_coord_to_deg(fields[5], fields[6])
        out['lat'] = lat
        out['lon'] = lon
        # speed in knots -> m/s
        spd = None
        if fields[7]:
            spd = float(fields[7]) * 0.514444
        out['speed_m_s'] = spd
        out['track_deg'] = float(fields[8]) if fields[8] else None
    except Exception:
        pass
    return out


def parse_gpgga(fields: list[str]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    try:
        lat = _nmea_coord_to_deg(fields[2], fields[3])
        lon = _nmea_coord_to_deg(fields[4], fields[5])
        out['lat'] = lat
        out['lon'] = lon
        out['fix_quality'] = int(fields[6]) if fields[6] else None
        out['num_satellites'] = int(fields[7]) if fields[7] else None
    except Exception:
        pass
    return out


def parse_packet(line: str) -> tuple[str, Dict[str, Any]]:
    """Return (packet_type, parsed_fields).

    packet_type is a short token like 'IMU', 'ALT', 'GPRMC', 'BIN', or 'RAW'.
    parsed_fields is a dict with extracted numerical/string values.
    """
    parsed: Dict[str, Any] = {}
    if not line:
        return "", parsed

    if line.startswith('#'):
        return 'DIAG', {'msg': line[1:].strip()}

    if line.startswith('TRACK,') or line.startswith('EVENT,'):
        try:
            token, rest = line.split(',', 1)
            for part in rest.split(','):
                if '=' not in part:
                    continue
                k, v = part.split('=', 1)
                try:
                    if '.' in v:
                        parsed[k] = float(v)
                    else:
                        parsed[k] = int(v, 0)
                except Exception:
                    parsed[k] = v
            return token, parsed
        except Exception:
            return 'RAW', {'raw': line}

    if line.startswith('BIN,'):
        # Binary-decoded printer from the Heltec receiver: comma-separated
        # key=value pairs after the 'BIN,' prefix.
        try:
            rest = line[4:]
            for part in rest.split(','):
                if '=' in part:
                    k, v = part.split('=', 1)
                    # try to coerce numbers
                    try:
                        if '.' in v:
                            parsed[k] = float(v)
                        else:
                            parsed[k] = int(v, 0)
                    except Exception:
                        parsed[k] = v
        except Exception:
            pass
        return 'BIN', parsed

    if line.startswith('$'):
        # NMEA or simple $IMU/$ALT/$CTRL tokens
        try:
            token, rest = (line.split(',', 1) + [''])[:2]
            token = token.lstrip('$')
            fields = ([token] + rest.split(',')) if rest else [token]
            if token == 'IMU':
                # $IMU,ax,ay,az,gx,gy,gz
                vals = rest.split(',')
                keys = ['ax','ay','az','gx','gy','gz']
                for k, v in zip(keys, vals):
                    try:
                        parsed[k] = float(v)
                    except Exception:
                        parsed[k] = None
                return 'IMU', parsed
            if token == 'ALT':
                try:
                    parsed['alt_m'] = float(rest.strip())
                except Exception:
                    parsed['alt_m'] = None
                return 'ALT', parsed
            if token == 'CTRL':
                # $CTRL,seq,craft,mode, ... rest numeric
                vals = rest.split(',')
                try:
                    parsed['seq'] = int(vals[0])
                except Exception:
                    parsed['seq'] = None
                parsed['craft'] = vals[1] if len(vals) > 1 else None
                parsed['mode'] = vals[2] if len(vals) > 2 else None
                # remaining numeric values lumped
                parsed['values'] = []
                for v in vals[3:]:
                    try:
                        parsed['values'].append(float(v))
                    except Exception:
                        parsed['values'].append(v)
                return 'CTRL', parsed
            # Common NMEA messages
            if token in ('GPRMC', 'GNRMC'):
                fields = line.split(',')
                parsed.update(parse_gprmc(fields))
                return 'GPRMC', parsed
            if token == 'GPGGA':
                fields = line.split(',')
                parsed.update(parse_gpgga(fields))
                return 'GPGGA', parsed
        except Exception:
            return 'RAW', {'raw': line}

    # Fallback: unknown plain text
    return 'RAW', {'raw': line}


def make_summary(pkt_type: str, parsed: Dict[str, Any]) -> str | None:
    # Produce compact human-readable summaries for common packet types.
    try:
        if pkt_type == 'TRACK':
            lat = parsed.get('lat')
            lon = parsed.get('lon')
            alt = parsed.get('alt_m')
            if lat is not None and lon is not None and alt is not None:
                return f"TRACKING MODE ACTIVE GPS {lat:.7f},{lon:.7f} ALT {alt:.1f} m"
            return "TRACKING MODE ACTIVE"
        if pkt_type == 'EVENT':
            msg = parsed.get('msg')
            mode = parsed.get('MODE')
            peak = parsed.get('peak_m')
            parts = []
            if msg:
                parts.append(str(msg))
            if mode:
                parts.append(f"mode={mode}")
            if peak is not None:
                parts.append(f"peak={peak:.1f} m" if isinstance(peak, float) else f"peak={peak} m")
            return " ".join(parts) if parts else None
        if pkt_type == 'BIN' or pkt_type == 'GPRMC' or pkt_type == 'GPGGA':
            lat = parsed.get('lat')
            lon = parsed.get('lon')
            spd = parsed.get('speed_m_s')
            hdg = parsed.get('track_deg') or parsed.get('heading')
            parts = []
            if lat is not None and lon is not None:
                parts.append(f"GPS {lat:.6f},{lon:.6f}")
            if spd is not None:
                parts.append(f"{spd:.1f}m/s")
            if hdg is not None:
                parts.append(f"{hdg:.1f}°")
            return ' '.join(parts) if parts else None
        if pkt_type == 'IMU':
            ax = parsed.get('ax'); ay = parsed.get('ay'); az = parsed.get('az')
            return f"IMU a={ax:.3f},{ay:.3f},{az:.3f}" if ax is not None else 'IMU'
        if pkt_type == 'ALT':
            alt = parsed.get('alt_m')
            return f"ALT {alt:.2f} m" if alt is not None else 'ALT'
        if pkt_type == 'CTRL':
            seq = parsed.get('seq')
            vals = parsed.get('values', [])
            hdg = None
            if len(vals) >= 9:
                # heuristically pick a heading-like field near the end
                hdg = vals[7]
            return f"CTRL seq={seq} hdg={hdg}" if seq is not None else 'CTRL'
    except Exception:
        return None
    return None


def main() -> int:
    args = parse_args()
    log_path = args.log
    csv_path = args.csv
    write_raw = not args.no_raw

    # Open files and CSV writer (append mode). CSV has columns: timestamp, type,
    # raw, parsed (JSON string).
    try:
        csv_file = csv_path.open("a", encoding="utf-8", newline="")
        csv_writer = csv.writer(csv_file)
        # If file was empty, write header.
        if csv_file.tell() == 0:
            csv_writer.writerow(["timestamp", "type", "raw", "parsed"])

        raw_file = None
        if write_raw:
            raw_file = log_path.open("a", encoding="utf-8")

        while True:
            try:
                with open_serial_port(args.port, args.baud) as ser:
                    print(
                        (
                            f"Listening on {args.port} at {args.baud} baud. "
                            f"Logging raw={write_raw} csv={csv_path}"
                        ),
                        flush=True,
                    )

                    while True:
                        raw_line = ser.readline()
                        if not raw_line:
                            continue

                        line = raw_line.decode("utf-8", errors="replace").rstrip(
                            "\r\n"
                        )

                        # Parse packet type and produce compact terminal summary
                        pkt_type, parsed = parse_packet(line)
                        summary = make_summary(pkt_type, parsed)
                        if summary:
                            print(summary, flush=True)
                        else:
                            print(line, flush=True)

                        ts = "" if args.no_timestamp else utc_timestamp()

                        if write_raw and raw_file is not None:
                            if args.no_timestamp:
                                raw_file.write(f"{line}\n")
                            else:
                                raw_file.write(f"{ts} {line}\n")
                            raw_file.flush()

                        # Write CSV row: parsed fields as compact JSON
                        parsed_json = json.dumps(parsed, separators=(',', ':'))
                        csv_writer.writerow([ts, pkt_type, line, parsed_json])
                        csv_file.flush()

            except serial.SerialException as exc:
                print(
                    (
                        f"Serial error on {args.port}: {exc}. "
                        f"Retrying in {args.reconnect_delay:.1f}s..."
                    ),
                    file=sys.stderr,
                    flush=True,
                )
                time.sleep(max(args.reconnect_delay, 0.1))

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
