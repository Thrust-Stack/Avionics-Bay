"""Read ground-station Heltec telemetry from serial and append it to a log.

The paired Heltec receiver sketch forwards LoRa payloads as newline-delimited
USB serial text. Lines beginning with "# " are receiver diagnostics; telemetry
payloads from the avionics ESP32D are normally GPS NMEA, $IMU, or $ALT lines.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import time

import serial


DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
DEFAULT_LOG_FILE = Path(__file__).with_name("telemetry_log.txt")
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
        help=f"Path to append received lines. Default: {DEFAULT_LOG_FILE}",
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


def main() -> int:
    args = parse_args()
    log_path = args.log

    try:
        with log_path.open("a", encoding="utf-8") as log_file:
            while True:
                try:
                    with open_serial_port(args.port, args.baud) as ser:
                        print(
                            (
                                f"Listening on {args.port} at {args.baud} baud. "
                                f"Logging to {log_path}"
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
                            print(line, flush=True)

                            if args.no_timestamp:
                                log_file.write(f"{line}\n")
                            else:
                                log_file.write(f"{utc_timestamp()} {line}\n")
                            log_file.flush()

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
