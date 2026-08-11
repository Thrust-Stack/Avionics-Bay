"""Read ground-station Heltec telemetry from serial and append it to a log.

The paired Heltec receiver sketch forwards LoRa payloads as newline-delimited
USB serial text. Lines beginning with "# " are receiver diagnostics; telemetry
payloads from the avionics ESP32D are normally GPS NMEA, $IMU, $ALT, or $CTRL
lines.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

import serial


DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
DEFAULT_LOG_FILE = "telemetry_log.txt"


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
        help=f"Path to append received lines. Default: {DEFAULT_LOG_FILE}",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="Write raw received lines to the log without a timestamp prefix.",
    )
    return parser.parse_args()


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def main() -> int:
    args = parse_args()
    log_path = Path(args.log)

    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser, log_path.open(
            "a", encoding="utf-8"
        ) as log_file:
            print(
                f"Listening on {args.port} at {args.baud} baud. Logging to {log_path}",
                flush=True,
            )

            while True:
                raw_line = ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
                print(line, flush=True)

                if args.no_timestamp:
                    log_file.write(f"{line}\n")
                else:
                    log_file.write(f"{utc_timestamp()} {line}\n")
                log_file.flush()

    except serial.SerialException as exc:
        print(f"Serial error opening or reading {args.port}: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
