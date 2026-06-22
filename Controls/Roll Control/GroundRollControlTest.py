import argparse
import csv
import glob
import math
import os
import sqlite3
import time
from dataclasses import dataclass

import serial

# -----------------------------
# Ground test configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0

# Aggressive ground-test gains. With the defaults, about 11 deg/s of roll-rate
# error or 8 degrees of tilt is enough to saturate the command.
ROLL_RATE_GAIN = 1.4
TILT_GAIN = 2.0
MAX_FIN_DEFLECTION = 15.0

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

TELEMETRY_DB_PATH = None

# GPSReader.py logs gyro values in deg/s. Use --gyro-units rad/s if your bridge
# is sending raw Adafruit gyro radians/second instead.
GYRO_INPUT_UNITS = "deg/s"

SERVO_NEUTRAL_COMMAND = 0.0
CONTROL_LOOP_DELAY_S = 0.02
STALE_TELEMETRY_TIMEOUT_S = 0.5

# -----------------------------
# Data models
# -----------------------------


@dataclass
class TelemetrySnapshot:
    timestamp: str | None = None
    monotonic_read_time: float | None = None
    altitude_m: float | None = None
    accel_x: float | None = None
    accel_y: float | None = None
    accel_z: float | None = None
    gyro_x: float | None = None
    gyro_y: float | None = None
    gyro_z: float | None = None
    lat: float | None = None
    lon: float | None = None
    speed_mph: float | None = None
    heading_deg: float | None = None


class GPSReaderTelemetry:
    """Reads the newest telemetry rows written by GPSReader.py."""

    def __init__(self, db_path=None, gyro_units=GYRO_INPUT_UNITS):
        self.db_path = db_path or find_latest_telemetry_db()
        self.gyro_units = gyro_units
        if not self.db_path:
            raise FileNotFoundError(
                "No GPSReader database found in logs/. Run GPSReader.py first "
                "or pass --db path\\to\\flight.db."
            )

        self.conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        self.conn.row_factory = sqlite3.Row

    def close(self):
        self.conn.close()

    def latest(self):
        snapshot = TelemetrySnapshot(monotonic_read_time=time.monotonic())

        imu = self._latest_row("imu")
        if imu:
            snapshot.timestamp = imu["timestamp"]
            snapshot.accel_x = imu["accel_x"]
            snapshot.accel_y = imu["accel_y"]
            snapshot.accel_z = imu["accel_z"]
            snapshot.gyro_x = normalize_gyro(imu["gyro_x"], self.gyro_units)
            snapshot.gyro_y = normalize_gyro(imu["gyro_y"], self.gyro_units)
            snapshot.gyro_z = normalize_gyro(imu["gyro_z"], self.gyro_units)

        alt = self._latest_row("alt")
        if alt:
            snapshot.altitude_m = alt["agl_m"]
            snapshot.timestamp = newest_timestamp(snapshot.timestamp, alt["timestamp"])

        gps = self._latest_row("gps")
        if gps:
            snapshot.lat = signed_coordinate(gps["lat"], gps["lat_dir"])
            snapshot.lon = signed_coordinate(gps["lon"], gps["lon_dir"])
            snapshot.speed_mph = gps["speed_mph"]
            snapshot.heading_deg = gps["heading_deg"]
            snapshot.timestamp = newest_timestamp(snapshot.timestamp, gps["timestamp"])

        return snapshot

    def _latest_row(self, table):
        return self.conn.execute(
            f"SELECT * FROM {table} ORDER BY id DESC LIMIT 1"
        ).fetchone()


# -----------------------------
# Helper functions
# -----------------------------


def repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def find_latest_telemetry_db():
    logs_dir = os.path.join(repo_root(), "logs")
    db_files = glob.glob(os.path.join(logs_dir, "flight_*.db"))
    if not db_files:
        return None
    return max(db_files, key=os.path.getmtime)


def newest_timestamp(current, candidate):
    if current is None:
        return candidate
    if candidate is None:
        return current
    return max(current, candidate)


def signed_coordinate(value, direction):
    if value is None:
        return None
    if direction in ("S", "W"):
        return -value
    return value


def normalize_gyro(value, units):
    if units == "rad/s":
        return math.degrees(value)
    return value


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def estimate_rotation_x_y(snapshot):
    """
    Estimate tilt from accelerometer data.

    This is useful for ground handling and slow movement. Do not use this as a
    flight attitude source during powered flight.
    """
    if None in (snapshot.accel_x, snapshot.accel_y, snapshot.accel_z):
        return 0.0, 0.0

    ax = snapshot.accel_x
    ay = snapshot.accel_y
    az = snapshot.accel_z

    rotation_x_deg = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
    rotation_y_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    return rotation_x_deg, rotation_y_deg


def calculate_ground_test_command(roll_rate, rotation_x_deg, rotation_y_deg):
    """
    Aggressively command fins from both roll rate and body tilt.

    The gyro term reacts to quick rotation around the configured roll axis.
    The tilt term makes slow hand rotation obvious during bench testing.
    """
    roll_rate_error = TARGET_ROLL_RATE - roll_rate
    dominant_tilt = rotation_y_deg

    command = (ROLL_RATE_GAIN * roll_rate_error) - (TILT_GAIN * dominant_tilt)
    return clamp(command, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)


def send_command_to_esp32(esp32, fin_command):
    message = f"ROLL,{fin_command:.2f}\n"
    esp32.write(message.encode("utf-8"))


def open_command_link(port):
    if port is None:
        return None
    return serial.Serial(port, BAUD_RATE, timeout=0.1)


def write_log_header(logger):
    logger.writerow([
        "time",
        "telemetry_timestamp",
        "state",
        "altitude_m",
        "rotation_x_deg",
        "rotation_y_deg",
        "gyro_x_deg_s",
        "gyro_y_deg_s",
        "gyro_z_deg_s",
        "roll_rate_deg_s",
        "lat",
        "lon",
        "speed_mph",
        "heading_deg",
        "fin_command"
    ])


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Aggressive ground-test roll controller. This bypasses the flight "
            "altitude gate and should not be used as the flight controller."
        )
    )
    parser.add_argument(
        "--db",
        default=TELEMETRY_DB_PATH,
        help="Path to a GPSReader flight_*.db file. Defaults to latest in logs/."
    )
    parser.add_argument(
        "--port",
        default=SERIAL_PORT,
        help="Serial port used to send fin commands to the ESP32. Use 'none' to disable."
    )
    parser.add_argument(
        "--log",
        default="ground_roll_test_log.csv",
        help="CSV path for ground-test log output."
    )
    parser.add_argument(
        "--gyro-units",
        choices=("deg/s", "rad/s"),
        default=GYRO_INPUT_UNITS,
        help="Units stored in the imu gyro columns."
    )
    parser.add_argument(
        "--roll-axis",
        choices=("x", "y", "z"),
        default="z",
        help="Gyro axis to treat as rocket roll rate."
    )
    return parser.parse_args()


def select_roll_rate(snapshot, axis):
    if axis == "x":
        return snapshot.gyro_x or 0.0
    if axis == "y":
        return snapshot.gyro_y or 0.0
    return snapshot.gyro_z or 0.0


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    command_port = None if str(args.port).lower() == "none" else args.port
    telemetry = GPSReaderTelemetry(args.db, gyro_units=args.gyro_units)
    esp32 = open_command_link(command_port)

    print("Ground roll-control test is running.")
    print("This bypasses altitude safety logic and uses aggressive gains.")
    print(f"Database: {telemetry.db_path}")
    print(f"Command port: {command_port or 'disabled'}")
    print("Press Ctrl+C to stop and send neutral.")

    with open(args.log, "w", newline="") as log_file:
        logger = csv.writer(log_file)
        write_log_header(logger)

        last_timestamp = None
        last_fresh_read = time.monotonic()

        try:
            while True:
                now = time.time()
                snapshot = telemetry.latest()

                if snapshot.timestamp != last_timestamp:
                    last_timestamp = snapshot.timestamp
                    last_fresh_read = time.monotonic()

                rotation_x_deg, rotation_y_deg = estimate_rotation_x_y(snapshot)

                gyro_x = snapshot.gyro_x or 0.0
                gyro_y = snapshot.gyro_y or 0.0
                gyro_z = snapshot.gyro_z or 0.0
                roll_rate = select_roll_rate(snapshot, args.roll_axis)

                telemetry_is_fresh = (
                    time.monotonic() - last_fresh_read
                ) <= STALE_TELEMETRY_TIMEOUT_S

                state = "GROUND_TEST_ACTIVE" if telemetry_is_fresh else "STALE_TELEMETRY"
                fin_command = (
                    calculate_ground_test_command(
                        roll_rate,
                        rotation_x_deg,
                        rotation_y_deg
                    )
                    if telemetry_is_fresh
                    else SERVO_NEUTRAL_COMMAND
                )

                if esp32:
                    send_command_to_esp32(esp32, fin_command)

                logger.writerow([
                    now,
                    snapshot.timestamp,
                    state,
                    snapshot.altitude_m,
                    rotation_x_deg,
                    rotation_y_deg,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    roll_rate,
                    snapshot.lat,
                    snapshot.lon,
                    snapshot.speed_mph,
                    snapshot.heading_deg,
                    fin_command
                ])

                log_file.flush()

                print(
                    f"{state} roll_rate={roll_rate:7.2f} deg/s "
                    f"rot_x={rotation_x_deg:7.2f} rot_y={rotation_y_deg:7.2f} "
                    f"cmd={fin_command:6.2f}",
                    end="\r",
                    flush=True
                )

                time.sleep(CONTROL_LOOP_DELAY_S)

        except KeyboardInterrupt:
            if esp32:
                send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
            print("\nStopped. Neutral command sent.")
        finally:
            telemetry.close()
            if esp32:
                esp32.close()


if __name__ == "__main__":
    main()
