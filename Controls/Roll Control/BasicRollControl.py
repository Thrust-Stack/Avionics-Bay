import argparse
import glob
import math
import os
import sqlite3
import time
from dataclasses import dataclass

import serial

# -----------------------------
# Configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0        # deg/s
KP = 0.02                     # proportional gain
MAX_FIN_DEFLECTION = 7.5     # degrees

SERIAL_PORT = "COM3"          # command link to ESP32; change for your machine
BAUD_RATE = 115200

# GPSReader.py stores sessions in repo_root/logs/flight_YYYYMMDD_HHMMSS.db.
TELEMETRY_DB_PATH = None      # set to a specific .db path, or leave None for latest

# The ESP32 bridge prints Adafruit MPU gyro values. Those are radians/second.
GYRO_INPUT_UNITS = "rad/s"

# -----------------------------
# Safety Parameters
# -----------------------------

MIN_CONTROL_ALTITUDE_M = 20.0

MAX_X_ROTATION_DEG = 90.0
MAX_Y_ROTATION_DEG = 90.0

SERVO_NEUTRAL_COMMAND = 0.0

# -----------------------------
# Data models
# -----------------------------


@dataclass
class TelemetrySnapshot:
    timestamp: str | None = None
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

    def __init__(self, db_path=None):
        self.db_path = db_path or find_latest_telemetry_db()
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
        snapshot = TelemetrySnapshot()

        imu = self._latest_row("imu")
        if imu:
            snapshot.timestamp = imu["timestamp"]
            snapshot.accel_x = imu["accel_x"]
            snapshot.accel_y = imu["accel_y"]
            snapshot.accel_z = imu["accel_z"]
            snapshot.gyro_x = normalize_gyro(imu["gyro_x"])
            snapshot.gyro_y = normalize_gyro(imu["gyro_y"])
            snapshot.gyro_z = normalize_gyro(imu["gyro_z"])

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


def normalize_gyro(value):
    if GYRO_INPUT_UNITS == "rad/s":
        return math.degrees(value)
    return value


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def estimate_rotation_x_y(snapshot):
    """
    Estimate tilt from accelerometer data.

    This is usable for ground testing and slow motion. During powered flight,
    acceleration can make accelerometer-only attitude estimates inaccurate.
    """
    if None in (snapshot.accel_x, snapshot.accel_y, snapshot.accel_z):
        return 0.0, 0.0

    ax = snapshot.accel_x
    ay = snapshot.accel_y
    az = snapshot.accel_z

    rotation_x_deg = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
    rotation_y_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    return rotation_x_deg, rotation_y_deg


def control_is_allowed(altitude_m, rotation_x_deg, rotation_y_deg):
    """
    Roll control is only allowed if:
    - altitude is above 20 meters
    - x rotation is within +/- 90 degrees
    - y rotation is within +/- 90 degrees
    """
    if altitude_m is None:
        return False

    altitude_ok = altitude_m > MIN_CONTROL_ALTITUDE_M
    x_rotation_ok = abs(rotation_x_deg) < MAX_X_ROTATION_DEG
    y_rotation_ok = abs(rotation_y_deg) < MAX_Y_ROTATION_DEG

    return altitude_ok and x_rotation_ok and y_rotation_ok


def calculate_fin_command(roll_rate):
    error = TARGET_ROLL_RATE - roll_rate
    command = KP * error
    command = clamp(command, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)

    return command


def send_command_to_esp32(esp32, fin_command):
    message = f"ROLL,{fin_command:.2f}\n"
    esp32.write(message.encode("utf-8"))


def open_command_link(port):
    if port is None:
        return None
    return serial.Serial(port, BAUD_RATE, timeout=0.1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Roll controller using telemetry written by GPSReader.py."
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
    return parser.parse_args()


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    command_port = None if str(args.port).lower() == "none" else args.port
    telemetry = GPSReaderTelemetry(args.db)
    esp32 = open_command_link(command_port)

    try:
        while True:
            snapshot = telemetry.latest()

            rotation_x_deg, rotation_y_deg = estimate_rotation_x_y(snapshot)

            # Choose the gyro axis that matches the rocket's roll axis.
            # The current ESP32 bridge wiring assumes gyro_z is roll rate.
            roll_rate = snapshot.gyro_z or 0.0

            allowed = control_is_allowed(
                snapshot.altitude_m,
                rotation_x_deg,
                rotation_y_deg
            )

            state = "CONTROL_ACTIVE" if allowed else "IDLE"
            fin_command = (
                calculate_fin_command(roll_rate)
                if state == "CONTROL_ACTIVE"
                else SERVO_NEUTRAL_COMMAND
            )

            if esp32:
                send_command_to_esp32(esp32, fin_command)

            time.sleep(0.01)  # 100 Hz loop

    except KeyboardInterrupt:
        if esp32:
            send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
    finally:
        telemetry.close()
        if esp32:
            esp32.close()


if __name__ == "__main__":
    main()
