import argparse
import datetime
import glob
import math
import os
import socket
import sqlite3
import time
from dataclasses import dataclass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR   = os.path.join(SCRIPT_DIR, "logs")
DB_PATH    = os.path.join(LOGS_DIR, "GRCTRollData.db")

# -----------------------------
# Ground test configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0

# Aggressive ground-test gains. With the defaults, about 11 deg/s of roll-rate
# error or 8 degrees of tilt is enough to saturate the command.
ROLL_RATE_GAIN = 1.4
TILT_GAIN = 2.0
MAX_FIN_DEFLECTION = 15.0

# Must match TELEMETRY_UDP_PORT in GPSReader.py
TELEMETRY_UDP_PORT = 5761

# Must match COMMAND_HOST / COMMAND_UDP_PORT in GPSReader.py
COMMAND_HOST = "127.0.0.1"
COMMAND_PORT = 5760

# If no IMU packet arrives within this window, declare telemetry stale
# and command servos to neutral.
UDP_TIMEOUT_S = 0.5

# GPSReader.py logs gyro values in deg/s. Use --gyro-units rad/s if your bridge
# is sending raw Adafruit gyro radians/second instead.
GYRO_INPUT_UNITS = "rad/s"

SERVO_NEUTRAL_COMMAND = 0.0
CONTROL_LOOP_DELAY_S = 0.02
STALE_TELEMETRY_TIMEOUT_S = 0.5


def now():
    return datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds")


def init_db(path):
    conn = sqlite3.connect(path)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS roll_control (
            id                  INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp           TEXT    NOT NULL,
            state               TEXT    NOT NULL,
            raw_roll_rate       REAL    NOT NULL,
            filtered_roll_rate  REAL    NOT NULL,
            rotation_x_deg      REAL    NOT NULL,
            rotation_y_deg      REAL    NOT NULL,
            fin_command         REAL    NOT NULL
        )
    """)
    conn.commit()
    return conn

# Scalar Kalman-filter tuning for roll rate in (deg/s)^2. Increase
# GYRO_PROCESS_VARIANCE for faster response; increase GYRO_MEASUREMENT_VARIANCE
# for stronger noise rejection.
GYRO_PROCESS_VARIANCE = 0.1
GYRO_MEASUREMENT_VARIANCE = 4.0

# -----------------------------
# Data model
# -----------------------------


@dataclass
class TelemetrySnapshot:
    timestamp: str | None = None
    imu_timestamp: str | None = None
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


class KalmanFilter1D:
    """Scalar Kalman filter for a signal modeled as locally constant."""

    def __init__(self, process_variance, measurement_variance):
        if process_variance < 0:
            raise ValueError("process_variance must be non-negative")
        if measurement_variance <= 0:
            raise ValueError("measurement_variance must be positive")

        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = None
        self.estimate_variance = None

    def update(self, measurement):
        if self.estimate is None:
            self.estimate = measurement
            self.estimate_variance = self.measurement_variance
            return self.estimate

        predicted_variance = self.estimate_variance + self.process_variance
        kalman_gain = predicted_variance / (
            predicted_variance + self.measurement_variance
        )
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.estimate_variance = (1.0 - kalman_gain) * predicted_variance
        return self.estimate


class GPSReaderTelemetry:
    """Reads the newest telemetry rows written by GPSReader.py."""

    def __init__(self, port=TELEMETRY_UDP_PORT, gyro_units=GYRO_INPUT_UNITS):
        self.gyro_units = gyro_units
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))

    def receive(self, timeout=UDP_TIMEOUT_S):
        """Return a TelemetrySnapshot from the next IMU packet, or None on timeout."""
        self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(256)
        except socket.timeout:
            return None

        line = data.decode("ascii", errors="replace").strip()
        if not line.startswith("$IMU,"):
            return None

        parts = line.split(",")
        if len(parts) != 7:
            return None

        try:
            return TelemetrySnapshot(
                accel_x=float(parts[1]),
                accel_y=float(parts[2]),
                accel_z=float(parts[3]),
                gyro_x=normalize_gyro(float(parts[4]), self.gyro_units),
                gyro_y=normalize_gyro(float(parts[5]), self.gyro_units),
                gyro_z=normalize_gyro(float(parts[6]), self.gyro_units),
            )
        except (ValueError, IndexError):
            return None

    def close(self):
        self.sock.close()


# -----------------------------
# Helper functions
# -----------------------------


def select_roll_rate(snapshot, axis):
    rate = {"x": snapshot.gyro_x, "y": snapshot.gyro_y, "z": snapshot.gyro_z}.get(axis)
    return rate if rate is not None else 0.0


def normalize_gyro(value, units):
    if units == "rad/s":
        return math.degrees(value)
    return value


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def estimate_rotation_x_y(snapshot):
    """
    Estimate tilt from accelerometer data.
    Only valid for slow / ground movement — not for powered flight.
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
    roll_rate_error = TARGET_ROLL_RATE - roll_rate
    dominant_tilt = rotation_y_deg
    command = (ROLL_RATE_GAIN * roll_rate_error) - (TILT_GAIN * dominant_tilt)
    return clamp(command, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)


def send_command_to_esp32(esp32, fin_command):
    message = f"ROLL,{fin_command:.2f}\n"
    esp32.sendto(message.encode("utf-8"), (COMMAND_HOST, COMMAND_PORT))


def open_command_link(enabled):
    if not enabled:
        return None
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Aggressive ground-test roll controller. "
            "Bypasses altitude safety logic — do not use as flight controller."
        )
    )
    parser.add_argument(
        "--no-commands",
        action="store_true",
        help="Log only — do not send ROLL commands or move servos.",
    )
    parser.add_argument(
        "--gyro-units",
        choices=("deg/s", "rad/s"),
        default=GYRO_INPUT_UNITS,
        help="Gyro units GPSReader.py is storing.",
    )
    parser.add_argument(
        "--roll-axis",
        choices=("x", "y", "z"),
        default="z",
        help="Gyro axis to treat as rocket roll rate.",
    )
    return parser.parse_args()


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    telemetry = GPSReaderTelemetry(gyro_units=args.gyro_units)
    esp32 = open_command_link(not args.no_commands)
    roll_rate_filter = KalmanFilter1D(
        process_variance=GYRO_PROCESS_VARIANCE,
        measurement_variance=GYRO_MEASUREMENT_VARIANCE,
    )

    os.makedirs(LOGS_DIR, exist_ok=True)
    conn = init_db(DB_PATH)

    print("Ground roll-control test is running.")
    print("This bypasses altitude safety logic and uses aggressive gains.")
    print(f"Command link: {'disabled' if not esp32 else f'{COMMAND_HOST}:{COMMAND_PORT}'}")
    print(f"Log:          {DB_PATH}")
    print("Press Ctrl+C to stop and send neutral.")

    last_fresh_read = time.monotonic()
    filtered_roll_rate = 0.0
    raw_roll_rate = 0.0
    rotation_x_deg = 0.0
    rotation_y_deg = 0.0

    try:
        while True:
            snapshot = telemetry.receive()

            if snapshot is not None:
                last_fresh_read = time.monotonic()
                raw_roll_rate = select_roll_rate(snapshot, args.roll_axis)
                filtered_roll_rate = roll_rate_filter.update(raw_roll_rate)
                rotation_x_deg, rotation_y_deg = estimate_rotation_x_y(snapshot)

            telemetry_is_fresh = (
                time.monotonic() - last_fresh_read
            ) <= STALE_TELEMETRY_TIMEOUT_S

            state = "GROUND_TEST_ACTIVE" if telemetry_is_fresh else "STALE_TELEMETRY"
            fin_command = (
                calculate_ground_test_command(
                    filtered_roll_rate,
                    rotation_x_deg,
                    rotation_y_deg,
                )
                if telemetry_is_fresh
                else SERVO_NEUTRAL_COMMAND
            )

            if esp32:
                send_command_to_esp32(esp32, fin_command)

            print(
                f"{state} roll_rate={filtered_roll_rate:7.2f} deg/s "
                f"(raw={raw_roll_rate:7.2f}) "
                f"rot_x={rotation_x_deg:7.2f} rot_y={rotation_y_deg:7.2f} "
                f"cmd={fin_command:6.2f}",
                end="\r",
                flush=True,
            )

            conn.execute(
                "INSERT INTO roll_control "
                "(timestamp, state, raw_roll_rate, filtered_roll_rate, "
                " rotation_x_deg, rotation_y_deg, fin_command) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (now(), state, raw_roll_rate, filtered_roll_rate,
                 rotation_x_deg, rotation_y_deg, fin_command),
            )
            conn.commit()

            time.sleep(CONTROL_LOOP_DELAY_S)

    except KeyboardInterrupt:
        if esp32:
            send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
        print("\nStopped. Neutral command sent.")
        print(f"Session saved to: {DB_PATH}")
    finally:
        conn.close()
        telemetry.close()
        if esp32:
            esp32.close()


if __name__ == "__main__":
    main()
