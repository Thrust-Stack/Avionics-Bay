import argparse
import copy
import datetime
import glob
import math
import os
import socket
import sqlite3
import threading
import time
from dataclasses import dataclass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR   = os.path.join(SCRIPT_DIR, "logs")
DB_PATH    = os.path.join(LOGS_DIR, "BRCRollData.db")

# -----------------------------
# Configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0       # deg/s
KP = 0.0025                  # proportional gain
MAX_FIN_DEFLECTION = 7.5     # degrees

# Must match TELEMETRY_UDP_PORT in GPSReader.py
TELEMETRY_UDP_PORT = 5761

# Must match COMMAND_HOST / COMMAND_UDP_PORT in GPSReader.py
COMMAND_HOST = "127.0.0.1"
COMMAND_PORT = 5760

# The ESP32 bridge (Adafruit MPU6050) outputs gyro in radians/second.
GYRO_INPUT_UNITS = "rad/s"

# -----------------------------
# Safety parameters
# -----------------------------

MIN_CONTROL_ALTITUDE_M = 0.0
MAX_X_ROTATION_DEG     = 90.0
MAX_Y_ROTATION_DEG     = 90.0
STALE_TIMEOUT_S        = 0.5   # seconds without IMU data → stale

SERVO_NEUTRAL_COMMAND = 0.0


def now():
    return datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds")


def init_db(path):
    conn = sqlite3.connect(path)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS roll_control (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp        TEXT    NOT NULL,
            state            TEXT    NOT NULL,
            altitude_m       REAL,
            roll_rate        REAL    NOT NULL,
            rotation_x_deg   REAL    NOT NULL,
            rotation_y_deg   REAL    NOT NULL,
            control_allowed  INTEGER NOT NULL,
            fin_command      REAL    NOT NULL
        )
    """)
    conn.commit()
    return conn


# -----------------------------
# Data model
# -----------------------------


@dataclass
class TelemetrySnapshot:
    altitude_m: float | None = None
    accel_x:    float | None = None
    accel_y:    float | None = None
    accel_z:    float | None = None
    gyro_x:     float | None = None
    gyro_y:     float | None = None
    gyro_z:     float | None = None


# -----------------------------
# Live telemetry receiver
# -----------------------------


class LiveTelemetry:
    """
    Background thread receives $IMU and $ALT packets broadcast by GPSReader.py.
    Main loop calls latest() for the freshest combined snapshot and is_fresh()
    to detect if the data has gone stale.
    """

    def __init__(self, port=TELEMETRY_UDP_PORT, gyro_units=GYRO_INPUT_UNITS):
        self.gyro_units = gyro_units
        self._snapshot = TelemetrySnapshot()
        self._lock = threading.Lock()
        self._last_imu_time = 0.0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", port))
        self._sock.settimeout(0.1)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(256)
            except socket.timeout:
                continue

            line = data.decode("ascii", errors="replace").strip()

            if line.startswith("$IMU,"):
                parts = line.split(",")
                if len(parts) == 7:
                    try:
                        with self._lock:
                            self._snapshot.accel_x = float(parts[1])
                            self._snapshot.accel_y = float(parts[2])
                            self._snapshot.accel_z = float(parts[3])
                            self._snapshot.gyro_x = normalize_gyro(float(parts[4]), self.gyro_units)
                            self._snapshot.gyro_y = normalize_gyro(float(parts[5]), self.gyro_units)
                            self._snapshot.gyro_z = normalize_gyro(float(parts[6]), self.gyro_units)
                            self._last_imu_time = time.monotonic()
                    except (ValueError, IndexError):
                        pass

            elif line.startswith("$ALT,"):
                parts = line.split(",")
                if len(parts) == 2:
                    try:
                        with self._lock:
                            self._snapshot.altitude_m = float(parts[1])
                    except (ValueError, IndexError):
                        pass

    def latest(self):
        with self._lock:
            return copy.copy(self._snapshot)

    def is_fresh(self):
        return (time.monotonic() - self._last_imu_time) < STALE_TIMEOUT_S

    def close(self):
        self._stop.set()
        self._sock.close()


# -----------------------------
# Helper functions
# -----------------------------


def normalize_gyro(value, units):
    if units == "rad/s":
        return math.degrees(value)
    return value


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def estimate_rotation_x_y(snapshot):
    if None in (snapshot.accel_x, snapshot.accel_y, snapshot.accel_z):
        return 0.0, 0.0

    ax = snapshot.accel_x
    ay = snapshot.accel_y
    az = snapshot.accel_z

    rotation_x_deg = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
    rotation_y_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    return rotation_x_deg, rotation_y_deg


def control_is_allowed(altitude_m, rotation_x_deg, rotation_y_deg):
    effective_alt = altitude_m if altitude_m is not None else 0.0
    return (
        effective_alt >= MIN_CONTROL_ALTITUDE_M
        and abs(rotation_x_deg) < MAX_X_ROTATION_DEG
        and abs(rotation_y_deg) < MAX_Y_ROTATION_DEG
    )


def calculate_fin_command(roll_rate):
    error = TARGET_ROLL_RATE - roll_rate
    return clamp(KP * error, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)


def send_command_to_esp32(esp32, fin_command):
    message = f"ROLL,{fin_command:.2f}\n"
    esp32.sendto(message.encode("utf-8"), (COMMAND_HOST, COMMAND_PORT))


def open_command_link(enabled):
    if not enabled:
        return None
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Roll controller — requires GPSReader.py to be running first."
    )
    parser.add_argument(
        "--no-commands",
        action="store_true",
        help="Log only — do not send ROLL commands or move servos.",
    )
    return parser.parse_args()


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    telemetry = LiveTelemetry()
    esp32 = open_command_link(not args.no_commands)

    os.makedirs(LOGS_DIR, exist_ok=True)
    conn = init_db(DB_PATH)

    print("Roll controller running — requires GPSReader.py to be running first.")
    print(f"Command link: {'disabled' if not esp32 else f'{COMMAND_HOST}:{COMMAND_PORT}'}")
    print(f"Log:          {DB_PATH}")
    print("Press Ctrl+C to stop and send neutral.")

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

            fresh = "LIVE" if telemetry.is_fresh() else "STALE"
            print(
                f"{state} [{fresh}] "
                f"roll_rate={roll_rate:7.2f} deg/s  "
                f"rot_x={rotation_x_deg:7.2f}  rot_y={rotation_y_deg:7.2f}  "
                f"alt={snapshot.altitude_m if snapshot.altitude_m is not None else '---':>6}  "
                f"cmd={fin_command:6.2f}",
                end="\r",
                flush=True,
            )

            conn.execute(
                "INSERT INTO roll_control "
                "(timestamp, state, altitude_m, roll_rate, "
                " rotation_x_deg, rotation_y_deg, control_allowed, fin_command) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (now(), state, snapshot.altitude_m, roll_rate,
                 rotation_x_deg, rotation_y_deg, int(allowed), fin_command),
            )
            conn.commit()

            time.sleep(0.01)  # 100 Hz loop

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
