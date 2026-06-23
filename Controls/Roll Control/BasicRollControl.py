import argparse
import copy
import csv
import math
import socket
import threading
import time
from dataclasses import dataclass

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


def write_log_header(logger):
    logger.writerow([
        "time",
        "state",
        "altitude_m",
        "rotation_x_deg",
        "rotation_y_deg",
        "gyro_x_deg_s",
        "gyro_y_deg_s",
        "gyro_z_deg_s",
        "roll_rate_deg_s",
        "control_allowed",
        "fin_command",
    ])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Roll controller — requires GPSReader.py to be running first."
    )
    parser.add_argument(
        "--no-commands",
        action="store_true",
        help="Log only — do not send ROLL commands or move servos.",
    )
    parser.add_argument(
        "--log",
        default="flight_log.csv",
        help="CSV path for roll-control log output.",
    )
    parser.add_argument(
        "--gyro-units",
        choices=("deg/s", "rad/s"),
        default=GYRO_INPUT_UNITS,
        help="Gyro units the ESP32 bridge outputs.",
    )
    return parser.parse_args()


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    telemetry = LiveTelemetry(gyro_units=args.gyro_units)
    esp32 = open_command_link(not args.no_commands)

    print("Roll controller running.")
    print("Requires GPSReader.py to be running first.")
    print(f"Telemetry : UDP localhost:{TELEMETRY_UDP_PORT}")
    print(f"Commands  : {'UDP -> ' + COMMAND_HOST + ':' + str(COMMAND_PORT) if esp32 else 'disabled (--no-commands)'}")
    print("Press Ctrl+C to stop.\n")

    # Zero the canards before the control loop starts
    if esp32:
        send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
        print("Canards zeroed (neutral). Starting control loop in 5 seconds...")
        for i in range(5, 0, -1):
            print(f"  {i}...", flush=True)
            time.sleep(1.0)
        print("GO\n")

    with open(args.log, "w", newline="") as log_file:
        logger = csv.writer(log_file)
        write_log_header(logger)

        try:
            while True:
                now = time.time()
                snapshot = telemetry.latest()

                if not telemetry.is_fresh():
                    state = "STALE_TELEMETRY"
                    fin_command = SERVO_NEUTRAL_COMMAND
                    rotation_x_deg = rotation_y_deg = 0.0
                    gyro_x = gyro_y = gyro_z = roll_rate = 0.0
                    allowed = False
                else:
                    rotation_x_deg, rotation_y_deg = estimate_rotation_x_y(snapshot)
                    gyro_x    = snapshot.gyro_x or 0.0
                    gyro_y    = snapshot.gyro_y or 0.0
                    gyro_z    = snapshot.gyro_z or 0.0
                    roll_rate = gyro_z

                    allowed = control_is_allowed(
                        snapshot.altitude_m, rotation_x_deg, rotation_y_deg
                    )
                    state = "CONTROL_ACTIVE" if allowed else "IDLE"
                    fin_command = (
                        calculate_fin_command(roll_rate)
                        if allowed
                        else SERVO_NEUTRAL_COMMAND
                    )

                if esp32:
                    send_command_to_esp32(esp32, fin_command)

                logger.writerow([
                    now,
                    state,
                    snapshot.altitude_m,
                    rotation_x_deg,
                    rotation_y_deg,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    roll_rate,
                    allowed,
                    fin_command,
                ])
                log_file.flush()

                print(
                    f"{state:<16}  alt={str(snapshot.altitude_m or '---'):>7}m  "
                    f"roll={roll_rate:7.2f} deg/s  cmd={fin_command:6.2f}",
                    end="\r",
                    flush=True,
                )

                time.sleep(0.01)  # 100 Hz loop

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
