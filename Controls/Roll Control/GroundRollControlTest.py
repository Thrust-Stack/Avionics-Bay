import argparse
import csv
import math
import socket
import time
from dataclasses import dataclass

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

# -----------------------------
# Data model
# -----------------------------


@dataclass
class TelemetrySnapshot:
    accel_x: float | None = None
    accel_y: float | None = None
    accel_z: float | None = None
    gyro_x:  float | None = None
    gyro_y:  float | None = None
    gyro_z:  float | None = None


# -----------------------------
# Live telemetry receiver
# -----------------------------


class LiveTelemetry:
    """
    Receives $IMU packets broadcast by GPSReader.py over UDP.
    Each call to receive() blocks until a packet arrives or the timeout expires.
    The control loop is naturally paced by the 20 Hz IMU stream — no sleep needed.
    """

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


def write_log_header(logger):
    logger.writerow([
        "time",
        "state",
        "rotation_x_deg",
        "rotation_y_deg",
        "gyro_x_deg_s",
        "gyro_y_deg_s",
        "gyro_z_deg_s",
        "roll_rate_deg_s",
        "fin_command",
    ])


def select_roll_rate(snapshot, axis):
    if axis == "x":
        return snapshot.gyro_x or 0.0
    if axis == "y":
        return snapshot.gyro_y or 0.0
    return snapshot.gyro_z or 0.0


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
        "--log",
        default="ground_roll_test_log.csv",
        help="CSV path for ground-test log output.",
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
    telemetry = LiveTelemetry(gyro_units=args.gyro_units)
    esp32 = open_command_link(not args.no_commands)

    print("Ground roll-control test running.")
    print("Requires GPSReader.py to be running first.")
    print(f"Telemetry : UDP localhost:{TELEMETRY_UDP_PORT}")
    print(f"Commands  : {'UDP -> ' + COMMAND_HOST + ':' + str(COMMAND_PORT) if esp32 else 'disabled (--no-commands)'}")
    print("Press Ctrl+C to stop.\n")

    # Zero the canards before the control loop starts.
    # Re-send every second in case the ESP32 was still booting when we first connected.
    if esp32:
        send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
        print("Canards zeroed (neutral). Starting control loop in 5 seconds...")
        for i in range(5, 0, -1):
            send_command_to_esp32(esp32, SERVO_NEUTRAL_COMMAND)
            print(f"  {i}...", flush=True)
            time.sleep(1.0)
        print("GO\n")

    with open(args.log, "w", newline="") as log_file:
        logger = csv.writer(log_file)
        write_log_header(logger)

        try:
            while True:
                # Blocks here until an IMU packet arrives (or UDP_TIMEOUT_S elapses).
                # The 20 Hz IMU stream is the natural pacing — no sleep() needed.
                snapshot = telemetry.receive()

                if snapshot is None:
                    state = "STALE_TELEMETRY"
                    fin_command = SERVO_NEUTRAL_COMMAND
                    rotation_x_deg = rotation_y_deg = 0.0
                    gyro_x = gyro_y = gyro_z = roll_rate = 0.0
                else:
                    state = "ACTIVE"
                    rotation_x_deg, rotation_y_deg = estimate_rotation_x_y(snapshot)
                    gyro_x    = snapshot.gyro_x or 0.0
                    gyro_y    = snapshot.gyro_y or 0.0
                    gyro_z    = snapshot.gyro_z or 0.0
                    roll_rate = select_roll_rate(snapshot, args.roll_axis)
                    fin_command = calculate_ground_test_command(
                        roll_rate, rotation_x_deg, rotation_y_deg
                    )

                if esp32:
                    send_command_to_esp32(esp32, fin_command)

                logger.writerow([
                    time.time(),
                    state,
                    rotation_x_deg,
                    rotation_y_deg,
                    gyro_x,
                    gyro_y,
                    gyro_z,
                    roll_rate,
                    fin_command,
                ])
                log_file.flush()

                print(
                    f"{state:<16}  roll={roll_rate:7.2f} deg/s  "
                    f"rot_x={rotation_x_deg:7.2f}  rot_y={rotation_y_deg:7.2f}  "
                    f"cmd={fin_command:6.2f}",
                    end="\r",
                    flush=True,
                )

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
