import argparse
import copy
import math
import socket
import threading
import time
from dataclasses import dataclass

# -----------------------------
# Configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0       # deg/s
KP = 0.0025                  # nominal roll-rate gain
MAX_FIN_DEFLECTION = 7.5     # degrees
CANARD_ACCELERATION_COEFFICIENT = 0.02430  # deg/s^2 per deg of fin deflection at 1 m/s^2
MIN_VERTICAL_VELOCITY_M_S = 0.1

# Must match TELEMETRY_UDP_PORT in GPSReader.py
TELEMETRY_UDP_PORT = 5761

# Must match COMMAND_HOST / COMMAND_UDP_PORT in GPSReader.py
COMMAND_HOST = "127.0.0.1"
COMMAND_PORT = 5760

# The ESP32 bridge (Adafruit MPU6050) outputs gyro in radians/second.
GYRO_INPUT_UNITS = "rad/s"

# Scalar Kalman-filter tuning in (deg/s)^2. Increase process variance for
# faster response, or measurement variance for stronger noise rejection.
GYRO_PROCESS_VARIANCE = 0.1
GYRO_MEASUREMENT_VARIANCE = 4.0

# Velocity-fusion tuning in SI units. These values are deliberately grouped so
# sensor trust and gating can be adjusted without changing estimator code.
VELOCITY_PROCESS_VARIANCE = 0.5       # (m/s)^2 added per second
GPS_VELOCITY_VARIANCE = 1.5           # GPS velocity measurement variance
ACCEL_VELOCITY_VARIANCE = 4.0         # integrated acceleration drifts fastest
ALTIMETER_VELOCITY_VARIANCE = 2.5     # differentiated altitude is fairly noisy
VELOCITY_GATE_SIGMA = 4.0             # reject measurements far from the estimate
MAX_ABS_VELOCITY_M_S = 500.0
MAX_ABS_ACCELERATION_M_S2 = 80.0
STANDARD_GRAVITY_M_S2 = 9.80665

# -----------------------------
# Safety parameters
# -----------------------------

MIN_CONTROL_ALTITUDE_M = 0.0
# Kill control once the nose is more than this far from vertical. acos(az/|a|)
# yields a true 0–180° tilt, so a nose-down attitude (>90°) is caught instead of
# aliasing back toward 0° the way per-axis atan2 tilt does.
MAX_TILT_FROM_VERTICAL_DEG = 90.0
# Inhibit canard control once clearly descending. A small threshold keeps control
# from chattering as vertical velocity crosses zero at apogee.
DESCENT_INHIBIT_M_S    = 2.0   # descent rate (negative velocity) that halts control
STALE_TIMEOUT_S        = 0.5   # seconds without IMU data → stale

SERVO_NEUTRAL_COMMAND = 0.0

# -----------------------------
# Data model
# -----------------------------


@dataclass
class TelemetrySnapshot:
    imu_sequence: int = 0
    gps_sequence: int = 0
    altimeter_sequence: int = 0
    altitude_m: float | None = None
    accel_x:    float | None = None
    accel_y:    float | None = None
    accel_z:    float | None = None
    gyro_x:     float | None = None
    gyro_y:     float | None = None
    gyro_z:     float | None = None
    gps_velocity_m_s: float | None = None
    accelerometer_velocity_m_s: float | None = None
    altimeter_velocity_m_s: float | None = None
    fused_velocity_m_s: float | None = None
    vertical_velocity_m_s: float | None = None


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


class VelocityFusionEstimator:
    """One-state Kalman estimator whose state is velocity in metres/second.

    GPS, integrated accelerometer, and differentiated altimeter readings are
    optional measurements of that state. Invalid or gated readings are ignored;
    when none are usable, ``estimate`` retains the last valid fused velocity.
    """

    def __init__(self):
        self.estimate = None
        self.estimate_variance = None

    def predict(self, dt):
        if self.estimate is not None and is_valid_number(dt) and dt > 0.0:
            self.estimate_variance += VELOCITY_PROCESS_VARIANCE * dt

    def update(self, measurement, measurement_variance):
        if (
            not is_valid_number(measurement)
            or abs(measurement) > MAX_ABS_VELOCITY_M_S
        ):
            return self.estimate

        if self.estimate is None:
            self.estimate = measurement
            self.estimate_variance = measurement_variance
            return self.estimate

        innovation = measurement - self.estimate
        innovation_variance = self.estimate_variance + measurement_variance
        if abs(innovation) > VELOCITY_GATE_SIGMA * math.sqrt(innovation_variance):
            return self.estimate

        kalman_gain = self.estimate_variance / innovation_variance
        self.estimate += kalman_gain * innovation
        self.estimate_variance *= 1.0 - kalman_gain
        return self.estimate


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
        self._last_altitude_time = None
        self._last_imu_velocity_time = None

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
                            now = time.monotonic()
                            acceleration = self._snapshot.accel_z - STANDARD_GRAVITY_M_S2
                            if abs(acceleration) <= MAX_ABS_ACCELERATION_M_S2:
                                if self._last_imu_velocity_time is not None:
                                    dt = now - self._last_imu_velocity_time
                                    if 0.0 < dt <= STALE_TIMEOUT_S:
                                        previous = self._snapshot.accelerometer_velocity_m_s
                                        if previous is None:
                                            previous = self._snapshot.fused_velocity_m_s or 0.0
                                        self._snapshot.accelerometer_velocity_m_s = (
                                            previous + acceleration * dt
                                        )
                                self._last_imu_velocity_time = now
                            self._snapshot.imu_sequence += 1
                            self._last_imu_time = now
                    except (ValueError, IndexError):
                        pass

            elif line.startswith("$ALT,"):
                parts = line.split(",")
                if len(parts) == 2:
                    try:
                        with self._lock:
                            alt = float(parts[1])
                            now = time.monotonic()
                            if (
                                self._snapshot.altitude_m is not None
                                and self._last_altitude_time is not None
                            ):
                                dt = now - self._last_altitude_time
                                if dt > 1e-3:
                                    self._snapshot.vertical_velocity_m_s = (
                                        alt - self._snapshot.altitude_m
                                    ) / dt
                                    self._snapshot.altimeter_velocity_m_s = (
                                        self._snapshot.vertical_velocity_m_s
                                    )
                            self._snapshot.altitude_m = alt
                            self._snapshot.altimeter_sequence += 1
                            self._last_altitude_time = now
                    except (ValueError, IndexError):
                        pass

            # Optional packet supplied by a GPS producer: $GPS,<velocity_m_s>.
            elif line.startswith("$GPS,"):
                parts = line.split(",")
                if len(parts) == 2:
                    try:
                        velocity = float(parts[1])
                        if is_valid_number(velocity):
                            with self._lock:
                                self._snapshot.gps_velocity_m_s = velocity
                                self._snapshot.gps_sequence += 1
                    except (ValueError, IndexError):
                        pass

    def latest(self):
        with self._lock:
            return copy.copy(self._snapshot)

    def set_fused_velocity(self, velocity_m_s):
        """Publish the estimator output for other control calculations."""
        with self._lock:
            self._snapshot.fused_velocity_m_s = velocity_m_s

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


def is_valid_number(value):
    """Return True for finite numeric sensor values (not None, NaN, or inf)."""
    return isinstance(value, (int, float)) and math.isfinite(value)


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def tilt_from_vertical_deg(snapshot):
    """Angle between the roll axis (nose) and world-up, 0–180°.

    Uses acos(az/|a|) rather than per-axis atan2 tilt: the latter buries az in a
    positive square root and folds at ±90°, so a nose-down attitude aliases back
    toward 0° and slips past the kill limit. This form reads 180° when the nose
    points straight down. Relies on the same az≈+g nose-up convention the
    velocity integrator assumes.
    """
    if None in (snapshot.accel_x, snapshot.accel_y, snapshot.accel_z):
        return 0.0

    ax = snapshot.accel_x
    ay = snapshot.accel_y
    az = snapshot.accel_z

    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-6:
        return 0.0

    return math.degrees(math.acos(clamp(az / norm, -1.0, 1.0)))


def control_is_allowed(altitude_m, tilt_deg, vertical_velocity_m_s):
    effective_alt = altitude_m if altitude_m is not None else 0.0
    descending = (
        vertical_velocity_m_s is not None
        and vertical_velocity_m_s < -DESCENT_INHIBIT_M_S
    )
    return (
        effective_alt >= MIN_CONTROL_ALTITUDE_M
        and tilt_deg < MAX_TILT_FROM_VERTICAL_DEG
        and not descending
    )


def calculate_gain_from_vertical_velocity(vertical_velocity_m_s):
    if vertical_velocity_m_s is None or abs(vertical_velocity_m_s) < MIN_VERTICAL_VELOCITY_M_S:
        return KP
    return KP / (CANARD_ACCELERATION_COEFFICIENT * vertical_velocity_m_s**2)


def calculate_fin_command(roll_rate, vertical_velocity_m_s):
    error = TARGET_ROLL_RATE - roll_rate
    gain = calculate_gain_from_vertical_velocity(vertical_velocity_m_s)
    return clamp(gain * error, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)


def send_command_to_esp32(esp32, fin_command):
    """Send one signed deflection applied to both mirrored roll canards."""
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
        help="Receive telemetry without sending ROLL commands or moving servos.",
    )
    return parser.parse_args()


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    telemetry = LiveTelemetry()
    esp32 = open_command_link(not args.no_commands)
    roll_rate_filter = KalmanFilter1D(
        process_variance=GYRO_PROCESS_VARIANCE,
        measurement_variance=GYRO_MEASUREMENT_VARIANCE
    )
    last_imu_sequence = 0
    last_gps_sequence = 0
    last_altimeter_sequence = 0
    filtered_roll_rate = 0.0
    velocity_filter = VelocityFusionEstimator()
    last_velocity_update_time = time.monotonic()

    try:
        while True:
            snapshot = telemetry.latest()

            now = time.monotonic()
            velocity_filter.predict(now - last_velocity_update_time)
            last_velocity_update_time = now

            # Only consume each source when it has a new sample. GPS and
            # altimeter updates are applied before the noisier integrated IMU.
            if snapshot.gps_sequence != last_gps_sequence:
                last_gps_sequence = snapshot.gps_sequence
                velocity_filter.update(
                    snapshot.gps_velocity_m_s, GPS_VELOCITY_VARIANCE
                )
            if snapshot.altimeter_sequence != last_altimeter_sequence:
                last_altimeter_sequence = snapshot.altimeter_sequence
                velocity_filter.update(
                    snapshot.altimeter_velocity_m_s,
                    ALTIMETER_VELOCITY_VARIANCE,
                )
            if snapshot.imu_sequence != last_imu_sequence:
                velocity_filter.update(
                    snapshot.accelerometer_velocity_m_s,
                    ACCEL_VELOCITY_VARIANCE,
                )

            # None means no source has ever produced a valid value. Thereafter,
            # the estimator retains its last estimate through sensor dropouts.
            fused_velocity_m_s = velocity_filter.estimate
            telemetry.set_fused_velocity(fused_velocity_m_s)

            tilt_deg = tilt_from_vertical_deg(snapshot)

            # The current ESP32 bridge wiring assumes gyro_z is roll rate.
            raw_roll_rate = snapshot.gyro_z or 0.0
            if snapshot.imu_sequence != last_imu_sequence:
                last_imu_sequence = snapshot.imu_sequence
                filtered_roll_rate = roll_rate_filter.update(raw_roll_rate)

            allowed = control_is_allowed(
                snapshot.altitude_m,
                tilt_deg,
                fused_velocity_m_s,
            )

            state = "CONTROL_ACTIVE" if allowed and telemetry.is_fresh() else "IDLE"
            fin_command = (
                calculate_fin_command(
                    filtered_roll_rate,
                    fused_velocity_m_s,
                )
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
