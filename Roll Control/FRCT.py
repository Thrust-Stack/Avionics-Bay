"""
FRCT.py — Flight Roll-Control + Roll-Test controller.

Requires GPSReader.py to be running first (it broadcasts $IMU / $ALT over UDP
and relays ROLL commands to the ESP32).

Flight sequence
---------------
  1. PRE_TEST   : roll-RATE damping (drive gyro_z -> 0), same proven law as
                  BRC.py. Meanwhile count consecutive $ALT samples above 200 m.
  2. TRIGGER    : after N consecutive $ALT samples above 200 m, LATCH (one-time
                  per flight) and start the roll test. The gyro-integrated roll
                  angle is zeroed at this instant, so integration drift only
                  accumulates over the ~1.2 s test, not the whole flight.
  3. ROLL_TEST  : 0 deg -> +90 deg (right) at 180 deg/s, hold 0.2 s, +90 -> 0.
                  Phases advance when the measured angle is within +/-2 deg of
                  the setpoint. Tracked with a PD law on a ramped setpoint.
  4. POST_TEST  : resume roll-RATE damping for the rest of the flight.

Fail-safe
---------
  * Stale telemetry during PRE_TEST / POST_TEST -> send neutral, hold state,
    resume automatically when telemetry recovers (rate damping is safe to pause).
  * Stale telemetry DURING the roll test, or the test exceeding its time budget
    -> latch FAILSAFE_LOCK: send neutral (canards centered / "zero") for the
    rest of the flight. An open-loop angle maneuver is never continued on bad
    data.

IMPORTANT — this is flight hardware:
  The roll-angle PD gains (KP_ANGLE / KD_ANGLE) depend on canard effectiveness
  at flight airspeed and CANNOT be known a priori. The values below are only a
  starting point. They MUST be validated in simulation / on the bench before
  flight. BRC.py is intentionally left untouched as the proven fallback.
"""

import argparse
import copy
import datetime
import math
import os
import socket
import sqlite3
import threading
import time
from dataclasses import dataclass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR   = os.path.join(SCRIPT_DIR, "logs")
DB_PATH    = os.path.join(LOGS_DIR, "FRCTRollData.db")

# -----------------------------
# Rate-damping configuration (PRE_TEST / POST_TEST)
# -----------------------------

TARGET_ROLL_RATE = 0.0       # deg/s — hold zero spin
KP_RATE = 0.0025             # proportional gain on roll rate (matches BRC.py)
MAX_FIN_DEFLECTION = 7.5     # degrees — hard clamp on every command

# -----------------------------
# Roll-test configuration
# -----------------------------

ROLL_TEST_ALTITUDE_M      = 200.0   # trigger altitude (AGL, from $ALT / BMP585)
ROLL_TEST_ARM_SAMPLES     = 4       # consecutive $ALT samples above trigger to arm
ROLL_TEST_TARGET_DEG      = 90.0    # rotate to +90 deg (right)
ROLL_TEST_SLEW_RATE_DEG_S = 180.0   # setpoint ramp rate -> ~0.5 s per 90 deg
ROLL_TEST_HOLD_S          = 0.2     # dwell at +90 deg
ROLL_TEST_ANGLE_TOL_DEG   = 2.0     # +/- tolerance to consider a phase reached
ROLL_TEST_TIMEOUT_S       = 2.5     # hard budget; exceeded -> FAILSAFE_LOCK

# Roll-ANGLE PD gains — TUNABLE, MUST be validated in sim/bench before flight.
# Command [deg] = KP_ANGLE*(angle_err) + KD_ANGLE*(rate_err).
# The KD*(setpoint_rate) term acts as feed-forward: during the 180 deg/s ramp it
# commands ~KD_ANGLE*180 deg of steady deflection to establish the roll rate.
KP_ANGLE = 0.12
KD_ANGLE = 0.02

# -----------------------------
# Telemetry / links (must match GPSReader.py)
# -----------------------------

TELEMETRY_UDP_PORT = 5761
COMMAND_HOST = "127.0.0.1"
COMMAND_PORT = 5760

# The ESP32 bridge (Adafruit MPU6050) outputs gyro in radians/second.
GYRO_INPUT_UNITS = "rad/s"

STALE_TIMEOUT_S = 0.5   # seconds without IMU data -> stale
SERVO_NEUTRAL_COMMAND = 0.0

# Integration guard: ignore absurd loop dt (startup hiccups, scheduler stalls)
MAX_INTEGRATION_DT_S = 0.1

LOOP_PERIOD_S = 0.01    # 100 Hz control loop

# Flight states
STATE_PRE_TEST      = "PRE_TEST"       # rate damping, waiting for trigger
STATE_ROLL_TEST     = "ROLL_TEST"      # executing the maneuver
STATE_POST_TEST     = "POST_TEST"      # rate damping, test complete
STATE_FAILSAFE_LOCK = "FAILSAFE_LOCK"  # terminal: canards neutral for rest of flight

# Roll-test sub-phases
PHASE_RAMP_UP   = "RAMP_UP"
PHASE_HOLD      = "HOLD"
PHASE_RAMP_DOWN = "RAMP_DOWN"


def now():
    return datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds")


def init_db(path):
    conn = sqlite3.connect(path)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS roll_control (
            id               INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp        TEXT    NOT NULL,
            state            TEXT    NOT NULL,
            phase            TEXT,
            fresh            INTEGER NOT NULL,
            altitude_m       REAL,
            roll_rate        REAL    NOT NULL,
            roll_angle_deg   REAL,
            angle_setpoint   REAL,
            fin_command      REAL    NOT NULL,
            canard1_deg      REAL    NOT NULL,
            canard2_deg      REAL    NOT NULL
        )
    """)
    conn.commit()
    return conn


# -----------------------------
# Data model
# -----------------------------


@dataclass
class TelemetrySnapshot:
    altitude_m:   float | None = None
    altitude_seq: int = 0        # increments on every $ALT update (for sample counting)
    accel_x:      float | None = None
    accel_y:      float | None = None
    accel_z:      float | None = None
    gyro_x:       float | None = None
    gyro_y:       float | None = None
    gyro_z:       float | None = None


# -----------------------------
# Live telemetry receiver
# -----------------------------


class LiveTelemetry:
    """Background thread receiving $IMU / $ALT packets broadcast by GPSReader.py."""

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
                        alt = float(parts[1])
                    except ValueError:
                        continue
                    with self._lock:
                        self._snapshot.altitude_m = alt
                        self._snapshot.altitude_seq += 1

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


def rate_damping_command(roll_rate):
    """PRE_TEST / POST_TEST: drive roll rate to zero (same law as BRC.py)."""
    error = TARGET_ROLL_RATE - roll_rate
    return clamp(KP_RATE * error, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)


def angle_tracking_command(angle_setpoint, setpoint_rate, roll_angle, roll_rate):
    """ROLL_TEST: PD tracking of a ramped roll-angle setpoint."""
    angle_err = angle_setpoint - roll_angle
    rate_err = setpoint_rate - roll_rate
    command = KP_ANGLE * angle_err + KD_ANGLE * rate_err
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
        description="Flight roll controller with a one-time 200 m roll test. "
                    "Requires GPSReader.py to be running first."
    )
    parser.add_argument(
        "--no-commands",
        action="store_true",
        help="Log only — do not send ROLL commands or move servos.",
    )
    return parser.parse_args()


# -----------------------------
# Roll-test state machine
# -----------------------------


class RollTest:
    """
    Runs the +90 deg / hold / return maneuver off a ramped setpoint.

    step() is called once per control loop with the freshest measured roll angle
    and rate. It returns (fin_command, phase, done, failed):
      done  -> maneuver completed cleanly, caller should resume rate damping.
      failed-> time budget exceeded, caller should latch FAILSAFE_LOCK.
    """

    def __init__(self, start_time):
        self.start_time = start_time
        self.phase = PHASE_RAMP_UP
        self.setpoint = 0.0
        self.setpoint_rate = 0.0
        self._hold_start = None

    def step(self, dt, roll_angle, roll_rate, current_time):
        if current_time - self.start_time > ROLL_TEST_TIMEOUT_S:
            return SERVO_NEUTRAL_COMMAND, self.phase, False, True

        if self.phase == PHASE_RAMP_UP:
            self.setpoint = min(ROLL_TEST_TARGET_DEG,
                                self.setpoint + ROLL_TEST_SLEW_RATE_DEG_S * dt)
            self.setpoint_rate = ROLL_TEST_SLEW_RATE_DEG_S
            if (self.setpoint >= ROLL_TEST_TARGET_DEG
                    and abs(roll_angle - ROLL_TEST_TARGET_DEG) <= ROLL_TEST_ANGLE_TOL_DEG):
                self.phase = PHASE_HOLD
                self.setpoint = ROLL_TEST_TARGET_DEG
                self.setpoint_rate = 0.0
                self._hold_start = current_time

        elif self.phase == PHASE_HOLD:
            self.setpoint = ROLL_TEST_TARGET_DEG
            self.setpoint_rate = 0.0
            if current_time - self._hold_start >= ROLL_TEST_HOLD_S:
                self.phase = PHASE_RAMP_DOWN

        elif self.phase == PHASE_RAMP_DOWN:
            self.setpoint = max(0.0,
                                self.setpoint - ROLL_TEST_SLEW_RATE_DEG_S * dt)
            self.setpoint_rate = -ROLL_TEST_SLEW_RATE_DEG_S
            if self.setpoint <= 0.0 and abs(roll_angle) <= ROLL_TEST_ANGLE_TOL_DEG:
                self.setpoint = 0.0
                self.setpoint_rate = 0.0
                command = angle_tracking_command(
                    self.setpoint, self.setpoint_rate, roll_angle, roll_rate)
                return command, self.phase, True, False

        command = angle_tracking_command(
            self.setpoint, self.setpoint_rate, roll_angle, roll_rate)
        return command, self.phase, False, False


# -----------------------------
# Main loop
# -----------------------------


def main():
    args = parse_args()
    telemetry = LiveTelemetry()
    esp32 = open_command_link(not args.no_commands)

    os.makedirs(LOGS_DIR, exist_ok=True)
    conn = init_db(DB_PATH)

    print("Flight roll controller running — requires GPSReader.py running first.")
    print(f"Roll test: +{ROLL_TEST_TARGET_DEG:.0f} deg at {ROLL_TEST_SLEW_RATE_DEG_S:.0f} deg/s, "
          f"armed by {ROLL_TEST_ARM_SAMPLES} samples > {ROLL_TEST_ALTITUDE_M:.0f} m.")
    print(f"Command link: {'disabled' if not esp32 else f'{COMMAND_HOST}:{COMMAND_PORT}'}")
    print(f"Log:          {DB_PATH}")
    print("Press Ctrl+C to stop and send neutral.")

    state = STATE_PRE_TEST
    roll_test = None
    phase = None

    roll_angle = 0.0            # gyro-integrated, only meaningful during the test
    alt_above_count = 0         # consecutive $ALT samples above trigger
    last_alt_seq = 0
    last_loop_time = time.monotonic()

    try:
        while True:
            loop_time = time.monotonic()
            dt = clamp(loop_time - last_loop_time, 0.0, MAX_INTEGRATION_DT_S)
            last_loop_time = loop_time

            snapshot = telemetry.latest()
            fresh = telemetry.is_fresh()
            # +gyro_z = right roll (confirmed on airframe). Right = +90 deg.
            roll_rate = snapshot.gyro_z if snapshot.gyro_z is not None else 0.0

            # --- Terminal fail-safe: canards centered for the rest of the flight ---
            if state == STATE_FAILSAFE_LOCK:
                fin_command = SERVO_NEUTRAL_COMMAND
                phase = None

            # --- Stale telemetry ---
            elif not fresh:
                if state == STATE_ROLL_TEST:
                    # Never continue an open-loop angle maneuver on bad data.
                    state = STATE_FAILSAFE_LOCK
                    roll_test = None
                    phase = None
                # PRE/POST: safe to pause damping; hold neutral, recover when fresh.
                fin_command = SERVO_NEUTRAL_COMMAND

            # --- PRE_TEST: rate damping + arming ---
            elif state == STATE_PRE_TEST:
                fin_command = rate_damping_command(roll_rate)

                if snapshot.altitude_seq != last_alt_seq:
                    last_alt_seq = snapshot.altitude_seq
                    if (snapshot.altitude_m is not None
                            and snapshot.altitude_m > ROLL_TEST_ALTITUDE_M):
                        alt_above_count += 1
                    else:
                        alt_above_count = 0

                if alt_above_count >= ROLL_TEST_ARM_SAMPLES:
                    # LATCH: one-time. Zero the roll-angle integrator here.
                    roll_angle = 0.0
                    roll_test = RollTest(start_time=loop_time)
                    state = STATE_ROLL_TEST
                    print(f"\n[{now()}] Roll test ARMED at "
                          f"alt={snapshot.altitude_m:.1f} m — executing.")

            # --- ROLL_TEST: integrate angle and track the setpoint ---
            elif state == STATE_ROLL_TEST:
                roll_angle += roll_rate * dt
                fin_command, phase, done, failed = roll_test.step(
                    dt, roll_angle, roll_rate, loop_time)
                if failed:
                    state = STATE_FAILSAFE_LOCK
                    roll_test = None
                    phase = None
                    print(f"\n[{now()}] Roll test TIMEOUT — canards locked neutral.")
                elif done:
                    state = STATE_POST_TEST
                    roll_test = None
                    phase = None
                    print(f"\n[{now()}] Roll test COMPLETE — resuming rate damping.")

            # --- POST_TEST: rate damping for the rest of the flight ---
            else:  # STATE_POST_TEST
                fin_command = rate_damping_command(roll_rate)

            if esp32:
                send_command_to_esp32(esp32, fin_command)

            # Differential deflection, matching the ESP32's setCanards(): canard 1
            # takes +fin_command, canard 2 takes -fin_command (deg from neutral).
            canard1_deg = fin_command
            canard2_deg = -fin_command

            alt_str = f"{snapshot.altitude_m:6.1f}" if snapshot.altitude_m is not None else "  ---"
            print(
                f"{state:<13} [{'LIVE' if fresh else 'STALE'}] "
                f"phase={phase or '-':<9} "
                f"rate={roll_rate:7.2f}  angle={roll_angle:7.2f}  "
                f"alt={alt_str}  cmd={fin_command:6.2f}  "
                f"canard1={canard1_deg:6.2f}  canard2={canard2_deg:6.2f}",
                end="\r",
                flush=True,
            )

            conn.execute(
                "INSERT INTO roll_control "
                "(timestamp, state, phase, fresh, altitude_m, roll_rate, "
                " roll_angle_deg, angle_setpoint, fin_command, canard1_deg, canard2_deg) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (now(), state, phase, int(fresh), snapshot.altitude_m, roll_rate,
                 roll_angle if state in (STATE_ROLL_TEST, STATE_POST_TEST) else None,
                 roll_test.setpoint if roll_test is not None else None,
                 fin_command, canard1_deg, canard2_deg),
            )
            conn.commit()

            time.sleep(LOOP_PERIOD_S)

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
