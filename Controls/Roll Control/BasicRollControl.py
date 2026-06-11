import time
import serial
import csv

# -----------------------------
# Configuration
# -----------------------------

TARGET_ROLL_RATE = 0.0        # deg/s
KP = 0.02                     # proportional gain
MAX_FIN_DEFLECTION = 20.0     # degrees

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# -----------------------------
# Safety Parameters
# -----------------------------

MIN_CONTROL_ALTITUDE_M = 20.0

MAX_X_ROTATION_DEG = 90.0
MAX_Y_ROTATION_DEG = 90.0

SERVO_NEUTRAL_COMMAND = 0.0

# -----------------------------
# Setup
# -----------------------------

esp32 = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

log_file = open("flight_log.csv", "w", newline="")
logger = csv.writer(log_file)

logger.writerow([
    "time",
    "state",
    "altitude_m",
    "rotation_x_deg",
    "rotation_y_deg",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "roll_rate",
    "control_allowed",
    "fin_command"
])

state = "IDLE"

# -----------------------------
# Helper functions
# -----------------------------

def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def read_gyro():
    """
    Replace this with real IMU code.
    Return gyro values in deg/s.
    """
    gyro_x = 0.0
    gyro_y = 0.0
    gyro_z = 0.0

    return gyro_x, gyro_y, gyro_z


def read_altitude():
    """
    Replace this with real altimeter code.
    Return altitude above launch site in meters.
    """
    altitude_m = 0.0

    return altitude_m


def estimate_rotation_x_y():
    """
    Replace this with real attitude estimation code.

    For now, return estimated x and y rotational positions in degrees.
    These are probably pitch and yaw angles.
    """
    rotation_x_deg = 0.0
    rotation_y_deg = 0.0

    return rotation_x_deg, rotation_y_deg


def control_is_allowed(altitude_m, rotation_x_deg, rotation_y_deg):
    """
    Roll control is only allowed if:
    - altitude is above 20 meters
    - x rotation is within +/- 90 degrees
    - y rotation is within +/- 90 degrees
    """

    altitude_ok = altitude_m > MIN_CONTROL_ALTITUDE_M
    x_rotation_ok = abs(rotation_x_deg) < MAX_X_ROTATION_DEG
    y_rotation_ok = abs(rotation_y_deg) < MAX_Y_ROTATION_DEG

    return altitude_ok and x_rotation_ok and y_rotation_ok


def calculate_fin_command(roll_rate):
    error = TARGET_ROLL_RATE - roll_rate
    command = KP * error
    command = clamp(command, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)

    return command


def send_command_to_esp32(fin_command):
    message = f"ROLL,{fin_command:.2f}\n"
    esp32.write(message.encode("utf-8"))


# -----------------------------
# Main loop
# -----------------------------

try:
    while True:
        now = time.time()

        altitude_m = read_altitude()
        rotation_x_deg, rotation_y_deg = estimate_rotation_x_y()

        gyro_x, gyro_y, gyro_z = read_gyro()

        # Choose the gyro axis that matches the rocket's roll axis.
        # Example: assume gyro_z is roll rate.
        roll_rate = gyro_z

        allowed = control_is_allowed(
            altitude_m,
            rotation_x_deg,
            rotation_y_deg
        )

        if state == "CONTROL_ACTIVE" and allowed:
            fin_command = calculate_fin_command(roll_rate)
        else:
            fin_command = SERVO_NEUTRAL_COMMAND

        send_command_to_esp32(fin_command)

        logger.writerow([
            now,
            state,
            altitude_m,
            rotation_x_deg,
            rotation_y_deg,
            gyro_x,
            gyro_y,
            gyro_z,
            roll_rate,
            allowed,
            fin_command
        ])

        log_file.flush()

        time.sleep(0.01)  # 100 Hz loop

except KeyboardInterrupt:
    send_command_to_esp32(SERVO_NEUTRAL_COMMAND)
    log_file.close()
    esp32.close()