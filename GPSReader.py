"""
Avionics Telemetry - Laptop Side

Reads combined GPS + IMU + BMP585 data from the ESP32 bridge.
GPS data arrives as NMEA sentences, IMU as $IMU, BMP585 altitude as $ALT.

Setup:
  1. Upload avionics_bridge.ino to ESP32
  2. pip install pyserial pynmea2
  3. Update COM_PORT below
  4. python GPSReader.py
"""

import serial
import serial.tools.list_ports
import time
import sys

try:
    import pynmea2
except ImportError:
    print("Missing pynmea2 library. Install it:")
    print("  pip install pynmea2")
    sys.exit(1)

# ============================================================
# CONFIGURATION
# ============================================================
COM_PORT = None   # Set to e.g. "/dev/ttyUSB0" to override auto-detect
BAUD_RATE = 115200

# USB-serial chip descriptions used by common ESP32 dev boards
ESP32_KEYWORDS = ["cp210", "ch340", "ch341", "esp32", "uart", "usb serial", "usb-serial"]
# ============================================================


def find_esp32_port():
    """Return the first serial port that looks like an ESP32 USB-serial adapter."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()
        if any(kw in desc or kw in hwid for kw in ESP32_KEYWORDS):
            return port.device
    for port in ports:
        if "ttyusb" in port.device.lower() or "ttyacm" in port.device.lower():
            return port.device
    return None


def degrees_to_compass(degrees):
    """Convert heading in degrees to compass direction."""
    if degrees is None:
        return "---"
    d = float(degrees)
    if d >= 338 or d < 23:
        return "N"
    elif d < 68:
        return "NE"
    elif d < 113:
        return "E"
    elif d < 158:
        return "SE"
    elif d < 203:
        return "S"
    elif d < 248:
        return "SW"
    elif d < 293:
        return "W"
    else:
        return "NW"


def parse_bmp(line):
    """Parse custom $ALT sentence: $ALT,<agl_metres>"""
    try:
        parts = line.strip().split(",")
        if len(parts) != 2 or parts[0] != "$ALT":
            return None
        return float(parts[1])
    except (ValueError, IndexError):
        return None


def parse_imu(line):
    """Parse custom $IMU sentence from ESP32."""
    try:
        parts = line.strip().split(",")
        if len(parts) != 7 or parts[0] != "$IMU":
            return None

        ax = float(parts[1])
        ay = float(parts[2])
        az = float(parts[3])
        gx = float(parts[4])
        gy = float(parts[5])
        gz = float(parts[6])

        return (
            f"[IMU] Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f} m/s2  "
            f"Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f} deg/s"
        )
    except (ValueError, IndexError):
        return None


def parse_gps(line):
    """Parse NMEA sentences for GPS telemetry."""
    try:
        msg = pynmea2.parse(line)
    except (pynmea2.ParseError, ValueError):
        return None, None

    # GGA - grab MSL altitude
    if isinstance(msg, pynmea2.types.talker.GGA):
        try:
            if int(msg.gps_qual) > 0:
                return "alt", f"{float(msg.altitude):.1f}m"
        except (ValueError, AttributeError, TypeError):
            pass
        return "alt", None

    # RMC - main GPS fix line
    if isinstance(msg, pynmea2.types.talker.RMC):
        if msg.status != "A":
            return "no_fix", None

        try:
            lat = f"{msg.latitude:.6f}°{msg.lat_dir}"
            lon = f"{msg.longitude:.6f}°{msg.lon_dir}"
            speed_mph = msg.spd_over_grnd * 1.15078 if msg.spd_over_grnd else 0
            course = msg.true_course
        except (ValueError, AttributeError):
            return None, None

        if course is not None and course != "":
            compass = degrees_to_compass(course)
            heading = f"{float(course):.1f}° {compass}"
        else:
            heading = "---"

        return "gps", (lat, lon, speed_mph, heading)

    return None, None


def main():
    port = COM_PORT or find_esp32_port()
    if port is None:
        print("ERROR: No ESP32 port found. Plug in the ESP32 or set COM_PORT manually.")
        sys.exit(1)

    print("==========================================")
    print("  Avionics Telemetry")
    print("==========================================")
    print(f"  Port: {port}")
    print("  Press Ctrl+C to stop")
    print("==========================================\n")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}")
        print(f"  {e}")
        sys.exit(1)

    time.sleep(2)
    ser.reset_input_buffer()

    gps_fix = False
    latest_alt = "---"
    latest_bmp_alt = "---"
    gps_count = 0
    imu_count = 0
    alt_count = 0

    try:
        while True:
            line = ser.readline().decode("ascii", errors="replace").strip()

            if not line:
                continue

            # BMP585 AGL altitude at 10Hz
            if line.startswith("$ALT"):
                agl = parse_bmp(line)
                if agl is not None:
                    alt_count += 1
                    latest_bmp_alt = f"{agl:.1f}m"
                    print(f"[ALT] Altitude: {latest_bmp_alt}")
                continue

            # IMU data
            if line.startswith("$IMU"):
                parsed = parse_imu(line)
                if parsed:
                    imu_count += 1
                    print(parsed)
                continue

            # GPS NMEA data (may contain embedded $IMU if ESP32 concatenates lines)
            if line.startswith("$"):
                tokens = ["$" + seg for seg in line.split("$") if seg]
                for token in tokens:
                    if token.startswith("$IMU"):
                        parsed = parse_imu(token)
                        if parsed:
                            imu_count += 1
                            print(parsed)
                        continue

                    msg_type, data = parse_gps(token)

                    if msg_type == "alt":
                        if data:
                            latest_alt = data

                    elif msg_type == "no_fix":
                        if not gps_fix:
                            print("[GPS] Searching for satellites...")

                    elif msg_type == "gps":
                        lat, lon, speed, heading = data

                        if not gps_fix:
                            gps_fix = True
                            print("\n  GPS FIX ACQUIRED!\n")

                        gps_count += 1
                        print(
                            f"[GPS] Lat: {lat}  Lon: {lon}  "
                            f"ALT: {latest_alt}  "
                            f"Speed: {speed:.1f} mph  Heading: {heading}"
                        )

                continue

            # Non-sensor lines (ESP32 startup messages)
            if line.strip():
                print(f"[ESP32] {line}")

    except KeyboardInterrupt:
        print(f"\n\nStopped. GPS: {gps_count}  IMU: {imu_count}  ALT: {alt_count}")
        ser.close()


if __name__ == "__main__":
    main()