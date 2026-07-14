"""
Avionics Telemetry - Laptop Side

Reads combined GPS + IMU + BMP585 data from the ESP32 bridge.
GPS data arrives as NMEA sentences, IMU as $IMU, BMP585 altitude as $ALT.

Setup:
  1. Upload avionics_bridge.ino to ESP32
  2. pip install pyserial pynmea2
  3. Update COM_PORT below
  4. python GPSReader.py

Each session is saved to logs/flight_YYYYMMDD_HHMMSS.db
Run  python export_to_excel.py  after a session to convert to .xlsx
"""

import serial
import serial.tools.list_ports
import sqlite3
import datetime
import os
import time
import sys
import threading
import socket

try:
    import pynmea2
except ImportError:
    print("Missing pynmea2 library. Install it:")
    print("  pip install pynmea2")
    sys.exit(1)

from export_to_excel import export as export_xlsx

# ============================================================
# CONFIGURATION
# ============================================================
COM_PORT = "/dev/ttyAMA0"  # Hardware UART on Pi GPIO14(TX)/GPIO15(RX) — on Pi 5, /dev/serial0 is the debug connector, NOT the GPIO header
BAUD_RATE = 115200

# UDP port on localhost that GroundRollControlTest.py sends ROLL commands to.
# GPSReader.py forwards them to the ESP32 over the serial port it already holds.
COMMAND_UDP_PORT  = 5760  # receives ROLL commands from GroundRollControlTest.py
TELEMETRY_UDP_PORT = 5761  # broadcasts live $IMU packets to GroundRollControlTest.py

# USB-serial chip descriptions used by common ESP32 dev boards
ESP32_KEYWORDS = ["cp210", "ch340", "ch341", "esp32", "uart", "usb serial", "usb-serial"]

# How often to auto-export the .xlsx while a session is running (seconds)
XLSX_EXPORT_INTERVAL = 30
# ============================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(SCRIPT_DIR, "logs")


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


def init_db(path):
    """Create the SQLite database and tables for this session."""
    conn = sqlite3.connect(path)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS gps (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT    NOT NULL,
            lat         REAL    NOT NULL,
            lat_dir     TEXT    NOT NULL,
            lon         REAL    NOT NULL,
            lon_dir     TEXT    NOT NULL,
            alt_msl     REAL,
            speed_mph   REAL,
            heading_deg REAL,
            heading_dir TEXT
        )
    """)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS imu (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT NOT NULL,
            accel_x     REAL NOT NULL,
            accel_y     REAL NOT NULL,
            accel_z     REAL NOT NULL,
            gyro_x      REAL NOT NULL,
            gyro_y      REAL NOT NULL,
            gyro_z      REAL NOT NULL
        )
    """)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS alt (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp   TEXT NOT NULL,
            agl_m       REAL NOT NULL
        )
    """)
    conn.commit()
    return conn


def now():
    return datetime.datetime.now().isoformat(sep=" ", timespec="milliseconds")


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
    """Parse custom $IMU sentence. Returns (ax, ay, az, gx, gy, gz) or None."""
    try:
        parts = line.strip().split(",")
        if len(parts) != 7 or parts[0] != "$IMU":
            return None
        return tuple(float(p) for p in parts[1:])
    except (ValueError, IndexError):
        return None


def parse_gps(line):
    """Parse NMEA sentences for GPS telemetry."""
    try:
        msg = pynmea2.parse(line)
    except (pynmea2.ParseError, ValueError):
        return None, None

    # GGA - grab MSL altitude as raw float
    if isinstance(msg, pynmea2.types.talker.GGA):
        try:
            if int(msg.gps_qual) > 0:
                return "alt", float(msg.altitude)
        except (ValueError, AttributeError, TypeError):
            pass
        return "alt", None

    # RMC - main GPS fix line
    if isinstance(msg, pynmea2.types.talker.RMC):
        if msg.status != "A":
            return "no_fix", None

        try:
            speed_mph = float(msg.spd_over_grnd) * 1.15078 if msg.spd_over_grnd else 0.0
            course = float(msg.true_course) if msg.true_course else None
        except (ValueError, AttributeError):
            return None, None

        compass = degrees_to_compass(course) if course is not None else "---"
        return "gps", (msg.latitude, msg.lat_dir, msg.longitude, msg.lon_dir,
                        speed_mph, course, compass)

    return None, None


def command_relay(ser, stop_event):
    """Receive ROLL commands via UDP from GroundRollControlTest.py and forward to ESP32."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", COMMAND_UDP_PORT))
    sock.settimeout(0.1)
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(64)
            ser.write(data)
        except socket.timeout:
            pass
    sock.close()


def xlsx_exporter(db_path, stop_event):
    """Background thread: re-exports .xlsx every XLSX_EXPORT_INTERVAL seconds."""
    while not stop_event.wait(XLSX_EXPORT_INTERVAL):
        result = export_xlsx(db_path)
        if result:
            print(f"[LOG] .xlsx updated: {os.path.basename(result)}")
        else:
            print("[LOG] .xlsx skipped (file open in another program)")


def log_imu(conn, vals):
    ax, ay, az, gx, gy, gz = vals
    conn.execute(
        "INSERT INTO imu (timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z) "
        "VALUES (?, ?, ?, ?, ?, ?, ?)",
        (now(), ax, ay, az, gx, gy, gz)
    )
    conn.commit()


def main():
    port = COM_PORT or find_esp32_port()
    if port is None:
        print("ERROR: No ESP32 port found. Plug in the ESP32 or set COM_PORT manually.")
        sys.exit(1)

    os.makedirs(LOGS_DIR, exist_ok=True)
    db_path = os.path.join(LOGS_DIR, "flight_log.db")
    conn = init_db(db_path)

    print("==========================================")
    print("  Avionics Telemetry")
    print("==========================================")
    print(f"  Port:    {port}")
    print(f"  Log:     {db_path}")
    print("  Press Ctrl+C to stop")
    print("==========================================\n")

    # Start background thread that refreshes the .xlsx every XLSX_EXPORT_INTERVAL seconds
    stop_event = threading.Event()
    exporter = threading.Thread(target=xlsx_exporter, args=(db_path, stop_event), daemon=True)
    exporter.start()

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}")
        print(f"  {e}")
        sys.exit(1)

    relay = threading.Thread(target=command_relay, args=(ser, stop_event), daemon=True)
    relay.start()

    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    time.sleep(2)
    ser.reset_input_buffer()

    gps_fix = False
    latest_alt_msl = None   # raw float, updated from GGA
    latest_alt_str = "---"  # display string
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
                    telem_sock.sendto(line.encode("ascii"), ("127.0.0.1", TELEMETRY_UDP_PORT))
                    print(f"[ALT] Altitude: {agl:.1f}m")
                    conn.execute(
                        "INSERT INTO alt (timestamp, agl_m) VALUES (?, ?)",
                        (now(), agl)
                    )
                    conn.commit()
                continue

            # IMU data
            if line.startswith("$IMU"):
                vals = parse_imu(line)
                if vals:
                    ax, ay, az, gx, gy, gz = vals
                    imu_count += 1
                    telem_sock.sendto(line.encode("ascii"), ("127.0.0.1", TELEMETRY_UDP_PORT))
                    print(
                        f"[IMU] Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f} m/s2  "
                        f"Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f} deg/s"
                    )
                    log_imu(conn, vals)
                continue

            # GPS NMEA data (may contain embedded $IMU if ESP32 concatenates lines)
            if line.startswith("$"):
                tokens = ["$" + seg for seg in line.split("$") if seg]
                for token in tokens:
                    if token.startswith("$IMU"):
                        vals = parse_imu(token)
                        if vals:
                            ax, ay, az, gx, gy, gz = vals
                            imu_count += 1
                            telem_sock.sendto(token.encode("ascii"), ("127.0.0.1", TELEMETRY_UDP_PORT))
                            print(
                                f"[IMU] Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f} m/s2  "
                                f"Gyro: X={gx:.2f} Y={gy:.2f} Z={gz:.2f} deg/s"
                            )
                            log_imu(conn, vals)
                        continue

                    msg_type, data = parse_gps(token)

                    if msg_type == "alt":
                        if data is not None:
                            latest_alt_msl = data
                            latest_alt_str = f"{data:.1f}m"

                    elif msg_type == "no_fix":
                        if not gps_fix:
                            print("[GPS] Searching for satellites...")

                    elif msg_type == "gps":
                        lat, lat_dir, lon, lon_dir, speed, course, compass = data

                        if not gps_fix:
                            gps_fix = True
                            print("\n  GPS FIX ACQUIRED!\n")

                        heading_str = f"{course:.1f}° {compass}" if course is not None else "---"
                        gps_count += 1
                        print(
                            f"[GPS] Lat: {lat:.6f}°{lat_dir}  Lon: {lon:.6f}°{lon_dir}  "
                            f"ALT: {latest_alt_str}  "
                            f"Speed: {speed:.1f} mph  Heading: {heading_str}"
                        )
                        conn.execute(
                            "INSERT INTO gps "
                            "(timestamp, lat, lat_dir, lon, lon_dir, alt_msl, speed_mph, heading_deg, heading_dir) "
                            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                            (now(), lat, lat_dir, lon, lon_dir, latest_alt_msl, speed, course, compass)
                        )
                        conn.commit()

                continue

            # Non-sensor lines (ESP32 startup messages)
            if line.strip():
                print(f"[ESP32] {line}")

    except KeyboardInterrupt:
        stop_event.set()
        print(f"\n\nStopped.  GPS: {gps_count}  IMU: {imu_count}  ALT: {alt_count}")
        print(f"Session saved to: {db_path}")
        print("Exporting final .xlsx...")
        result = export_xlsx(db_path)
        if result:
            print(f"Exported: {result}")
        else:
            print("Could not write .xlsx (file open in another program). Run export_to_excel.py manually.")
        conn.close()
        ser.close()


if __name__ == "__main__":
    main()
