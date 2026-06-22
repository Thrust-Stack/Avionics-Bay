"""
Export a flight session from SQLite to a formatted Excel workbook.

Usage:
  python export_to_excel.py                    # converts the latest .db in logs/
  python export_to_excel.py logs/flight_X.db   # converts a specific file

Requires:  pip install openpyxl
"""

import sqlite3
import sys
import os

try:
    import openpyxl
    from openpyxl.styles import Font, PatternFill, Alignment
    from openpyxl.utils import get_column_letter
except ImportError:
    print("Missing openpyxl library. Install it:")
    print("  pip install openpyxl")
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(SCRIPT_DIR, "logs")

SHEETS = {
    "GPS": {
        "table": "gps",
        "columns": ["id", "timestamp", "lat", "lat_dir", "lon", "lon_dir",
                    "alt_msl", "speed_mph", "heading_deg", "heading_dir"],
        "headers": ["#", "Timestamp", "Latitude", "Dir", "Longitude", "Dir",
                    "Alt MSL (m)", "Speed (mph)", "Heading (°)", "Direction"],
    },
    "IMU": {
        "table": "imu",
        "columns": ["id", "timestamp", "accel_x", "accel_y", "accel_z",
                    "gyro_x", "gyro_y", "gyro_z"],
        "headers": ["#", "Timestamp", "Accel X (m/s²)", "Accel Y (m/s²)", "Accel Z (m/s²)",
                    "Gyro X (°/s)", "Gyro Y (°/s)", "Gyro Z (°/s)"],
    },
    "ALT": {
        "table": "alt",
        "columns": ["id", "timestamp", "agl_m"],
        "headers": ["#", "Timestamp", "AGL (m)"],
    },
}

HEADER_FILL = PatternFill("solid", fgColor="1F4E79")
HEADER_FONT = Font(bold=True, color="FFFFFF")


def auto_width(ws):
    for col in ws.columns:
        max_len = max(len(str(cell.value or "")) for cell in col)
        ws.column_dimensions[get_column_letter(col[0].column)].width = min(max_len + 4, 32)


def write_sheet(ws, conn, cfg):
    # Header row
    for ci, header in enumerate(cfg["headers"], 1):
        cell = ws.cell(row=1, column=ci, value=header)
        cell.font = HEADER_FONT
        cell.fill = HEADER_FILL
        cell.alignment = Alignment(horizontal="center")

    # Data rows
    rows = conn.execute(
        f"SELECT {', '.join(cfg['columns'])} FROM {cfg['table']} ORDER BY id"
    ).fetchall()

    for ri, row in enumerate(rows, 2):
        for ci, val in enumerate(row, 1):
            ws.cell(row=ri, column=ci, value=val)

    ws.freeze_panes = "B2"
    auto_width(ws)
    return len(rows)


def export(db_path):
    """Export db_path to an .xlsx beside it. Returns xlsx path, or None if file is locked."""
    xlsx_path = db_path.replace(".db", ".xlsx")
    conn = sqlite3.connect(db_path)

    wb = openpyxl.Workbook()
    wb.remove(wb.active)

    for sheet_name, cfg in SHEETS.items():
        ws = wb.create_sheet(sheet_name)
        write_sheet(ws, conn, cfg)

    conn.close()
    try:
        wb.save(xlsx_path)
    except PermissionError:
        return None  # file is open in Excel — skip this cycle
    return xlsx_path


def main():
    if len(sys.argv) > 1:
        db_path = sys.argv[1]
    else:
        db_path = os.path.join(LOGS_DIR, "flight_log.db")

    if not os.path.exists(db_path):
        print(f"File not found: {db_path}")
        sys.exit(1)

    xlsx_path = export(db_path)
    if xlsx_path:
        conn = sqlite3.connect(db_path)
        print(f"Saved: {xlsx_path}")
        for name, cfg in SHEETS.items():
            count = conn.execute(f"SELECT COUNT(*) FROM {cfg['table']}").fetchone()[0]
            print(f"  {name:4s}: {count} rows")
        conn.close()
    else:
        print(f"ERROR: {db_path.replace('.db', '.xlsx')} is open in another program. Close it and retry.")


if __name__ == "__main__":
    main()
