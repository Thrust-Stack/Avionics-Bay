# Avionics-Bay

Avionics bay for the **Thrust-Stack Tilt & Roll Control** high power model rocket:
an active canard-based **roll control** system, its sensor/telemetry stack, and
the airframe CAD.

> **Read this first — repository layout is unusual.**
> The `main` branch (which you are reading) currently contains **only the
> mechanical CAD** (`Tilt & Roll Control Project/`) and this README. **None of
> the firmware or control software is on `main`.** It lives on separate feature
> branches:
>
> | Branch | Contents |
> |--------|----------|
> | [`Controls`](../../tree/Controls) | Most complete software set — ESP32 firmware, roll-control laws, GPS/telemetry logging, LoRa telemetry, wiring reference |
> | [`Raspi`](../../tree/Raspi) | Raspberry Pi 5 camera capture + a copy of the control/GPS programs |
> | [`Meshtastic-Programs`](../../tree/Meshtastic-Programs) | Meshtastic serial send/receive experiments + wiring reference |
>
> File paths and code excerpts below are taken from the **`Controls`** branch
> unless noted. Nothing here has been merged into `main`.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Architecture](#2-hardware-architecture)
   - [2.1 Components](#21-components)
   - [2.2 Buses and Wiring](#22-buses-and-wiring)
   - [2.3 Power System](#23-power-system)
   - [2.4 Wiring Diagram / Schematic (placeholder)](#24-wiring-diagram--schematic-placeholder)
   - [2.5 Documentation vs. firmware discrepancies](#25-documentation-vs-firmware-discrepancies)
3. [Software Architecture](#3-software-architecture)
   - [3.1 Data path](#31-data-path)
   - [3.2 Firmware / module reference](#32-firmware--module-reference)
   - [3.3 Control loop](#33-control-loop)
   - [3.4 Arming / safety state logic](#34-arming--safety-state-logic)
4. [Build and Flash Instructions](#4-build-and-flash-instructions)
5. [Repository Structure](#5-repository-structure)
6. [Safety Notes](#6-safety-notes)
7. [Status / Known Limitations](#7-status--known-limitations)

---

## 1. Project Overview

This repository is the avionics bay and control system for an active
**tilt/roll control** high power model rocket built by Thrust-Stack, intended
to fly on an **AeroTech H219** motor. The airframe carries movable **canards**
whose deflection generates a roll moment; the goal of the avionics bay is to
**hold the airframe near zero roll rate during boost and coast**, rather than
letting the rocket spin up on its own. On-board sensors measure angular rate,
acceleration, barometric altitude, and GPS position; a control law converts
roll-rate error into a commanded canard deflection.

> **What is actually implemented vs. the project name.** The folder is named
> *Tilt & Roll Control*, and the CAD includes both static fins and movable
> canards, but the **firmware and control laws in this repo implement roll
> control only** — a single signed deflection applied to two mirrored canards.
> Tilt (pitch/yaw) is *estimated* from the accelerometer and used as an input
> to the ground-test controller, but there is no thrust-vectoring or 4-axis fin
> control in the flashed firmware. See [§7](#7-status--known-limitations).
>
> The AeroTech H219 is the project's stated flight motor; the repository itself
> contains **no motor data or thrust curve** (the CAD `Fake Motor` parts are
> inert mass/fit models used for CFD and assembly, not the real motor).

---

## 2. Hardware Architecture

The wiring below reflects the **current physical harness** (I²C on GPIO 23/32,
direct-PWM canard servos, and a wired ESP32 ↔ Raspberry Pi 5 UART link). Where
older firmware or the historical branch wiring reference disagrees with this
harness, the conflict is called out in
[§2.5](#25-documentation-vs-firmware-discrepancies) — reconcile against the
sketch actually flashed before flight.

### 2.1 Components

| Role | Part | Interface | Address / pins / rate |
|------|------|-----------|------------------------|
| Flight MCU / sensor bridge | **ESP32**, board `NodeMCU-32S` | — | USB serial @ **115200** to host |
| IMU (accel + gyro) | **MPU6050** (Adafruit driver) | I²C (SDA GPIO23 / SCL GPIO32) | **AD0 → GND** (nominal `0x68`) ¹; ±16 g accel, ±500 °/s gyro, 21 Hz DLPF; sampled @ **20 Hz** |
| Barometric altimeter | **BMP585** (Adafruit `BMP5xx`) | I²C (SDA GPIO23 / SCL GPIO32) | `BMP5XX_DEFAULT_ADDRESS` (**`0x46`** per code comment) ¹; 4× pressure OSR, IIR coeff 3; sampled @ **10 Hz** |
| GPS | **Adafruit Ultimate GPS V3** | UART (ESP32 `Serial2`) | **9600** baud, NMEA; GPS TX → GPIO16, GPS RX → GPIO17; powered from ESP32 3V3 |
| Roll actuators | **2 × canard servos** (wiring ref names *BMS-127WV+*) | Direct ESP32 PWM | **Canard 1 → GPIO26, Canard 2 → GPIO25**; neutral 90°; ±15° max deflection |
| Onboard computer | **Raspberry Pi 5** | UART to ESP32 | Pi GPIO14 (TX) → ESP32 **GPIO27** (RX); Pi GPIO15 (RX) ← ESP32 **GPIO33** (TX); common GND required |
| Telemetry radio (optional) | **Heltec WiFi LoRa 32 V4** (Semtech **SX1262**) | SPI + LoRa | 915 MHz, SF7, BW 125 kHz, CR 4/5, +14 dBm, 2 Hz |
| Ground station | Laptop running the Python stack | USB serial + UDP | — |

¹ With AD0 physically tied to GND the MPU6050 should respond at `0x68`, but the
`Controls`-branch firmware historically defined `MPU6050_ADDR = 0x69` with the
comment *"AD0 reads high on this board despite being wired to GND."* The old
wiring reference also listed the BMP585 at `0x47` in places. Confirm both
addresses with an I²C scanner on the current harness before flight.

> **PCA9685 removed from the harness.** Earlier revisions drove the canards
> through a PCA9685 16-channel PWM driver (I²C `0x40`, channels 12/13). The
> current wiring drives both servos **directly from ESP32 GPIO 25/26** and the
> PCA9685 is no longer part of the signal chain. If the flashed sketch still
> initializes a PCA9685, it does not match this harness — see
> [§2.5](#25-documentation-vs-firmware-discrepancies).

### 2.2 Buses and Wiring

Current harness, ESP32 as the hub:

```text
BMP585 ──┐
          ├── I²C (SDA GPIO23 / SCL GPIO32) ──► ESP32 ──USB serial 115200──► Laptop
MPU6050 ─┘   (MPU AD0 → GND)                     │
                                                  ├── UART2 (RX GPIO16 ◄─ GPS TX,
                                                  │          TX GPIO17 ─► GPS RX, 9600)
                                                  ├── GPIO26 ─► Canard 1 servo PWM
                                                  ├── GPIO25 ─► Canard 2 servo PWM
                                                  └── UART (RX GPIO27 ◄─ Pi GPIO14 TX,
                                                            TX GPIO33 ─► Pi GPIO15 RX)
                                                                 Raspberry Pi 5
                                                            (common GND with ESP32)
```

Pin-by-pin:

| Signal | From | To |
|--------|------|-----|
| GPS VIN | ESP32 **3V3** | GPS VIN |
| GPS GND | ESP32 GND | GPS GND |
| GPS TX | GPS | ESP32 **GPIO16** (Serial2 RX) |
| GPS RX | GPS | ESP32 **GPIO17** (Serial2 TX) |
| I²C SDA (shared) | MPU6050 + BMP585 SDA | ESP32 **GPIO23** |
| I²C SCL (shared) | MPU6050 + BMP585 SCL | ESP32 **GPIO32** |
| MPU6050 AD0 | MPU6050 | ESP32 GND |
| Canard 1 servo PWM | ESP32 **GPIO26** | Servo 1 signal |
| Canard 2 servo PWM | ESP32 **GPIO25** | Servo 2 signal |
| Pi TX → ESP32 RX | Pi **GPIO14** | ESP32 **GPIO27** |
| Pi RX ← ESP32 TX | Pi **GPIO15** | ESP32 **GPIO33** |
| Pi GND | Pi | ESP32 GND (**common ground required**) |

- **I²C bus (shared):** MPU6050 and BMP585 share `SDA = GPIO23`,
  `SCL = GPIO32`, with unique addresses.
- **GPS:** UART on `Serial2` (GPIO16/17), 9600 baud; raw NMEA is forwarded to
  the host.
- **Actuators:** the two mirrored canards are driven by **direct ESP32 PWM** on
  GPIO26 (canard 1) and GPIO25 (canard 2). No external PWM driver.
- **Pi 5 link:** a dedicated UART now physically connects the ESP32
  (GPIO27 RX / GPIO33 TX) to the Pi's GPIO14/15, with grounds commoned. Verify
  which sketch/host program actually uses this link — the `Controls`-branch
  bridge historically streamed over **USB serial to a laptop**
  ([§2.5](#25-documentation-vs-firmware-discrepancies)).
- **Telemetry radio (separate board):** The Heltec V4 pinout is SPI
  (`SCK 9 / MISO 11 / MOSI 10 / NSS 8`), `DIO1 14`, `RST 12`, `BUSY 13`, plus
  RF front-end enable on `GPIO2`. It is an independent link, not wired into the
  ESP32 sensor bridge.

### 2.3 Power System

| Component | Voltage | Source |
|-----------|---------|--------|
| ESP32 | 5 V | USB or 5 V regulator |
| Raspberry Pi 5 | 5 V | USB-C PD or 5V-5A battery |
| GPS V3 / BMP585 / MPU6050 | 3.3 V | ESP32 `3V3` pin |
| Canard servos | 7.4 V 2S LiPo with a 2200 μF bulk cap — *the wiring reference also mentions a 5 V BEC; reconcile before flight* | Dedicated rail, **never** the ESP32 |

- Sensors and the GPS run off the ESP32's onboard 3.3 V regulator (AMS1117).
  Note this regulator has limited headroom with GPS + BMP585 + MPU6050 all on
  the 3V3 pin; watch for brownouts under load and consider a dedicated 3.3 V
  buck for the sensor rail.
- Servos must be powered from a **dedicated rail** (a 5 V BEC rated ≥5 A per the
  doc, 8–10 A recommended; or a 7.4 V 2S LiPo). The wiring reference is
  internally inconsistent about which — verify your servo voltage rating (2S is
  8.4 V fully charged) before connecting.
- **All grounds must be common:** ESP32, servo power rail, sensors, GPS, and Pi.
  The Pi ↔ ESP32 ground link is explicitly part of the harness (see §2.2).

### 2.4 Wiring Diagram / Schematic (placeholder)

> **No schematic or wiring image exists in the repository yet.** The pin table
> in §2.2 above is the current authoritative connection list. A proper
> KiCad/schematic capture or a labeled wiring photo should be added here:
>
> ```
> docs/wiring-diagram.png   <-- TODO: add schematic / harness diagram
> ```

### 2.5 Documentation vs. firmware discrepancies

The harness in §2.2 is the **current physical wiring**. The `Controls`-branch
firmware and the older branch wiring reference were written against earlier
revisions and may not match it. **Verify the flashed sketch against this table
before flight:**

| Topic | Current harness (§2.2) | Older firmware / wiring reference |
|-------|------------------------|-----------------------------------|
| I²C pins | SDA **GPIO23** / SCL **GPIO32** | Firmware: `Wire.begin(21, 22)` |
| Actuators | **2 canards, direct ESP32 PWM** on GPIO26 (canard 1) / GPIO25 (canard 2) | Firmware: 2 canards via **PCA9685** (I²C `0x40`) ch 12/13 @ 50 Hz; old wiring ref: 4 servos on GPIO 13/14/25/26 @ 250 Hz |
| PCA9685 | **Not in the harness** | Firmware initializes it; old wiring ref maps servos to ch 0–3 in one section, ch 12/13 in the firmware |
| MPU6050 address | AD0 → GND (nominal **`0x68`**) | Firmware: `0x69` (comment: "AD0 reads high on this board despite being wired to GND") — rescan on the current board |
| BMP585 address | `0x46` (code comment) | Old wiring ref: `0x47` in places; firmware error string separately mentions `0x47` |
| ESP32 ↔ Pi 5 UART | **Physically wired**: Pi GPIO14/15 ↔ ESP32 GPIO27/33, common GND | Firmware: link **not implemented** — the flashed bridge streams over USB serial to a laptop; old wiring ref described UART1 on GPIO 2/4 |

---

## 3. Software Architecture

### 3.1 Data path

The implemented system is a **hardware-in-the-loop link over USB + localhost
UDP**, not a self-contained onboard controller:

```text
[ESP32 bridge] --USB serial 115200--> [GPSReader.py]
      ^  ($IMU 20Hz, $ALT 10Hz, NMEA passthrough)   |
      |                                              | UDP 5761 (telemetry: $IMU/$ALT/$GPS)
      |                                              v
      |                                    [BasicRollControl.py  OR
      |                                     GroundRollControlTest.py]
      |                                              |
      +-------- serial <-- UDP 5760 (ROLL,<deg>) ----+
                (GPSReader relays UDP 5760 -> ESP32 serial)
```

- ESP32 emits `$IMU,ax,ay,az,gx,gy,gz` (accel m/s², **gyro rad/s** from the
  Adafruit driver) at 20 Hz, `$ALT,<agl_m>` (ground-zeroed barometric altitude)
  at 10 Hz, and passes GPS NMEA through unchanged.
- `GPSReader.py` logs everything to SQLite and re-broadcasts sensor packets on
  UDP `127.0.0.1:5761`. It relays inbound `ROLL,<deg>` commands from UDP
  `5760` straight to the ESP32 over the serial port it already holds.
- A controller listens on 5761, computes a deflection, and sends `ROLL,<deg>`
  to 5760. The ESP32 applies the same signed deflection to both canards.
- **Pi 5 UART:** the ESP32 ↔ Pi UART link (ESP32 GPIO27/33 ↔ Pi GPIO14/15) is
  now **physically wired** (§2.2), but the `Controls`-branch software path
  above still runs over USB serial to a laptop. Software support for the Pi
  link (on both the ESP32 sketch and a Pi-side reader) must be confirmed or
  added before it can replace the USB path.

### 3.2 Firmware / module reference

| File (branch) | Runs on | Responsibility |
|---------------|---------|----------------|
| `Esp32 Programs/Esp32_sensor_bridge/Esp32_sensor_bridge.ino` (Controls) | ESP32 | Reads MPU6050 + BMP585 (I²C) and GPS (UART2); streams `$IMU`/`$ALT`/NMEA over USB; parses `ROLL,<deg>` and drives the canards. Zeroes ground pressure at boot (20-sample average). **Note:** the checked-in version drives the canards via PCA9685 and uses I²C on GPIO21/22 — it predates the direct-PWM / GPIO23-32 harness in §2.2. |
| `GPSReader.py` (Controls/Raspi) | Laptop | Auto-detects the ESP32 COM port; parses `$IMU`/`$ALT`/NMEA; logs to `logs/flight_*.db` (SQLite) and auto-exports `.xlsx`; UDP telemetry re-broadcast + command relay. |
| `Controls/Roll Control/BasicRollControl.py` (Controls) | Laptop | **Flight-intent** roll controller: Kalman-filtered roll rate, multi-sensor velocity fusion, velocity-scheduled gain, altitude/tilt/staleness safety gate. |
| `Controls/Roll Control/GroundRollControlTest.py` (Controls) | Laptop | **Bench/ground** controller: aggressive fixed gains, ±15° authority, roll-rate + tilt terms. Explicitly **bypasses altitude safety — not a flight controller.** |
| `export_to_excel.py` (Controls/Raspi) | Laptop | Converts a session `.db` into a formatted 3-sheet (GPS/IMU/ALT) Excel workbook. |
| `Communication/HeltecV4TelemetryTransmitter.ino` / `…Receiver.ino` (Controls) | 2× Heltec V4 | Point-to-point 20-byte LoRa telemetry (SX1262 via RadioLib) with CRC-16/CCITT and a packet counter for loss detection. **Transmitter payload is placeholder data** (`fillSensorData()` hardcodes example values). |
| `Communication/heltec_link_test.py` (Controls) | 2× laptops | tx/rx/compare link-quality test that reports delivered %, RSSI, SNR, and missing packet IDs. |
| `Communication/Sender.py` / `Reciever.py` (Controls/Meshtastic) | Laptop | Meshtastic serial send/receive experiments. `Sender.py` transmits **placeholder joke strings**, not telemetry. |
| `Camera/camera_capture.py` (Raspi) | Raspberry Pi 5 | Picamera2 H.264/MP4 video, periodic JPEGs, and full-res stills. Never raises on camera failure — isolated from the rest of the stack. |

### 3.3 Control loop

**`BasicRollControl.py` (flight-intent), ~100 Hz:**

1. Receive `$IMU`/`$ALT`/`$GPS` snapshots over UDP; convert gyro rad/s → deg/s.
2. Estimate roll rate with a scalar **Kalman filter** (`gyro_z` taken as roll
   axis).
3. Estimate vertical velocity with a **1-state Kalman velocity-fusion**
   estimator combining GPS vertical velocity, integrated accelerometer, and
   differentiated barometric altitude, with innovation gating.
4. **Gain scheduling:** below 0.1 m/s use nominal `KP`; above it, scale gain by
   `KP / (CANARD_ACCELERATION_COEFFICIENT · v²)` so control authority tracks
   dynamic pressure.
5. Command `fin = clamp(gain · (0 − roll_rate), ±7.5°)`, send `ROLL,<deg>`.
6. On exit / stale telemetry, command neutral (0°).

**`GroundRollControlTest.py` (bench), continuous:** fixed `ROLL_RATE_GAIN = 1.4`,
`TILT_GAIN = 2.0`, ±15° authority; `command = 1.4·roll_rate_error − 2.0·tilt_y`;
tilt estimated from the accelerometer (valid only for slow/ground motion).

**ESP32 `setCanards()`:** clamps to ±15°, applies the same signed deflection to
both canards around 90° neutral (plus per-canard trim constants, currently 0).
In the current harness the outputs are direct PWM on GPIO26/25 (§2.2); the
checked-in sketch still targets PCA9685 channels
([§2.5](#25-documentation-vs-firmware-discrepancies)).

### 3.4 Arming / safety state logic

There is **no hardware arm switch, insert-to-arm pin, or pyro interlock** in the
firmware. "Arming" is entirely **software gating** in the host controller:

- **`BasicRollControl.py`** runs a two-state machine:
  `CONTROL_ACTIVE` only when `control_is_allowed()` (altitude ≥ 0, |tilt_x| <
  90°, |tilt_y| < 90°) **and** telemetry is fresh (an IMU packet within 0.5 s);
  otherwise `IDLE` → **neutral command**. Loss of telemetry → neutral.
- **`GroundRollControlTest.py`** has only a freshness gate
  (`GROUND_TEST_ACTIVE` vs `STALE_TELEMETRY` → neutral) and **deliberately
  bypasses the altitude/tilt safety logic** — its own argparse help says *"do
  not use as flight controller."*
- **On the ESP32**, canards are parked at neutral (90°) at boot; the servos will
  physically move to neutral as soon as the board powers up.

---

## 4. Build and Flash Instructions

There is **no Makefile, PlatformIO project, or CMake** anywhere in the repo. The
firmware is Arduino sketches and the host code is plain Python scripts.

### 4.1 ESP32 sensor bridge (`Esp32_sensor_bridge.ino`)

- **Board:** `NodeMCU-32S` (ESP32) in the Arduino IDE / `arduino-cli`.
- **Libraries** (Library Manager, per the sketch header): Adafruit MPU6050,
  Adafruit BMP5xx, Adafruit Unified Sensor. (The Adafruit PWM Servo Driver
  Library was required by the PCA9685-era sketch; a direct-PWM build will use
  `ESP32Servo` or LEDC instead — match the sketch you flash.)
- Open `Esp32 Programs/Esp32_sensor_bridge/Esp32_sensor_bridge.ino`, select the
  board and port, and Upload.
- **Verify pins against §2.2 before flashing:** the checked-in sketch predates
  the current harness (I²C on 21/22, PCA9685 actuation). Update `Wire.begin()`
  to (23, 32) and the servo output path to direct PWM on GPIO26/25 — or rewire —
  before running the control loop.

### 4.2 Heltec V4 LoRa telemetry (`HeltecV4Telemetry*/…ino`)

- **Library:** RadioLib (Library Manager).
- Select the **exact Heltec WiFi LoRa 32 V4** board revision; attach the correct
  regional LoRa antenna **before powering** (there is a separate 2.4 GHz
  connector — do not confuse them).
- Flash the transmitter to the avionics board and the receiver to the ground
  board; open both serial monitors at 115200. For the transmitter's TX log lines
  over USB, compile with **USB CDC On Boot: Enabled**.

### 4.3 Host Python stack (laptop)

Dependencies are documented per-script; there is no single `requirements.txt`.
The only pinned file is `Communication/requirements-link-test.txt` (`pyserial`).

```bash
# GPS/telemetry logger + Excel export
pip install pyserial pynmea2 openpyxl
python GPSReader.py            # then, in another terminal:
python "Controls/Roll Control/BasicRollControl.py"       # or GroundRollControlTest.py
python export_to_excel.py     # convert the latest logs/*.db to .xlsx

# LoRa link-quality test (two computers)
py -m pip install -r Communication/requirements-link-test.txt
py Communication/heltec_link_test.py tx --port COM7 --duration 120 --output tx_report.json
py Communication/heltec_link_test.py rx --port COM9 --duration 120 --output rx_report.json
py Communication/heltec_link_test.py compare tx_report.json rx_report.json

# Meshtastic experiments
pip install meshtastic pypubsub
```

Use `--no-commands` on either controller to observe telemetry without moving
servos.

### 4.4 Raspberry Pi 5 camera (`camera_capture.py`, Raspi branch)

- **Picamera2 is installed via apt, not pip:** `sudo apt install -y
  python3-picamera2 ffmpeg`. If using a venv, create it with
  `--system-site-packages`.
- Verify with `rpicam-hello --list-cameras` (expect `imx708`), then:
  `python3 camera_capture.py --mode video --duration 30`.

---

## 5. Repository Structure

### `main` branch (this branch)

| Path | Purpose |
|------|---------|
| `README.md` | This document. |
| `Tilt & Roll Control Project/` | SolidWorks CAD for the airframe and bay. |
| `Tilt & Roll Control Project/*.SLDPRT` | Parts: canards, airfoil fins, avionics bay plates (top/bottom), body tubes, nose cone, couplers, servo mount/attachment, bearings & mounts, camera mount, Pi Camera Module 3, fin jig, "fake" motor/bearing (fit models). |
| `Tilt & Roll Control Project/*.SLDASM` | Assemblies: `Rocket Assembly (1–3)`, `Camera + Mount`, `Static Fin + Lower Bearing Mount`. |
| `Tilt & Roll Control Project/CFD Model (3)/` | CFD variant of the assembly + a `.STEP` export. |

> `main` contains **no source code, build files, or wiring diagrams** — only CAD
> and this README.

### Software branches (not merged into `main`)

| Path (on `Controls` / `Raspi`) | Purpose |
|--------------------------------|---------|
| `Esp32 Programs/Esp32_sensor_bridge/` | ESP32 sensor-bridge + roll-actuation firmware. |
| `GPSReader.py`, `export_to_excel.py` | Host logger and Excel export. |
| `Controls/Roll Control/` | `BasicRollControl.py` (flight-intent), `GroundRollControlTest.py` (bench). |
| `Communication/` | Heltec LoRa sketches + `heltec_link_test.py` + `HELTEC_V4_TELEMETRY.md`; Meshtastic `Sender.py`/`Reciever.py`. |
| `Camera/` (Raspi) | `camera_capture.py` + camera README. |
| `logs/` | Captured session `.db`/`.xlsx` files. |
| `README.md` (branch) | Historical wiring reference — superseded by §2.2 of this document; see [§2.5](#25-documentation-vs-firmware-discrepancies) for its known conflicts. |

---

## 6. Safety Notes

> ⚠️ This is a **high power rocket**. Treat every step below as capable of
> causing injury or fire.

- **Motor / launch (AeroTech H219):** High power motors require appropriate
  certification (e.g., NAR/Tripoli), an approved launch site, and FAA/landowner
  compliance. The repository contains no ignition or launch-detect control code;
  **motor ignition is an external, irreversible action** — arm the igniter last,
  from a safe distance, per range rules.
- **Recovery / ejection charges are NOT handled by this repo.** There is no
  pyro, ejection-charge, or deployment code anywhere in the firmware. If the
  airframe uses ejection charges or a separate commercial flight computer for
  recovery, that system is **out of scope here** and must be armed and verified
  independently. Do not assume this avionics bay provides recovery.
- **Servos move on power-up.** The ESP32 drives both canards to neutral the
  instant it boots. **Keep hands and tools clear of the canards** whenever the
  bay is powered.
- **Never power the servos from the ESP32.** Use the dedicated servo rail (5 V
  BEC or 2S LiPo per the wiring reference). Verify servo voltage rating against a
  fully charged 2S (8.4 V) before connecting.
- **LiPo handling:** charge/store/transport per standard LiPo safety; a 2S servo
  pack at 8.4 V can source high current into a fault.
- **Ground testing:** `GroundRollControlTest.py` uses **aggressive gains and the
  full ±15° deflection and bypasses altitude safety.** Bench-test with the
  airframe restrained and clear of people; it is not a flight controller.
- **Verify pins and addresses before flight:** the checked-in firmware and the
  current harness disagree on I²C pins, actuator wiring, and possibly I²C
  addresses ([§2.5](#25-documentation-vs-firmware-discrepancies)). Confirm the
  MPU6050 and BMP585 with an I²C scanner on GPIO23/32 and sweep both servos on
  GPIO26/25 before installing the bay.

---

## 7. Status / Known Limitations

Grounded in the current code and comments:

- **Software is unmerged and split across branches.** `main` has no firmware;
  `Controls`, `Raspi`, and `Meshtastic-Programs` are not integrated with each
  other.
- **Roll only.** Despite the "Tilt & Roll" name and the 4-fin/canard CAD, only
  **2-canard roll control** is implemented. There is no thrust-vectoring or
  independent 4-fin control in firmware.
- **No flight validation in the repo.** The controllers are labeled "basic" and
  "ground test"; session logs exist under `logs/`, but there is **no evidence of
  a validated powered flight**, and no test report characterizing performance.
- **Checked-in firmware lags the current harness.** The harness now uses I²C on
  GPIO23/32, direct servo PWM on GPIO26/25 (no PCA9685), and a wired ESP32↔Pi
  UART (GPIO27/33 ↔ Pi GPIO14/15) — the `Controls`-branch sketch still
  initializes I²C on 21/22 and drives a PCA9685
  ([§2.5](#25-documentation-vs-firmware-discrepancies)). Update and re-verify
  before flight.
- **Pi 5 UART link is wired but its software path is unconfirmed.** The
  physical UART now exists (§2.2), but the checked-in bridge streams to a
  **laptop over USB**; ESP32-side and Pi-side software for the UART link must
  be confirmed or written.
- **LoRa telemetry sends placeholder data.** `HeltecV4TelemetryTransmitter.ino`'s
  `fillSensorData()` hardcodes example altitude/battery/temp/GPS values —
  *"Replace these example values with real sensor reads."* It is not yet wired to
  the sensor bridge.
- **Meshtastic scripts are experiments.** `Sender.py` transmits placeholder joke
  strings; `Reciever.py`'s `PORT` is unset (`"xxxx"`).
- **Uncalibrated actuator trims.** `CANARD1_TRIM` / `CANARD2_TRIM` are `0.0`
  with a "tune until neutral" comment.
- **Gyro-unit assumptions.** Comments disagree on whether gyro is logged in
  rad/s or deg/s; the controllers default to `rad/s` and assume `gyro_z` is the
  roll axis for the current wiring. Confirm on hardware.
- **No build system / no schematic image.** Arduino sketches + loose Python
  scripts; no Makefile/PlatformIO; no wiring diagram or schematic file.
