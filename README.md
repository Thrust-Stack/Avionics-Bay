# Avionics-Bay

# Wiring Reference

Complete pin assignments for the avionics bay.

---

## System Overview

```
GPS V3 ──────(UART2 GPIO 16/17)──┐
BMP585 ──────(I2C  GPIO 23/32)───┤
MPU9250/6500 (I2C  GPIO 23/32)───┼──► ESP32 ──(SPI 13/14/27/33)──► microSD (data + actuation log)
                                  │
                                  ├──(PWM GPIO 26)──► Canard 1 servo
                                  ├──(PWM GPIO 25)──► Canard 2 servo
                                  └──(UART1 GPIO 19)──► Heltec (U0 GPIO 44)

Power: 2S LiPo ──► buck step-down ──► ESP32 5V/VIN (+ servo & SD 5V rail)
```

> **Direct-mode architecture:** the ESP32 remains responsible for its sensors,
> canard actuation, and SD logging. The ESP32D sends telemetry directly to the
> Heltec over GPIO 19; the Raspberry Pi 5 is no longer in this comm path. GPIO
> 27/33 remain assigned to the microSD SPI bus.

---

## ESP32 Pin Assignments

| GPIO | Function       | Connected To              | Wire Color |
|------|----------------|---------------------------|------------|
| 16   | UART2 RX       | GPS TX                    | Green      |
| 17   | UART2 TX       | GPS RX                    | Blue       |
| 18   | Reserved       | Not connected for Heltec one-way telemetry | — |
| 19   | UART1 TX       | Heltec GPIO 44 (U0RXD)    | —          |
| 23   | I2C SDA        | BMP585 SDA + IMU SDA      | Yellow     |
| 32   | I2C SCL        | BMP585 SCL + IMU SCL      | Orange     |
| 13   | SPI MOSI (SD)  | ADA254 DI                 | —          |
| 14   | SPI SCK (SD)   | ADA254 CLK                | —          |
| 25   | PWM (Canard 2) | Canard 2 servo signal     | White      |
| 26   | PWM (Canard 1) | Canard 1 servo signal     | White      |
| 27   | SPI MISO (SD)  | ADA254 DO                 | —          |
| 33   | SPI CS (SD)    | ADA254 CS                 | —          |
| 3.3V | Power out      | GPS VIN, BMP585 VIN, IMU VCC | Red     |
| GND  | Common ground  | All components + BEC GND  | Black      |

---

## Adafruit Ultimate GPS V3

| GPS Pin | ESP32 Pin | Notes                        |
|---------|-----------|------------------------------|
| VIN     | 3.3V      | Do NOT use 5V                |
| GND     | GND       | Common ground                |
| TX      | GPIO 16   | GPS transmits → ESP32 reads  |
| RX      | GPIO 17   | ESP32 sends commands to GPS  |

> Baud rate: 9600 (default). UART2 on ESP32.

---

## BMP585 (Altimeter)

| BMP585 Pin | ESP32 Pin | Notes                         |
|------------|-----------|-------------------------------|
| VIN        | 3.3V      |                               |
| GND        | GND       | Common ground                 |
| SDA        | GPIO 23   | Shared I2C bus with IMU       |
| SCL        | GPIO 32   | Shared I2C bus with IMU       |

> I2C address: 0x46. Shares bus with the IMU.

---

## MPU9250 / MPU6500 (IMU)

Replaces the original MPU6050. Requires the **FastIMU** library (the
`Adafruit_MPU6050` library rejects a 9250/6500 on its WHO-AM-I check).

| IMU Pin | ESP32 Pin | Notes                                        |
|---------|-----------|----------------------------------------------|
| VCC     | 3.3V      |                                              |
| GND     | GND       | Common ground                                |
| SDA     | GPIO 23   | Shared I2C bus with BMP585                   |
| SCL     | GPIO 32   | Shared I2C bus with BMP585                   |
| EDA     | —         | Aux I2C master bus — leave unconnected       |
| ECL     | —         | Aux I2C master bus — leave unconnected       |
| AD0     | GND       | Sets I2C address to 0x68 (tie to 3.3V → 0x69)|
| NCS     | 3.3V      | Forces I2C mode — do NOT leave floating      |
| FSYNC   | GND       | Tie low — do NOT leave floating              |

> I2C address: 0x68 (AD0 → GND). Shares bus with BMP585.
> WHO_AM_I: 0x71 (MPU9250) or 0x70 (MPU6500). If `imu.init()` returns a
> non-zero error, switch `MPU9250 imu;` to `MPU6500 imu;` in the sketch.
> The sketch converts FastIMU's g / deg-per-second output to m/s² and rad/s,
> so the `$IMU` packet format remains unchanged.
> The installed orientation uses **MPU X** as both the rocket roll axis and
> vertical-acceleration axis. `$IMU` packets retain raw MPU axis order, so
> control code uses `gyroX`/`gx` for roll rate and `accelX`/`ax` for vertical
> acceleration.

---

## Adafruit ADA254 MicroSD Breakout (SPI)

Logs every sensor sample and canard actuation angle to a CSV on the card.
Runs on the ESP32 HSPI bus on GPIO 13/14/27/33. This is separate from the I2C
sensors and the direct Heltec UART. The ADA254 is level-shifted and has an
onboard regulator, so use its `5V` input from the avionics 5V rail.

| ADA254 Pin | ESP32 Pin | Notes                                      |
|------------|-----------|--------------------------------------------|
| 5V         | 5V        | Regulator input from the buck/5V rail      |
| GND        | GND       | Common ground                              |
| CS         | GPIO 33   | Chip select                                |
| CLK        | GPIO 14   | SPI clock / ESP32 SCK                      |
| DI         | GPIO 13   | Data into card / ESP32 MOSI                |
| DO         | GPIO 27   | Data out of card / ESP32 MISO              |
| CD         | —         | Optional card-detect pin; leave unconnected |

> Card must be formatted **FAT32**.
> GPIO 19 is reserved for the direct Heltec telemetry UART and is not used for SPI.
> Firmware init: `SPIClass sdSPI(HSPI); sdSPI.begin(14,27,13,33); SD.begin(33,sdSPI);`

---

## Canard Servos (×2)

| Servo Wire  | Connects To        | Notes                              |
|-------------|--------------------|------------------------------------|
| Red (VCC)   | BEC / buck 5V      | Both canards share the 5V rail     |
| Brown (GND) | Common GND         | Shared with ESP32 and BEC          |
| Orange (SIG)| ESP32 GPIO (below) | PWM signal at 50Hz                 |

| Servo    | Signal Pin | Notes         |
|----------|------------|---------------|
| Canard 1 | GPIO 26    | Roll fin      |
| Canard 2 | GPIO 25    | Roll fin (same signed deflection as Canard 1) |

> PWM: 50Hz, pulse range 500–2400μs. The firmware treats
> `STARTING_CANARD_POSITION_DEG` as neutral, with optional
> `STARTING_CANARD1_POSITION_DEG` / `STARTING_CANARD2_POSITION_DEG` per-servo
> overrides, then applies signed fin commands relative to those
> startup/calibrated positions through `angleToPWM16()`.
> ⚠️ Never power servos from the ESP32 — use the dedicated 5V BEC/buck.

---

## 5V BEC (Servo Power)

| BEC Terminal | Connects To                        |
|--------------|------------------------------------|
| +5V output   | All 4 servo red (VCC) wires        |
| GND output   | Common ground rail (ESP32 + servos)|
| Input +      | Battery / main power bus           |
| Input −      | Battery negative                   |

> Minimum BEC rating: 5A continuous. 8–10A recommended for headroom.

---

## Direct ESP32D -> Heltec wiring

This is a one-way 3.3 V serial telemetry link from the ESP32D to the avionics
Heltec. The Raspberry Pi 5 is no longer required for this communication path,
and the avionics Heltec only sends packets to the ground station.

| ESP32D connection | Heltec connection    | Direction                         |
|-------------------|----------------------|-----------------------------------|
| GPIO 19 (TX)      | GPIO 44 (U0RXD)      | ESP32D telemetry → Heltec         |
| GND               | GND                  | Common ground — required          |

> UART settings on both ends: **115200 baud, 8 data bits, no parity, 1 stop
> bit** (`SERIAL_8N1`).
>
> ESP32D -> Heltec packets remain newline-delimited GPS NMEA, `$IMU,...`, and
> `$ALT,...` lines. There is no Heltec -> ESP32D command path in this setup.
>
> This pinout assumes GPIO 19 is operational on the selected ESP32D.
>
> The avionics-side firmware is
> `Communication/TransmitterHeltec/TransmitterHeltec.ino`.
> Build it with **USB CDC On Boot: Disabled** so `Serial` is hardware UART0 on
> GPIO 44. UART0 is protocol-only in this mode; do not send debug text on it.
> Its LoRa side uses a bounded telemetry queue; when RF airtime is slower than
> the UART stream, it retains newer complete packets instead of allowing the
> queue to grow without limit.
> If UART0 wiring interferes with a Heltec upload, disconnect ESP32D TX or hold the
> ESP32D quiet while flashing, then reconnect the direct link.

---

## Power Summary

| Component     | Voltage | Source              |
|---------------|---------|---------------------|
| ESP32         | 5V      | 2S LiPo → buck step-down → 5V/VIN |
| GPS V3        | 3.3V    | ESP32 3.3V pin      |
| BMP585        | 3.3V    | ESP32 3.3V pin      |
| MPU9250/6500  | 3.3V    | ESP32 3.3V pin      |
| ADA254 microSD| 5V      | 5V rail (buck)      |

---

## Ground station: dual raw + CSV logging

The `Communication/receiver_listener.py` script now supports writing both a
raw text log (the legacy `telemetry_log.txt`) and a structured CSV file
(`telemetry_log.csv` by default). Use `--csv <path>` to set the CSV path and
`--no-raw` to disable the raw text log when only structured rows are desired.
Each CSV row contains: `timestamp`, `type`, `raw`, and `parsed` (JSON) columns.

This makes automated processing, spreadsheet analysis, and compact terminal
summaries much easier while still preserving full raw telemetry for debugging.

## Binary LoRa telemetry format (Heltec V4)

The avionics transmitter now packs a compact fixed-size binary frame before
sending over LoRa. The ground receiver decodes and prints a `BIN,...` ASCII
line over USB for the laptop listener. The frame layout (big-endian) is:

- `version` (1 byte)
- `type` (1 byte)
- `seq` (2 bytes)
- `timestamp` (4 bytes)
- `lat` (4 bytes, int32, degrees * 1e7)
- `lon` (4 bytes, int32, degrees * 1e7)
- `alt_mm` (4 bytes, int32, millimeters)
- `speed_cms` (2 bytes, uint16, cm/s)
- `heading_cd` (2 bytes, uint16, centi-degrees)
- `pitch_cd` (2 bytes, int16, centi-degrees)
- `roll_cd` (2 bytes, int16, centi-degrees)
- `yaw_cd` (2 bytes, int16, centi-degrees)
- `battery_mv` (2 bytes, uint16)
- `status` (1 byte)
- `crc16` (2 bytes)

Why this helps:
- The binary frame is far smaller than repeating verbose NMEA and ASCII fields,
    reducing airtime and increasing effective range for the same RF settings.
- Fixed-size numeric fields encode values compactly and avoid the cost of ASCII
    decimal digits. Using a small application CRC16 provides end-to-end payload
    integrity even if radio CRC is enabled.

The Arduino sketches in `Communication/TransmitterHeltec/TransmitterHeltec.ino`
and `Communication/ReceiverHeltec/ReceiverHeltec.ino` implement this packing and
decoding.
| 2× Canards    | 5V      | Dedicated 5V BEC / buck |

---

## I2C Address Reference

| Device       | I2C Address | AD0/SDO Pin |
|--------------|-------------|-------------|
| BMP585       | 0x46        | SDO floating/GND (default) |
| MPU9250/6500 | 0x68        | AD0 → GND   |

> Both devices share GPIO 23 (SDA) and GPIO 32 (SCL).  
> Addresses are unique so no conflicts on the shared bus.

---

## Notes

- All grounds must be tied together: ESP32D GND, Heltec GND, BEC GND, and all servo browns.
- Keep servo signal wires away from power wires to reduce noise on PWM lines.
- The GPS needs clear sky view — route the antenna cable to the exterior of the airframe.
- BMP585 is I2C only (no SPI mode on this breakout).
