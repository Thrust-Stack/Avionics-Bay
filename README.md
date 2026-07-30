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
                                  └──(UART1 GPIO 19/18)◄──► Heltec (U0 GPIO 44/43)

Power: 2S LiPo ──► buck step-down ──► ESP32 5V/VIN (+ servo & SD 5V rail)
```

> **Direct-mode architecture:** the ESP32 remains responsible for its sensors,
> canard actuation, and SD logging. The ESP32D and Heltec communicate directly
> over GPIO 19/18; the Raspberry Pi 5 is no longer in this comm path. GPIO
> 27/33 remain assigned to the microSD SPI bus.

---

## ESP32 Pin Assignments

| GPIO | Function       | Connected To              | Wire Color |
|------|----------------|---------------------------|------------|
| 16   | UART2 RX       | GPS TX                    | Green      |
| 17   | UART2 TX       | GPS RX                    | Blue       |
| 18   | UART1 RX       | Heltec GPIO 43 (U0TXD)    | —          |
| 19   | UART1 TX       | Heltec GPIO 44 (U0RXD)    | —          |
| 23   | I2C SDA        | BMP585 SDA + IMU SDA      | Yellow     |
| 32   | I2C SCL        | BMP585 SCL + IMU SCL      | Orange     |
| 13   | SPI MOSI (SD)  | microSD MOSI (DI)         | —          |
| 14   | SPI SCK (SD)   | microSD SCK (CLK)         | —          |
| 25   | PWM (Canard 2) | Canard 2 servo signal     | White      |
| 26   | PWM (Canard 1) | Canard 1 servo signal     | White      |
| 27   | SPI MISO (SD)  | microSD MISO (DO)         | —          |
| 33   | SPI CS (SD)    | microSD CS (SS)           | —          |
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

## MicroSD Card Module (SPI)

Logs every sensor sample and canard actuation angle to a CSV on the card.
Runs on the ESP32 HSPI bus on GPIO 13/14/27/33. This is separate from the I2C
sensors and the direct Heltec UART.

| SD Module Pin | ESP32 Pin | Notes                                            |
|---------------|-----------|--------------------------------------------------|
| VCC           | 5V        | Level-shifted modules (8-pin LVC125) need 5V.    |
|               |           | Bare 3.3V-only modules: use 3V3 instead.         |
| GND           | GND       | Common ground                                    |
| CS  (SS)      | GPIO 33   | Chip select                                      |
| SCK (CLK)     | GPIO 14   | SPI clock                                        |
| MOSI (DI)     | GPIO 13   | Data to card                                     |
| MISO (DO)     | GPIO 27   | Data from card                                   |

> Card must be formatted **FAT32**.
> GPIO 18/19 are reserved for the direct Heltec UART and are not used for SPI.
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
| Canard 2 | GPIO 25    | Roll fin (differential — opposite of Canard 1) |

> PWM: 50Hz, pulse range 500–2400μs, neutral ≈1450μs (90°) — matches the
> firmware's `angleToPWM16()`.
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

## Direct ESP32D ↔ Heltec wiring

This is a direct 3.3 V serial link between the ESP32D and the avionics Heltec.
The Raspberry Pi 5 is no longer required for this communication path.

| ESP32D connection | Heltec connection    | Direction                         |
|-------------------|----------------------|-----------------------------------|
| GPIO 19 (TX)      | GPIO 44 (U0RXD)      | ESP32D telemetry → Heltec         |
| GPIO 18 (RX)      | GPIO 43 (U0TXD)      | Heltec commands → ESP32D          |
| GND               | GND                  | Common ground — required          |

> UART settings on both ends: **115200 baud, 8 data bits, no parity, 1 stop
> bit** (`SERIAL_8N1`).
>
> ESP32D → Heltec packets remain newline-delimited GPS NMEA, `$IMU,...`, and
> `$ALT,...` lines. Heltec → ESP32D commands remain `ROLL,<angle>\n`.
>
> This pinout assumes GPIO 18/19 are operational on the selected ESP32D.
>
> The avionics-side firmware is
> `Communication/HeltecV4TelemetryTransmitter/HeltecV4TelemetryTransmitter.ino`.
> Build it with **USB CDC On Boot: Disabled** so `Serial` is hardware UART0 on
> GPIO 44/43. UART0 is protocol-only in this mode; do not send debug text on it.
> Its LoRa side uses a bounded queue and explicit command-receive windows; when
> RF airtime is slower than the UART stream, it retains newer complete packets
> instead of allowing the queue to grow without limit.
> If UART0 wiring interferes with a Heltec upload, disconnect TX/RX or hold the
> ESP32D quiet while flashing, then reconnect the direct link.

---

## Power Summary

| Component     | Voltage | Source              |
|---------------|---------|---------------------|
| ESP32         | 5V      | 2S LiPo → buck step-down → 5V/VIN |
| GPS V3        | 3.3V    | ESP32 3.3V pin      |
| BMP585        | 3.3V    | ESP32 3.3V pin      |
| MPU9250/6500  | 3.3V    | ESP32 3.3V pin      |
| microSD module| 5V      | 5V rail (buck)      |
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
