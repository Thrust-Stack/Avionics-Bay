# Avionics Bay Wiring Reference

Complete pin assignments and wiring notes for the avionics bay system.

---

## System Overview

```text
BMP585 ──┐
          ├── I2C (GPIO 21/22) ──► ESP32 ──── UART1 (GPIO 2/4) ──► Raspberry Pi 5
MPU6050 ─┘                          │
                                     ├── UART2 (GPIO 16/17) ──► GPS
GPS V3 ─────────────────────────────┘
                                     ├── GPIO 13 ──► Servo 1
                                     ├── GPIO 14 ──► Servo 2
                                     ├── GPIO 25 ──► Servo 3
                                     └── GPIO 26 ──► Servo 4
                                                       ↑
                                               5V BEC dedicated power
```

---

## ESP32 Pin Assignments

| ESP32 Pin | Function      | Connected To                     | Wire Color |
| --------- | ------------- | -------------------------------- | ---------- |
| GPIO 16   | UART2 RX      | GPS TX                           | Green      |
| GPIO 17   | UART2 TX      | GPS RX                           | Blue       |
| GPIO 21   | I2C SDA       | BMP585 SDA + MPU6050 SDA         | Yellow     |
| GPIO 22   | I2C SCL       | BMP585 SCL + MPU6050 SCL         | Orange     |
| GPIO 13   | PWM           | Servo 1 signal                   | White      |
| GPIO 14   | PWM           | Servo 2 signal                   | White      |
| GPIO 25   | PWM           | Servo 3 signal                   | White      |
| GPIO 26   | PWM           | Servo 4 signal                   | White      |
| GPIO 4    | UART1 RX      | Pi 5 GPIO 14 TX                  | Purple     |
| GPIO 2    | UART1 TX      | Pi 5 GPIO 15 RX                  | Purple     |
| 3.3V      | Power output  | GPS VIN, BMP585 VIN, MPU6050 VCC | Red        |
| GND       | Common ground | All components + BEC GND         | Black      |

---

## Adafruit Ultimate GPS V3

| GPS Pin | ESP32 Pin | Notes                         |
| ------- | --------- | ----------------------------- |
| VIN     | 3.3V      | Do not use 5V                 |
| GND     | GND       | Common ground                 |
| TX      | GPIO 16   | GPS transmits, ESP32 receives |
| RX      | GPIO 17   | ESP32 sends commands to GPS   |

### Settings

* UART port: ESP32 UART2
* Baud rate: 9600 default

---

## BMP585 Altimeter

| BMP585 Pin | ESP32 Pin | Notes                       |
| ---------- | --------- | --------------------------- |
| VIN        | 3.3V      | Power input                 |
| GND        | GND       | Common ground               |
| SDA        | GPIO 21   | Shared I2C bus with MPU6050 |
| SCL        | GPIO 22   | Shared I2C bus with MPU6050 |

### Settings

* Interface: I2C
* Default I2C address: `0x47`
* Alternate I2C address: `0x46` if SDO is tied to GND

---

## MPU6050 IMU

| MPU6050 Pin | ESP32 Pin | Notes                      |
| ----------- | --------- | -------------------------- |
| VCC         | 3.3V      | Power input                |
| GND         | GND       | Common ground              |
| SDA         | GPIO 21   | Shared I2C bus with BMP585 |
| SCL         | GPIO 22   | Shared I2C bus with BMP585 |
| AD0         | GND       | Sets I2C address to `0x68` |

### Settings

* Interface: I2C
* I2C address: `0x68`

---

## BMS-127WV+ Servos

### Servo Power Wiring

| Servo Wire | Connects To   | Notes                                 |
| ---------- | ------------- | ------------------------------------- |
| Red VCC    | BEC 5V output | All 4 servos share the BEC power rail |
| Brown GND  | Common ground | Shared with ESP32 and BEC ground      |
| Orange SIG | ESP32 PWM pin | PWM signal at 250 Hz                  |

### Servo Signal Pins

| Servo   | PCA9685 Channel  | Notes          |
| ------- | ---------------- | -------------- |
| Servo 1 | CH 12            | Fin 1 / TVC X+ |
| Servo 2 | CH 13            | Fin 2 / TVC X- |
| Servo 3 | CH 14            | Fin 3 / TVC Y+ |
| Servo 4 | CH 15            | Fin 4 / TVC Y- |

### PWM Settings

* Frequency: 250 Hz
* Pulse range: 1000-2000 us
* Neutral pulse: 1520 us

> Warning: Never power the servos from the ESP32. Use a dedicated 5V BEC.

---

## 5V BEC Servo Power

| BEC Terminal | Connects To                                   |
| ------------ | --------------------------------------------- |
| +5V output   | All 4 servo red VCC wires                     |
| GND output   | Common ground rail for ESP32, BEC, and servos |
| Input +      | Battery or main power bus                     |
| Input -      | Battery negative                              |

### Recommended Rating

* Minimum: 5A continuous
* Recommended: 8-10A for extra headroom

---

## Raspberry Pi 5 to ESP32 UART

| Raspberry Pi 5 Pin | ESP32 Pin | Notes                         |
| ------------------ | --------- | ----------------------------- |
| GPIO 14 TX         | GPIO 4 RX | Pi sends commands to ESP32    |
| GPIO 15 RX         | GPIO 2 TX | ESP32 sends sensor data to Pi |
| GND                | GND       | Common ground                 |

### Settings

* Baud rate: 115200
* Logic level: 3.3V
* Level shifter: Not required because both Pi 5 and ESP32 use 3.3V UART logic

---

## Power Summary

| Component      | Voltage | Power Source          |
| -------------- | ------- | --------------------- |
| ESP32          | 5V      | USB or 5V regulator   |
| Raspberry Pi 5 | 5V      | USB-C PD power supply |
| GPS V3         | 3.3V    | ESP32 3.3V pin        |
| BMP585         | 3.3V    | ESP32 3.3V pin        |
| MPU6050        | 3.3V    | ESP32 3.3V pin        |
| 4x Servos      | 5V      | Dedicated 5V BEC      |

---

## I2C Address Reference

| Device  | I2C Address                                   | Address Select Pin |
| ------- | --------------------------------------------- | ------------------ |
| BMP585  | `0x47` default / `0x46` if SDO is tied to GND | SDO                |
| MPU6050 | `0x68`                                        | AD0 tied to GND    |

---

## PCA9685 Servo Driver Wiring

The PCA9685 is used to control the PWM signals for the servos. The ESP32 communicates with the PCA9685 over I2C, while the servos are powered by a separate 7.4V 2S LiPo battery.

---

## ESP32 to PCA9685 Connections

| ESP32 Pin | PCA9685 Pin | Purpose |
|---|---|---|
| 3V3 | VCC | Logic power for PCA9685 |
| GND | GND | Common ground |
| GPIO 21 | SDA | I2C data |
| GPIO 22 | SCL | I2C clock |
| GND | OE | Optional output enable |

> OE can be left disconnected on most PCA9685 boards, but connecting OE to GND keeps the PWM outputs enabled.

---

## Servo Battery to PCA9685 Connections

| Servo Battery | PCA9685 Pin | Purpose |
|---|---|---|
| 7.4V LiPo + | V+ | Servo power rail |
| 7.4V LiPo - | GND | Servo power ground |

The 2S LiPo battery is 7.4V nominal and 8.4V fully charged. Make sure the PCA9685 board and servos are rated for 8.4V max before connecting the battery.

---

## Servo Connections

Each servo connects to one PCA9685 channel.

| Servo | PCA9685 Channel |
|---|---|
| Servo 1 | Channel 0 |
| Servo 2 | Channel 1 |
| Servo 3 | Channel 2 |
| Servo 4 | Channel 3 |

Each servo has three wires:

| Servo Wire | PCA9685 Connection |
|---|---|
| Signal | PWM pin |
| Power | V+ pin |
| Ground | GND pin |

---

## Wiring Diagram

```text
ESP32 3V3 ───────────────→ PCA9685 VCC
ESP32 GND ───────────────→ PCA9685 GND
ESP32 GPIO21 SDA ────────→ PCA9685 SDA
ESP32 GPIO22 SCL ────────→ PCA9685 SCL
ESP32 GND ───────────────→ PCA9685 OE  optional

Servo Battery + 7.4V ────→ PCA9685 V+
Servo Battery - ─────────→ PCA9685 GND

PCA9685 Channel 0 ───────→ Servo 1
PCA9685 Channel 1 ───────→ Servo 2
PCA9685 Channel 2 ───────→ Servo 3
PCA9685 Channel 3 ───────→ Servo 4

---

Both sensors share the same I2C bus:

* SDA: GPIO 21
* SCL: GPIO 22

The addresses are unique, so there should be no conflict on the shared I2C bus.

---

## Important Notes

* All grounds must be tied together: ESP32 GND, BEC GND, Pi 5 GND, GPS GND, BMP585 GND, MPU6050 GND, and all servo ground wires.
* Keep servo power wires away from sensor and PWM signal wires when possible to reduce electrical noise.
* The GPS antenna needs a clear sky view, so route it toward the exterior of the airframe.
* The BMP585 should be used over I2C in this wiring setup.
* Power the servos only through the dedicated BEC, not through the ESP32.
* Confirm the BMP585 address with an I2C scanner before final flight testing.
