# Avionics-Bay

# Wiring Reference

Complete pin assignments for the avionics bay.

---

## System Overview

```
BMP585 ──┐
          ├── I2C (GPIO 21/22) ──► ESP32 ──── UART1 (GPIO 2/4) ──► Pi 5
MPU6050 ─┘                          │
                                     ├── UART2 (GPIO 16/17) ──► GPS
GPS V3 ─────────────────────────────┘
                                     ├── GPIO 13 ──► Servo 1
                                     ├── GPIO 14 ──► Servo 2
                                     ├── GPIO 25 ──► Servo 3
                                     └── GPIO 26 ──► Servo 4
                                                       ↑
                                               5V BEC (dedicated power)
```

---

## ESP32 Pin Assignments

| GPIO | Function       | Connected To              | Wire Color |
|------|----------------|---------------------------|------------|
| 16   | UART2 RX       | GPS TX                    | Green      |
| 17   | UART2 TX       | GPS RX                    | Blue       |
| 21   | I2C SDA        | BMP585 SDA + MPU6050 SDA  | Yellow     |
| 22   | I2C SCL        | BMP585 SCL + MPU6050 SCL  | Orange     |
| 13   | PWM (Servo 1)  | Servo 1 signal            | White      |
| 14   | PWM (Servo 2)  | Servo 2 signal            | White      |
| 25   | PWM (Servo 3)  | Servo 3 signal            | White      |
| 26   | PWM (Servo 4)  | Servo 4 signal            | White      |
| 4    | UART1 RX       | Pi 5 GPIO 14 (TX)         | Purple     |
| 2    | UART1 TX       | Pi 5 GPIO 15 (RX)         | Purple     |
| 3.3V | Power out      | GPS VIN, BMP585 VIN, MPU VCC | Red     |
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
| SDA        | GPIO 21   | Shared I2C bus with MPU6050   |
| SCL        | GPIO 22   | Shared I2C bus with MPU6050   |

> I2C address: 0x47. Shares bus with MPU6050.

---

## MPU6050 (IMU)

| MPU6050 Pin | ESP32 Pin | Notes                         |
|-------------|-----------|-------------------------------|
| VCC         | 3.3V      |                               |
| GND         | GND       | Common ground                 |
| SDA         | GPIO 21   | Shared I2C bus with BMP585    |
| SCL         | GPIO 22   | Shared I2C bus with BMP585    |
| AD0         | GND       | Sets I2C address to 0x68      |

> I2C address: 0x68 (AD0 → GND). Shares bus with BMP585.

---

## BMS-127WV+ Servos (×4)

| Servo Wire  | Connects To         | Notes                              |
|-------------|---------------------|------------------------------------|
| Red (VCC)   | BEC 5V output       | All 4 servos share BEC power rail  |
| Brown (GND) | Common GND          | Shared with ESP32 and BEC          |
| Orange (SIG)| ESP32 GPIO (below)  | PWM signal at 250Hz                |

| Servo   | Signal Pin | Notes          |
|---------|------------|----------------|
| Servo 1 | GPIO 13    | Fin 1 / TVC X+ |
| Servo 2 | GPIO 14    | Fin 2 / TVC X- |
| Servo 3 | GPIO 25    | Fin 3 / TVC Y+ |
| Servo 4 | GPIO 26    | Fin 4 / TVC Y- |

> PWM: 250Hz, pulse range 1000–2000μs, neutral 1520μs.  
> ⚠️ Never power servos from ESP32 — use a dedicated BEC.

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

## Raspberry Pi 5 ↔ ESP32

| Pi 5 Pin      | ESP32 Pin | Notes                          |
|---------------|-----------|--------------------------------|
| GPIO 14 (TX)  | GPIO 4    | Pi sends commands → ESP32      |
| GPIO 15 (RX)  | GPIO 2    | ESP32 sends sensor data → Pi   |
| GND           | GND       | Common ground                  |

> Both Pi 5 and ESP32 use 3.3V logic — direct connection is safe, no level shifter needed.  
> Baud rate: 115200.

---

## Power Summary

| Component     | Voltage | Source              |
|---------------|---------|---------------------|
| ESP32         | 5V      | USB or regulator    |
| Raspberry Pi 5| 5V      | USB-C PD (5A)       |
| GPS V3        | 3.3V    | ESP32 3.3V pin      |
| BMP585        | 3.3V    | ESP32 3.3V pin      |
| MPU6050       | 3.3V    | ESP32 3.3V pin      |
| 4× Servos     | 5V      | Dedicated 5V BEC    |

---

## I2C Address Reference

| Device  | I2C Address | AD0/SDO Pin |
|---------|-------------|-------------|
| BMP585  | 0x47        | Default     |
| MPU6050 | 0x68        | AD0 → GND   |

> Both devices share GPIO 21 (SDA) and GPIO 22 (SCL).  
> Addresses are unique so no conflicts on the shared bus.

---

## Notes

- All grounds must be tied together: ESP32 GND, BEC GND, Pi 5 GND, and all servo browns.
- Keep servo signal wires away from power wires to reduce noise on PWM lines.
- The GPS needs clear sky view — route the antenna cable to the exterior of the airframe.
- BMP585 is I2C only (no SPI mode on this breakout).