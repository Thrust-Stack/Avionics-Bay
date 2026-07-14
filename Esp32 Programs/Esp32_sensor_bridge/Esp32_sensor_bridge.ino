/*
 * Avionics Sensor Bridge + Roll Control â€” ESP32
 *
 * Reads GPS on UART2, MPU6050 and BMP585 on I2C,
 * forwards all sensor data to Raspberry Pi over UART1 (GPIO 33 TX / GPIO 27 RX).
 * Receives ROLL,<angle> commands from the Pi on the same UART1.
 * USB Serial (Serial) is debug-only for Arduino IDE Serial Monitor.
 * actuates two canard servos via ESP32 LEDC PWM on GPIO 26/25.
 *
 * Wiring:
 *   GPS VIN      -> ESP32 3.3V
 *   GPS GND      -> ESP32 GND
 *   GPS TX       -> ESP32 GPIO 16
 *   GPS RX       -> ESP32 GPIO 17
 *   MPU SDA      -> ESP32 GPIO 23  (shared I2C bus)
 *   MPU SCL      -> ESP32 GPIO 32  (shared I2C bus)
 *   BMP SDA      -> ESP32 GPIO 23  (shared I2C bus)
 *   BMP SCL      -> ESP32 GPIO 32  (shared I2C bus)
 *   NOTE: GPIO 18/19/21/22 are damaged on this board (verified by driving
 *   them HIGH with no load: they cannot reach 3.3V). Do not use them.
 *   Canard 1     -> ESP32 GPIO 26
 *   Canard 2     -> ESP32 GPIO 25
 *   Pi GPIO14    -> ESP32 GPIO 27  (Pi TX â†’ ESP32 RX)
 *   Pi GPIO15    -> ESP32 GPIO 33  (Pi RX â†’ ESP32 TX)
 *   Pi GND       -> ESP32 GND      (common ground â€” required)
 *
 * Install libraries in Arduino IDE (Tools -> Manage Libraries):
 *   - Adafruit MPU6050
 *   - Adafruit BMP5xx
 *   - Adafruit Unified Sensor
 *
 * Board: NodeMCU-32S
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

// â”€â”€ GPS config â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#define GPS_RX_PIN  16
#define GPS_TX_PIN  17
#define GPS_BAUD    9600

// â”€â”€ BMP585 config â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#define BMP585_ADDR BMP5XX_DEFAULT_ADDRESS  // 0x46

// â”€â”€ IMU config â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
Adafruit_MPU6050 mpu;
bool  mpuReady           = false;
unsigned long lastIMU    = 0;
const unsigned long IMU_INTERVAL = 50;  // 50ms = 20Hz

// â”€â”€ BMP585 config â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
Adafruit_BMP5xx bmp585;
float groundPressure     = 1013.25f;
bool  bmpReady           = false;
unsigned long lastALT    = 0;
const unsigned long ALT_INTERVAL = 100;  // 100ms = 10Hz

// â”€â”€ Servo config (direct LEDC PWM) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#define CANARD1_PIN     26
#define CANARD2_PIN     25
#define SERVO_FREQ_HZ   50
#define SERVO_MIN_US    500    // pulse width for 0Â°
#define SERVO_MAX_US    2400   // pulse width for 180Â°
#define NEUTRAL_ANGLE   90.0f  // resting angle when no command
#define MAX_DEFLECTION  15.0f  // matches GroundRollControlTest.py clamp

// â”€â”€ Pi UART (Serial1 on GPIO 33 TX / 27 RX) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
#define PI_RX_PIN  27
#define PI_TX_PIN  33
#define PI_BAUD    115200

// â”€â”€ I2C bus (SDA GPIO 23 / SCL GPIO 32) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// GPIO 18/19/21/22 are damaged on this board â€” do not use.
#define I2C_SDA_PIN  23
#define I2C_SCL_PIN  32

// Set to 1 to mirror the $IMU/$ALT stream to the USB Serial Monitor
// for bench debugging; set to 0 for flight (keeps USB quiet).
#define DEBUG_USB  1

// â”€â”€ Buffers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
String gpsBuffer = "";   // GPS NMEA line assembly
String cmdBuffer = "";   // incoming Pi command line assembly


// â”€â”€ Helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

// Convert servo angle (0â€“180Â°) to LEDC 16-bit duty count
uint32_t angleToPWM16(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)(us / 20000.0f * 65535.0f);
}

// Apply a differential fin deflection to both canards
// Positive fin_command deflects canard 1 up and canard 2 down (and vice versa)
void setCanards(float fin_command) {
  fin_command = constrain(fin_command, -MAX_DEFLECTION, MAX_DEFLECTION);
  float angle1 = NEUTRAL_ANGLE + fin_command;
  float angle2 = NEUTRAL_ANGLE - fin_command;
  ledcWrite(CANARD1_PIN, angleToPWM16(angle1));
  ledcWrite(CANARD2_PIN, angleToPWM16(angle2));
  Serial.printf("[ROLL] cmd=%.2f  canard1=%.1fÂ°  canard2=%.1fÂ°\n",
                fin_command, angle1, angle2);
}

// Parse and dispatch a complete command line from the laptop
void processCommand(const String& line) {
  if (line.startsWith("ROLL,")) {
    float cmd = line.substring(5).toFloat();
    setCanards(cmd);
  }
}


// â”€â”€ Setup â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void setup() {
  Serial.begin(115200);                                          // USB debug only
  Serial1.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);    // Pi comms

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // SDA=23, SCL=32 â€” GPIO 18/19/21/22 are damaged on this board

  Serial.println("==========================================");
  Serial.println("  Avionics Sensor Bridge + Roll Control");
  Serial.println("==========================================");

  // MPU6050
  if (!mpu.begin(0x69)) {  // AD0 is high on this breakout, so not the default 0x68
    Serial.println("[ERROR] MPU6050 not found! Check wiring on GPIO 23 (SDA) / 32 (SCL).");
  } else {
    mpuReady = true;
    Serial.println("[OK] MPU6050 initialized");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // BMP585
  if (!bmp585.begin((uint8_t)BMP585_ADDR)) {
    Serial.println("[ERROR] BMP585 not found! Check wiring on GPIO 23 (SDA) / 32 (SCL), address 0x46.");
  } else {
    Serial.println("[OK] BMP585 initialized");
    bmp585.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
    bmp585.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
    bmp585.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

    Serial.println("[BMP] Calibrating ground pressure...");
    delay(500);
    float pressureSum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 20; i++) {
      if (bmp585.performReading()) {
        pressureSum += bmp585.pressure;
        samples++;
      }
      delay(100);
    }
    if (samples > 0) {
      groundPressure = pressureSum / samples;
      bmpReady = true;
      Serial.print("[BMP] Ground pressure: ");
      Serial.print(groundPressure, 2);
      Serial.println(" hPa  (AGL zeroed)");
    } else {
      Serial.println("[ERROR] BMP585 calibration failed -- no readings received");
    }
  }

  // Canard servos â€” LEDC PWM on GPIO 26 (canard 1) and GPIO 25 (canard 2)
  ledcAttach(CANARD1_PIN, SERVO_FREQ_HZ, 16);
  ledcAttach(CANARD2_PIN, SERVO_FREQ_HZ, 16);
  ledcWrite(CANARD1_PIN, angleToPWM16(NEUTRAL_ANGLE));
  ledcWrite(CANARD2_PIN, angleToPWM16(NEUTRAL_ANGLE));
  Serial.println("[OK] Servos initialized â€” canards at neutral");

  Serial.println("[OK] GPS UART initialized");
  Serial.println("==========================================");
  Serial.println();
}


// â”€â”€ Main loop â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void loop() {
  // Forward GPS NMEA sentences to Pi
  while (Serial2.available()) {
    char c = Serial2.read();
    gpsBuffer += c;
    if (c == '\n') {
      Serial1.print(gpsBuffer);
      gpsBuffer = "";
    }
  }

  // IMU at 20Hz
  if (mpuReady && (millis() - lastIMU >= IMU_INTERVAL)) {
    lastIMU = millis();
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    char imuLine[96];
    snprintf(imuLine, sizeof(imuLine), "$IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
             gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    Serial1.print(imuLine);
#if DEBUG_USB
    Serial.print(imuLine);
#endif
  }

  // BMP585 altitude at 10Hz
  if (bmpReady && (millis() - lastALT >= ALT_INTERVAL)) {
    lastALT = millis();
    if (bmp585.performReading()) {
      float agl = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
      char altLine[32];
      snprintf(altLine, sizeof(altLine), "$ALT,%.2f\n", agl);
      Serial1.print(altLine);
#if DEBUG_USB
      Serial.print(altLine);
#endif
    }
  }

  // Parse ROLL commands from Pi (line-buffered)
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) {
        processCommand(cmdBuffer);
      }
      cmdBuffer = "";
    } else {
      cmdBuffer += c;
    }
  }
}