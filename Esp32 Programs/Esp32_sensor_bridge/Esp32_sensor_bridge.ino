/*
 * Avionics Sensor Bridge + Roll Control -- ESP32
 *
 * Reads GPS on UART2, MPU9250/6500 and BMP585 on I2C,
 * forwards all sensor data directly to the Heltec over UART1
 * (GPIO 19 TX / GPIO 18 RX).
 * Receives ROLL,<angle> commands from the Heltec on the same UART1.
 * USB Serial (Serial) is debug-only for Arduino IDE Serial Monitor.
 * Actuates two canard servos via ESP32 LEDC PWM on GPIO 26/25.
 *
 * Wiring:
 *   GPS VIN      -> ESP32 3.3V
 *   GPS GND      -> ESP32 GND
 *   GPS TX       -> ESP32 GPIO 16
 *   GPS RX       -> ESP32 GPIO 17
 *   IMU VCC      -> ESP32 3.3V
 *   IMU GND      -> ESP32 GND
 *   IMU SDA      -> ESP32 GPIO 23  (shared I2C bus)
 *   IMU SCL      -> ESP32 GPIO 32  (shared I2C bus)
 *   IMU EDA/ECL  -> NOT CONNECTED  (auxiliary I2C master bus; leave floating)
 *   IMU AD0      -> ESP32 GND      (sets I2C address to 0x68)
 *   IMU NCS      -> ESP32 3.3V     (forces I2C mode; do not leave floating)
 *   IMU FSYNC    -> ESP32 GND      (tie low; do not leave floating)
 *   BMP SDA      -> ESP32 GPIO 23  (shared I2C bus)
 *   BMP SCL      -> ESP32 GPIO 32  (shared I2C bus)
 *   Canard 1     -> ESP32 GPIO 26
 *   Canard 2     -> ESP32 GPIO 25
 *   Heltec GPIO44 (U0RXD) <- ESP32 GPIO 19  (ESP32 TX -> Heltec RX)
 *   Heltec GPIO43 (U0TXD) -> ESP32 GPIO 18  (Heltec TX -> ESP32 RX)
 *   Heltec GND             -> ESP32 GND      (common ground -- required)
 *
 * Install libraries in Arduino IDE (Tools -> Manage Libraries):
 *   - FastIMU            (supports MPU9250 AND MPU6500 clones)
 *   - Adafruit BMP5xx
 *   - Adafruit Unified Sensor
 *
 * Board: NodeMCU-32S
 */

#include <Wire.h>
#include <FastIMU.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

// -- GPS config ---------------------------------------------------
#define GPS_RX_PIN  16
#define GPS_TX_PIN  17
#define GPS_BAUD    9600

// -- BMP585 config ------------------------------------------------
#define BMP585_ADDR BMP5XX_DEFAULT_ADDRESS  // 0x46

// -- IMU config (MPU9250 / MPU6500) -------------------------------
// AD0 -> GND gives address 0x68. Tie AD0 -> 3.3V for 0x69 if 0x68 conflicts.
#define IMU_ADDRESS  0x68
// Your board is labeled "9250/6500". If IMU.init() below returns a non-zero
// error, it is an MPU6500 die (no magnetometer) -- change the type to MPU6500.
MPU9250 imu;                    // <-- change to  MPU6500 imu;  if init fails
calData imuCalib = { 0 };       // zeroed: raw output, matches old MPU6050 behavior
AccelData accelData;
GyroData  gyroData;
bool  mpuReady           = false;
unsigned long lastIMU    = 0;
const unsigned long IMU_INTERVAL = 50;  // 50ms = 20Hz

// -- BMP585 config ------------------------------------------------
Adafruit_BMP5xx bmp585;
float groundPressure     = 1013.25f;
bool  bmpReady           = false;
unsigned long lastALT    = 0;
const unsigned long ALT_INTERVAL = 100;  // 100ms = 10Hz

// -- Servo config (direct LEDC PWM) -------------------------------
#define CANARD1_PIN     26
#define CANARD2_PIN     25
#define SERVO_FREQ_HZ   50
#define SERVO_MIN_US    500    // pulse width for 0 deg
#define SERVO_MAX_US    2400   // pulse width for 180 deg
#define NEUTRAL_ANGLE   90.0f  // resting angle when no command
#define MAX_DEFLECTION  15.0f  // matches GroundRollControlTest.py clamp

// -- Heltec UART (Serial1 on GPIO 19 TX / 18 RX) ------------------
#define HELTEC_RX_PIN  18
#define HELTEC_TX_PIN  19
#define HELTEC_BAUD    115200

// -- I2C bus (SDA GPIO 23 / SCL GPIO 32) --------------------------
#define I2C_SDA_PIN  23
#define I2C_SCL_PIN  32

// Unit conversions to keep the $IMU stream identical to the old MPU6050
// output: accel in m/s^2, gyro in rad/s (FastIMU reports g and deg/s).
#define G_TO_MS2      9.80665f
#define DEGS_TO_RADS  0.01745329252f

// Set to 1 to mirror the $IMU/$ALT stream to the USB Serial Monitor
// for bench debugging; set to 0 for flight (keeps USB quiet).
#define DEBUG_USB  1

// -- Buffers ------------------------------------------------------
String gpsBuffer = "";   // GPS NMEA line assembly
String cmdBuffer = "";   // incoming Heltec command line assembly


// -- Helpers ------------------------------------------------------

// Convert servo angle (0-180 deg) to LEDC 16-bit duty count
uint32_t angleToPWM16(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)(us / 20000.0f * 65535.0f);
}

// Apply the same signed roll deflection to both canards.
void setCanards(float fin_command) {
  fin_command = constrain(fin_command, -MAX_DEFLECTION, MAX_DEFLECTION);
  float angle1 = NEUTRAL_ANGLE + fin_command;
  float angle2 = NEUTRAL_ANGLE + fin_command;
  ledcWrite(CANARD1_PIN, angleToPWM16(angle1));
  ledcWrite(CANARD2_PIN, angleToPWM16(angle2));
  Serial.printf("[ROLL] cmd=%.2f  canard1=%.1f  canard2=%.1f\n",
                fin_command, angle1, angle2);
}

// Parse and dispatch a complete command line from the Heltec
void processCommand(const String& line) {
  if (line.startsWith("ROLL,")) {
    float cmd = line.substring(5).toFloat();
    setCanards(cmd);
  }
}


// -- Setup --------------------------------------------------------
void setup() {
  Serial.begin(115200);                                                  // USB debug only
  Serial1.begin(HELTEC_BAUD, SERIAL_8N1, HELTEC_RX_PIN, HELTEC_TX_PIN);  // Heltec comms

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // SDA=23, SCL=32
  Wire.setClock(400000);

  Serial.println("==========================================");
  Serial.println("  Avionics Sensor Bridge + Roll Control");
  Serial.println("==========================================");

  // MPU9250 / MPU6500
  int imuErr = imu.init(imuCalib, IMU_ADDRESS);
  if (imuErr != 0) {
    Serial.print("[ERROR] IMU init failed (code ");
    Serial.print(imuErr);
    Serial.println("). Check wiring on GPIO 23 (SDA)/32 (SCL), AD0, NCS.");
    Serial.println("        If wiring is good, change 'MPU9250 imu;' to 'MPU6500 imu;'.");
  } else {
    mpuReady = true;
    Serial.println("[OK] IMU (MPU9250/6500) initialized");
    imu.setGyroRange(500);     // deg/s  (matches old MPU6050_RANGE_500_DEG)
    imu.setAccelRange(16);     // g      (matches old MPU6050_RANGE_16_G)
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

  // Canard servos -- LEDC PWM on GPIO 26 (canard 1) and GPIO 25 (canard 2)
  ledcAttach(CANARD1_PIN, SERVO_FREQ_HZ, 16);
  ledcAttach(CANARD2_PIN, SERVO_FREQ_HZ, 16);
  ledcWrite(CANARD1_PIN, angleToPWM16(NEUTRAL_ANGLE));
  ledcWrite(CANARD2_PIN, angleToPWM16(NEUTRAL_ANGLE));
  Serial.println("[OK] Servos initialized -- canards at neutral");

  Serial.println("[OK] GPS UART initialized");
  Serial.println("==========================================");
  Serial.println();
}


// -- Main loop ----------------------------------------------------
void loop() {
  // Forward GPS NMEA sentences to the Heltec
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
    imu.update();
    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);

    // Convert to the direct-link packet units: accel m/s^2, gyro rad/s.
    // MPU X is both the rocket's vertical-acceleration axis and roll axis:
    // downstream control uses ax for vertical acceleration and gx for roll rate.
    // The packet retains raw MPU axis order; re-verify the gx sign on the bench.
    float ax = accelData.accelX * G_TO_MS2;
    float ay = accelData.accelY * G_TO_MS2;
    float az = accelData.accelZ * G_TO_MS2;
    float gx = gyroData.gyroX * DEGS_TO_RADS;
    float gy = gyroData.gyroY * DEGS_TO_RADS;
    float gz = gyroData.gyroZ * DEGS_TO_RADS;

    char imuLine[96];
    snprintf(imuLine, sizeof(imuLine), "$IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
             ax, ay, az, gx, gy, gz);
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

  // Parse ROLL commands from the Heltec (line-buffered)
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
