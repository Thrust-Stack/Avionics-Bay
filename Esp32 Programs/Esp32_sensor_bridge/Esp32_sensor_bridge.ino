/*
 * Avionics Sensor Bridge + Roll Control — ESP32
 *
 * Reads GPS on UART2, MPU6050 and BMP585 on I2C,
 * forwards all sensor data to USB serial for the laptop.
 * Receives ROLL,<angle> commands from the laptop and
 * actuates two canard servos via PCA9685 PWM controller.
 *
 * Wiring:
 *   GPS VIN      -> ESP32 3.3V
 *   GPS GND      -> ESP32 GND
 *   GPS TX       -> ESP32 GPIO 16
 *   GPS RX       -> ESP32 GPIO 17
 *   MPU SDA      -> ESP32 GPIO 21  (shared I2C bus)
 *   MPU SCL      -> ESP32 GPIO 22  (shared I2C bus)
 *   BMP SDA      -> ESP32 GPIO 21  (shared I2C bus)
 *   BMP SCL      -> ESP32 GPIO 22  (shared I2C bus)
 *   PCA9685 SDA  -> ESP32 GPIO 21  (shared I2C bus)
 *   PCA9685 SCL  -> ESP32 GPIO 22  (shared I2C bus)
 *   Canard 1     -> PCA9685 channel 12
 *   Canard 2     -> PCA9685 channel 13
 *
 * Install libraries in Arduino IDE (Tools -> Manage Libraries):
 *   - Adafruit MPU6050
 *   - Adafruit BMP5xx
 *   - Adafruit PWM Servo Driver Library
 *   - Adafruit Unified Sensor
 *
 * Board: NodeMCU-32S
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_PWMServoDriver.h>

// ── GPS config ───────────────────────────────────────────────
#define GPS_RX_PIN  16
#define GPS_TX_PIN  17
#define GPS_BAUD    9600

// ── BMP585 config ────────────────────────────────────────────
#define BMP585_ADDR BMP5XX_DEFAULT_ADDRESS  // 0x46

// ── IMU config ───────────────────────────────────────────────
Adafruit_MPU6050 mpu;
unsigned long lastIMU = 0;
const unsigned long IMU_INTERVAL = 50;  // 50ms = 20Hz

// ── BMP585 state ─────────────────────────────────────────────
Adafruit_BMP5xx bmp585;
float groundPressure  = 1013.25f;
bool  bmpReady        = false;
unsigned long lastALT = 0;
const unsigned long ALT_INTERVAL = 100;  // 100ms = 10Hz

// ── PCA9685 / servo config ───────────────────────────────────
#define PCA9685_ADDR   0x40
#define SERVO_FREQ_HZ  50
#define SERVO_MIN_US   500    // pulse width for 0°
#define SERVO_MAX_US   2400   // pulse width for 180°
#define CANARD1_CH     12
#define CANARD2_CH     13
#define NEUTRAL_ANGLE  90.0f  // resting angle when no command
#define CANARD1_TRIM   0.0f   // tune +/- until canard 1 is physically neutral
#define CANARD2_TRIM   0.0f   // tune +/- until canard 2 is physically neutral
#define MAX_DEFLECTION 15.0f  // matches GroundRollControlTest.py clamp

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ── Buffers ──────────────────────────────────────────────────
String gpsBuffer = "";  // GPS NMEA line assembly
String cmdBuffer = "";  // incoming laptop command line assembly


// ── Helpers ──────────────────────────────────────────────────

// Convert servo angle (0–180°) to PCA9685 12-bit count
uint16_t angleToPWM(float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint16_t)(us / 20000.0f * 4096.0f);
}

// Apply differential deflection: canard1 goes up, canard2 goes down (and vice versa)
void setCanards(float fin_command) {
  fin_command = constrain(fin_command, -MAX_DEFLECTION, MAX_DEFLECTION);
  float angle1 = NEUTRAL_ANGLE + CANARD1_TRIM + fin_command;
  float angle2 = NEUTRAL_ANGLE + CANARD2_TRIM - fin_command;
  pwm.setPWM(CANARD1_CH, 0, angleToPWM(angle1));
  pwm.setPWM(CANARD2_CH, 0, angleToPWM(angle2));
  Serial.printf("[ROLL] cmd=%.2f  canard1=%.1f°  canard2=%.1f°\n",
                fin_command, angle1, angle2);
}

// Parse and dispatch a complete command line from the laptop
void processCommand(const String& line) {
  if (line.startsWith("ROLL,")) {
    float cmd = line.substring(5).toFloat();
    setCanards(cmd);
  }
}


// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(21, 22);

  Serial.println("==========================================");
  Serial.println("  Avionics Sensor Bridge + Roll Control");
  Serial.println("==========================================");

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 not found! Check wiring.");
  } else {
    Serial.println("[OK] MPU6050 initialized");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // BMP585
  if (!bmp585.begin((uint8_t)BMP585_ADDR)) {
    Serial.println("[ERROR] BMP585 not found! Check wiring and address 0x47.");
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

  // PCA9685
  pwm.begin();
  // If servos move to wrong angles, adjust oscillator to match your board:
  //   Genuine Adafruit PCA9685: 25000000
  //   Many clone boards:        27000000
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);
  // Park canards at neutral on startup
  pwm.setPWM(CANARD1_CH, 0, angleToPWM(NEUTRAL_ANGLE));
  pwm.setPWM(CANARD2_CH, 0, angleToPWM(NEUTRAL_ANGLE));
  Serial.println("[OK] PCA9685 initialized — canards at neutral (ch12, ch13)");

  Serial.println("[OK] GPS UART initialized");
  Serial.println("==========================================");
  Serial.println();
}


// ── Main loop ────────────────────────────────────────────────
void loop() {
  // Forward GPS NMEA sentences to USB serial
  while (Serial2.available()) {
    char c = Serial2.read();
    gpsBuffer += c;
    if (c == '\n') {
      Serial.print(gpsBuffer);
      gpsBuffer = "";
    }
  }

  // IMU at 20Hz
  if (millis() - lastIMU >= IMU_INTERVAL) {
    lastIMU = millis();
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    Serial.print("$IMU,");
    Serial.print(accel.acceleration.x, 3); Serial.print(",");
    Serial.print(accel.acceleration.y, 3); Serial.print(",");
    Serial.print(accel.acceleration.z, 3); Serial.print(",");
    Serial.print(gyro.gyro.x, 3);          Serial.print(",");
    Serial.print(gyro.gyro.y, 3);          Serial.print(",");
    Serial.println(gyro.gyro.z, 3);
  }

  // BMP585 altitude at 10Hz
  if (bmpReady && (millis() - lastALT >= ALT_INTERVAL)) {
    lastALT = millis();
    if (bmp585.performReading()) {
      float agl = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
      Serial.print("$ALT,");
      Serial.println(agl, 2);
    }
  }

  // Parse ROLL commands from the laptop (line-buffered)
  while (Serial.available()) {
    char c = Serial.read();
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