/*
 * Avionics Sensor Bridge - ESP32
 *
 * Reads GPS on UART2, MPU6050 and BMP585 on I2C,
 * forwards all to USB serial for the laptop to parse.
 *
 * Wiring:
 *   GPS VIN  -> ESP32 3.3V
 *   GPS GND  -> ESP32 GND
 *   GPS TX   -> ESP32 GPIO 16
 *   GPS RX   -> ESP32 GPIO 17
 *   MPU VCC  -> ESP32 3.3V
 *   MPU GND  -> ESP32 GND
 *   MPU SDA  -> ESP32 GPIO 21
 *   MPU SCL  -> ESP32 GPIO 22
 *   BMP SDA  -> ESP32 GPIO 21  (shared I2C bus)
 *   BMP SCL  -> ESP32 GPIO 22  (shared I2C bus)
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

// GPS config
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD   9600

// BMP585 I2C address: 0x46 default (SDO floating/GND), 0x47 if SDO tied to 3.3V
#define BMP585_ADDR BMP5XX_DEFAULT_ADDRESS  // 0x46

// IMU config
Adafruit_MPU6050 mpu;
unsigned long lastIMU = 0;
const unsigned long IMU_INTERVAL = 50;  // 50ms = 20Hz

// BMP585 config
Adafruit_BMP5xx bmp585;
float groundPressure = 1013.25f;  // hPa -- overwritten during calibration
bool bmpReady = false;
unsigned long lastALT = 0;
const unsigned long ALT_INTERVAL = 100;  // 100ms = 10Hz

// GPS line buffer -- prevents $IMU/$ALT from being injected mid-sentence
String gpsBuffer = "";

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(21, 22);

  Serial.println("==========================================");
  Serial.println("  Avionics Sensor Bridge");
  Serial.println("==========================================");

  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 not found! Check wiring.");
  } else {
    Serial.println("[OK] MPU6050 initialized");
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  if (!bmp585.begin((uint8_t)BMP585_ADDR)) {
    Serial.println("[ERROR] BMP585 not found! Check wiring and address 0x47.");
  } else {
    Serial.println("[OK] BMP585 initialized");

    bmp585.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
    bmp585.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
    bmp585.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

    // Average 20 readings over 2 seconds for AGL zero reference
    Serial.println("[BMP] Calibrating ground pressure...");
    delay(500);  // let sensor stabilise before sampling
    float pressureSum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 20; i++) {
      if (bmp585.performReading()) {
        pressureSum += bmp585.pressure;  // hPa
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

  Serial.println("[OK] GPS UART initialized");
  Serial.println("==========================================");
  Serial.println();
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    gpsBuffer += c;
    if (c == '\n') {
      Serial.print(gpsBuffer);
      gpsBuffer = "";
    }
  }

  if (millis() - lastIMU >= IMU_INTERVAL) {
    lastIMU = millis();
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    Serial.print("$IMU,");
    Serial.print(accel.acceleration.x, 3); Serial.print(",");
    Serial.print(accel.acceleration.y, 3); Serial.print(",");
    Serial.print(accel.acceleration.z, 3); Serial.print(",");
    Serial.print(gyro.gyro.x, 3); Serial.print(",");
    Serial.print(gyro.gyro.y, 3); Serial.print(",");
    Serial.println(gyro.gyro.z, 3);
  }

  if (bmpReady && (millis() - lastALT >= ALT_INTERVAL)) {
    lastALT = millis();
    if (bmp585.performReading()) {
      // Hypsometric formula -- both values in hPa so ratio is unit-agnostic
      float agl = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
      Serial.print("$ALT,");
      Serial.println(agl, 2);
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }
}