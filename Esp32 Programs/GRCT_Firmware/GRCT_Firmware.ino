/*
 * GRCT_Firmware -- Ground Roll-Control Test, standalone on ESP32.
 *
 * Port of GRCT.py. GROUND BENCH TEST ONLY: aggressive gains, bypasses ALL
 * altitude safety logic. *** DO NOT FLY THIS SKETCH ***
 *
 * On-board: reads MPU9250/6500 (I2C) + BMP585 (I2C) + GPS (UART2), runs
 * roll-position hold around the boot-time zero angle, drives two canard servos
 * (LEDC PWM), logs every sample plus canard angles to microSD (SPI, CSV),
 * and forwards flight-format telemetry packets to the avionics Heltec.
 * Heltec downlink lines: GPS NMEA, $IMU, $ALT, and $CTRL.
 *
 * Libraries (Arduino IDE -> Manage Libraries):
 *   Adafruit BMP5xx, Adafruit Unified Sensor, TinyGPSPlus.
 *   (SD, SPI, Wire are built into the ESP32 core.)
 *
 * Wiring (see README.md):
 *   I2C  SDA=23  SCL=32          Canards  1=GPIO26  2=GPIO25
 *   ADA254 SD  CLK=14 DO=27 DI=13 CS=33   GPS UART2 RX=16 TX=17
 *   Heltec UART TX=19            ESP32 TX19 -> Heltec GPIO44
 *
 * Board: NodeMCU-32S.  Card must be FAT32.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>

// ---- Pins ----
#define GPS_RX_PIN   16
#define GPS_TX_PIN   17
#define GPS_BAUD     9600
#define I2C_SDA_PIN  23
#define I2C_SCL_PIN  32
#define IMU_ADDRESS  0x68
#define BMP585_ADDR  BMP5XX_DEFAULT_ADDRESS   // 0x46
#define CANARD1_PIN  26
#define CANARD2_PIN  25
#define SD_SCK_PIN   14  // ADA254 CLK
#define SD_MISO_PIN  27  // ADA254 DO
#define SD_MOSI_PIN  13  // ADA254 DI
#define SD_CS_PIN    33  // ADA254 CS
#define HELTEC_TX_PIN 19  // ESP32 TX -> Heltec GPIO44 U0RXD
#define HELTEC_BAUD   115200

// ---- Servo PWM ----
#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define NEUTRAL_ANGLE 90.0f

// ---- Ground-test control ----
// GRCT zeros roll angle at boot. A 45 deg roll commands roughly 15 deg of fin
// deflection; larger roll errors clamp at 45 deg until the body returns toward 0 deg.
#define TARGET_ROLL_ANGLE_DEG 0.0f
#define ROLL_POSITION_GAIN    0.333f
#define MAX_FIN_DEFLECTION    45.0f
#define GYRO_PROCESS_VAR      0.1f
#define GYRO_MEASUREMENT_VAR  4.0f
#define GYRO_BIAS_SAMPLES     100

// MPU reads are converted to g; the log wants m/s^2.
#define G_TO_MS2 9.80665f

// ---- MPU6500 registers/scales ----
#define MPU_WHO_AM_I      0x75
#define MPU_PWR_MGMT_1    0x6B
#define MPU_SMPLRT_DIV    0x19
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_CONFIG2 0x1D
#define MPU_INT_PIN_CFG   0x37
#define MPU_INT_ENABLE    0x38
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU6500_WHOAMI    0x70
#define MPU9250_WHOAMI    0x71
#define MPU_GYRO_LSB_PER_DPS 65.5f    // +/-500 dps
#define MPU_ACCEL_LSB_PER_G  2048.0f  // +/-16 g

// ---- Timing ----
#define CONTROL_PERIOD_MS 20    // 50 Hz (GRCT.py used 0.02 s)
#define BMP_PERIOD_MS     100   // 10 Hz
#define HELTEC_IMU_MS      50   // 20 Hz, matches the flight telemetry bridge
#define HELTEC_CTRL_MS    100   // 10 Hz controller/filter/canard state downlink
#define SD_FLUSH_MS       250
#define STATUS_PRINT_MS   500
#define STATUS_HEADER_EVERY 20  // re-print the column header every N rows
#define IMU_MAX_READ_FAILURES 5
#define GPS_LINE_CAPACITY 96

// ---- Objects ----
Adafruit_BMP5xx bmp585;
TinyGPSPlus gps;
SPIClass sdSPI(HSPI);
File logFile;

bool  mpuReady = false, bmpReady = false, sdReady = false;
float groundPressure = 1013.25f;
float altitudeM = 0.0f;
float lastCanard1 = NEUTRAL_ANGLE, lastCanard2 = NEUTRAL_ANGLE;
float rollAngleDeg = 0.0f;
float gyroXBiasDps = 0.0f;
unsigned long lastControl = 0, lastBmp = 0, lastHeltecImu = 0, lastHeltecCtrl = 0, lastFlush = 0, lastStatus = 0;
unsigned long lineNo = 0;      // one per control sample, shared by SD + serial
uint16_t statusRows = 0;
uint8_t imuReadFailures = 0;
unsigned long lastImuWarn = 0;
char gpsLine[GPS_LINE_CAPACITY + 1] = {};
size_t gpsLineLength = 0;
bool discardOversizeGpsLine = false;

// Scalar Kalman filter for roll rate before angle integration.
struct Kalman1D {
  float q, r, x, p; bool init;
  void begin(float q_, float r_){ q = q_; r = r_; x = 0; p = 0; init = false; }
  float update(float z){
    if(!init){ x = z; p = r; init = true; return x; }
    float pp = p + q;
    float k  = pp / (pp + r);
    x += k * (z - x);
    p  = (1.0f - k) * pp;
    return x;
  }
} rollFilter;

// ---- Helpers ----
static inline float clampf(float v, float lo, float hi){
  return v < lo ? lo : (v > hi ? hi : v);
}

uint32_t angleToPWM16(float angle){
  angle = constrain(angle, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint32_t)(us / 20000.0f * 65535.0f);
}

void setCanards(float fin_command){
  fin_command = clampf(fin_command, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
  lastCanard1 = NEUTRAL_ANGLE + fin_command;
  lastCanard2 = NEUTRAL_ANGLE + fin_command;
  ledcWrite(CANARD1_PIN, angleToPWM16(lastCanard1));
  ledcWrite(CANARD2_PIN, angleToPWM16(lastCanard2));
}

void forwardGpsCharToHeltec(char c){
  if(c == '\n'){
    if(!discardOversizeGpsLine && gpsLineLength > 0){
      gpsLine[gpsLineLength] = '\0';
      if(gpsLine[0] == '$'){
        Serial1.print(gpsLine);
        Serial1.write('\n');
      }
    }
    gpsLineLength = 0;
    discardOversizeGpsLine = false;
    return;
  }

  if(discardOversizeGpsLine) return;

  if(gpsLineLength < GPS_LINE_CAPACITY){
    gpsLine[gpsLineLength++] = c;
  } else {
    gpsLineLength = 0;
    discardOversizeGpsLine = true;
  }
}

void sendHeltecImu(float axMs2, float ayMs2, float azMs2, float gxDps, float gyDps, float gzDps){
  char imuLine[96];
  snprintf(imuLine, sizeof(imuLine), "$IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           axMs2, ayMs2, azMs2,
           gxDps * DEG_TO_RAD, gyDps * DEG_TO_RAD, gzDps * DEG_TO_RAD);
  Serial1.print(imuLine);
}

void sendHeltecAlt(float aglMeters){
  char altLine[32];
  snprintf(altLine, sizeof(altLine), "$ALT,%.2f\n", aglMeters);
  Serial1.print(altLine);
}

void sendHeltecControl(unsigned long nowMs, bool imuFresh, float rawRollRate,
                       float filtRollRate, float rollAngle, float cmd){
  char ctrlLine[160];
  snprintf(ctrlLine, sizeof(ctrlLine),
           "$CTRL,%lu,GRCT,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f\n",
           nowMs, imuFresh ? 1 : 0, rawRollRate, filtRollRate,
           rollAngle, cmd, lastCanard1, lastCanard2, gyroXBiasDps);
  Serial1.print(ctrlLine);
}

bool mpuWrite(uint8_t reg, uint8_t value){
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadByte(uint8_t reg, uint8_t &value){
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  if(Wire.endTransmission(true) != 0) return false;
  delayMicroseconds(100);
  if(Wire.requestFrom((uint8_t)IMU_ADDRESS, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool mpuReadBytes(uint8_t reg, uint8_t *data, uint8_t count){
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  if(Wire.endTransmission(true) != 0) return false;
  delayMicroseconds(100);
  if(Wire.requestFrom((uint8_t)IMU_ADDRESS, count) != count) return false;
  for(uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

bool mpuRead16(uint8_t reg, int16_t &value){
  uint8_t raw[2];
  if(!mpuReadBytes(reg, raw, sizeof(raw))) return false;
  value = (int16_t)((raw[0] << 8) | raw[1]);
  return true;
}

void printI2CScan(){
  int found = 0;
  Serial.print("[INFO] I2C devices:");
  for(uint8_t addr = 0x03; addr <= 0x77; addr++){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0){
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if(found == 0) Serial.print(" none");
  Serial.println();
}

bool mpuBegin(){
  uint8_t who = 0;
  unsigned long deadline = millis() + 5000;
  while(millis() < deadline){
    if(mpuReadByte(MPU_WHO_AM_I, who)) break;
    delay(250);
  }
  if(who == 0){
    Serial.println("[ERR] IMU WHO_AM_I read failed");
    return false;
  }
  Serial.printf("[INFO] IMU WHO_AM_I 0x%02X\n", who);
  if(who != MPU6500_WHOAMI && who != MPU9250_WHOAMI){
    Serial.println("[ERR] IMU identity is not MPU6500/9250-compatible");
    return false;
  }

  if(!mpuWrite(MPU_PWR_MGMT_1, 0x80)) return false; // reset
  delay(100);
  if(!mpuWrite(MPU_PWR_MGMT_1, 0x01)) return false; // PLL clock, awake
  delay(100);
  if(!mpuWrite(MPU_SMPLRT_DIV, 0x04)) return false;
  if(!mpuWrite(MPU_CONFIG, 0x03)) return false;
  if(!mpuWrite(MPU_GYRO_CONFIG, 0x08)) return false;  // +/-500 dps
  if(!mpuWrite(MPU_ACCEL_CONFIG, 0x18)) return false; // +/-16 g
  if(!mpuWrite(MPU_ACCEL_CONFIG2, 0x03)) return false;
  if(!mpuWrite(MPU_INT_PIN_CFG, 0x22)) return false;
  if(!mpuWrite(MPU_INT_ENABLE, 0x01)) return false;
  return true;
}

bool mpuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz){
  int16_t rax, ray, raz, rgx, rgy, rgz;
  if(!mpuRead16(0x3B, rax)) return false;
  if(!mpuRead16(0x3D, ray)) return false;
  if(!mpuRead16(0x3F, raz)) return false;
  if(!mpuRead16(0x43, rgx)) return false;
  if(!mpuRead16(0x45, rgy)) return false;
  if(!mpuRead16(0x47, rgz)) return false;
  ax = (float)rax / MPU_ACCEL_LSB_PER_G;
  ay = (float)ray / MPU_ACCEL_LSB_PER_G;
  az = (float)raz / MPU_ACCEL_LSB_PER_G;
  gx = (float)rgx / MPU_GYRO_LSB_PER_DPS;
  gy = (float)rgy / MPU_GYRO_LSB_PER_DPS;
  gz = (float)rgz / MPU_GYRO_LSB_PER_DPS;
  return true;
}

void estimateRotation(float ax, float ay, float az, float &rx, float &ry){
  // The rocket's vertical/roll axis is MPU X. Treat MPU X as the old
  // longitudinal reference while preserving the existing two tilt outputs.
  rx = degrees(atan2f(ay, sqrtf(ax * ax + az * az)));
  ry = degrees(atan2f(-az, sqrtf(ay * ay + ax * ax)));
}

void printStatusHeader(){
  Serial.println();
  Serial.println("  line     time    gyroX    gyroY    gyroZ rollDeg  baroAlt   gpsAlt          lon          lat    accX    accY    accZ    can1    can2");
  Serial.println("------ -------- -------- -------- -------- ------- -------- -------- ------------ ------------ ------- ------- ------- ------- -------");
}

float calibrateGyroXBias(){
  float sum = 0.0f;
  int samples = 0;

  for(int i = 0; i < GYRO_BIAS_SAMPLES; i++){
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if(mpuRead(ax, ay, az, gx, gy, gz)){
      sum += gx;
      samples++;
    }
    delay(10);
  }

  return samples > 0 ? sum / (float)samples : 0.0f;
}

String nextLogName(const char* prefix){
  char name[24];
  for(int i = 0; i < 1000; i++){
    snprintf(name, sizeof(name), "/%s_%03d.csv", prefix, i);
    if(!SD.exists(name)) return String(name);
  }
  return String("/") + prefix + "_ovf.csv";
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(HELTEC_BAUD, SERIAL_8N1, -1, HELTEC_TX_PIN);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(500);

  Serial.println("GRCT_Firmware -- GROUND TEST ONLY (do not fly)");
  printI2CScan();

  if(!mpuBegin()){
    Serial.println("[ERR] IMU init failed -- check MPU power, SDA/SCL, address, and chip type");
  } else {
    mpuReady = true;
    Serial.println("[OK] IMU");
    Serial.println("[INFO] Hold still: calibrating gyro X bias for roll-angle integration");
    gyroXBiasDps = calibrateGyroXBias();
    Serial.printf("[OK] gyroX bias %.3f dps\n", gyroXBiasDps);
  }
  rollFilter.begin(GYRO_PROCESS_VAR, GYRO_MEASUREMENT_VAR);

  bool bmpFound = false;
  for(int i = 0; i < 10 && !bmpFound; i++){
    bmpFound = bmp585.begin((uint8_t)BMP585_ADDR);
    if(!bmpFound) delay(250);
  }
  if(!bmpFound){
    Serial.println("[ERR] BMP585 not found");
  } else {
    bmp585.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
    bmp585.setPressureOversampling(BMP5XX_OVERSAMPLING_4X);
    bmp585.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    delay(500);
    float sum = 0; int n = 0;
    for(int i = 0; i < 20; i++){ if(bmp585.performReading()){ sum += bmp585.pressure; n++; } delay(100); }
    if(n > 0){ groundPressure = sum / n; bmpReady = true; Serial.printf("[OK] BMP ground %.2f hPa\n", groundPressure); }
    else Serial.println("[ERR] BMP calibration failed");
  }

  ledcAttach(CANARD1_PIN, SERVO_FREQ_HZ, 16);
  ledcAttach(CANARD2_PIN, SERVO_FREQ_HZ, 16);
  setCanards(0.0f);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if(!SD.begin(SD_CS_PIN, sdSPI, 400000)){
    Serial.println("[ERR] SD init failed at 400 kHz -- logging disabled");
  } else if(SD.cardType() == CARD_NONE){
    Serial.println("[ERR] SD init found no card -- logging disabled");
  } else {
    uint8_t cardType = SD.cardType();
    Serial.print("[OK] SD card ");
    if(cardType == CARD_MMC) Serial.print("MMC");
    else if(cardType == CARD_SD) Serial.print("SDSC");
    else if(cardType == CARD_SDHC) Serial.print("SDHC/SDXC");
    else Serial.print("UNKNOWN");
    Serial.printf(" %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

    String fn = nextLogName("GRCT");
    logFile = SD.open(fn.c_str(), FILE_WRITE);
    if(logFile){
      logFile.println("line,timestamp_ms,gyro_x_dps,gyro_y_dps,gyro_z_dps,"
                      "baro_alt_m,gps_alt_m,longitude_deg,latitude_deg,"
                      "accel_x_ms2,accel_y_ms2,accel_z_ms2,roll_angle_deg,"
                      "canard1_deg,canard2_deg");
      logFile.flush();
      sdReady = true;
      Serial.printf("[OK] SD logging to %s\n", fn.c_str());
    } else Serial.println("[ERR] SD open failed");
  }

  Serial.println(mpuReady ? "Running -- GROUND_TEST_ACTIVE."
                          : "Running -- STALE_TELEMETRY (no IMU, canards held neutral).");
}

void loop(){
  while(Serial2.available()){
    char c = Serial2.read();
    gps.encode(c);
    forwardGpsCharToHeltec(c);
  }

  unsigned long nowMs = millis();

  if(bmpReady && nowMs - lastBmp >= BMP_PERIOD_MS){
    lastBmp = nowMs;
    if(bmp585.performReading()){
      altitudeM = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
      sendHeltecAlt(altitudeM);
    }
  }

  if(nowMs - lastControl >= CONTROL_PERIOD_MS){
    float dt = (nowMs - lastControl) / 1000.0f;
    if(dt <= 0.0f || dt > 0.25f) dt = CONTROL_PERIOD_MS / 1000.0f;
    lastControl = nowMs;

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float rawRollRate = 0, filtRollRate = 0, cmd = 0;
    bool imuFresh = false;

    if(mpuReady){
      if(mpuRead(ax, ay, az, gx, gy, gz)){
        imuReadFailures = 0;
        imuFresh = true;
        // MPU X is the rocket's roll axis. Roll position is integrated from
        // gyroX, then controlled back toward the boot-time 0 deg reference.
        rawRollRate = gx - gyroXBiasDps;
        filtRollRate = rollFilter.update(rawRollRate);
        rollAngleDeg += filtRollRate * dt;
        cmd = clampf(ROLL_POSITION_GAIN * (TARGET_ROLL_ANGLE_DEG - rollAngleDeg),
                     -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
      } else {
        imuReadFailures++;
        if(nowMs - lastImuWarn >= 1000){
          lastImuWarn = nowMs;
          Serial.println("[WARN] IMU read failed");
        }
        if(imuReadFailures >= IMU_MAX_READ_FAILURES){
          mpuReady = false;
          Serial.println("[ERR] IMU disabled after repeated read failures");
        }
      }
    }
    setCanards(cmd);   // cmd is 0 when IMU is not ready

    lineNo++;
    double gpsLon = gps.location.isValid() ? gps.location.lng()      : 0.0;
    double gpsLat = gps.location.isValid() ? gps.location.lat()      : 0.0;
    double gpsAlt = gps.altitude.isValid() ? gps.altitude.meters()   : 0.0;
    float  axMs2 = ax * G_TO_MS2, ayMs2 = ay * G_TO_MS2, azMs2 = az * G_TO_MS2;

    if(imuFresh && nowMs - lastHeltecImu >= HELTEC_IMU_MS){
      lastHeltecImu = nowMs;
      sendHeltecImu(axMs2, ayMs2, azMs2, gx, gy, gz);
    }

    if(nowMs - lastHeltecCtrl >= HELTEC_CTRL_MS){
      lastHeltecCtrl = nowMs;
      sendHeltecControl(nowMs, imuFresh, rawRollRate, filtRollRate, rollAngleDeg, cmd);
    }

    if(sdReady && logFile){
      char line[256];
      snprintf(line, sizeof(line),
        "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f\n",
        lineNo, nowMs, gx, gy, gz, altitudeM, gpsAlt, gpsLon, gpsLat,
        axMs2, ayMs2, azMs2, rollAngleDeg, lastCanard1, lastCanard2);
      logFile.print(line);
    }

    if(nowMs - lastStatus >= STATUS_PRINT_MS){
      lastStatus = nowMs;
      if(statusRows % STATUS_HEADER_EVERY == 0) printStatusHeader();
      statusRows++;
      Serial.printf("%6lu %8lu %8.2f %8.2f %8.2f %7.2f %8.2f %8.2f %12.6f %12.6f "
                    "%7.3f %7.3f %7.3f %7.1f %7.1f\n",
                    lineNo, nowMs, gx, gy, gz, rollAngleDeg, altitudeM, gpsAlt, gpsLon, gpsLat,
                    axMs2, ayMs2, azMs2, lastCanard1, lastCanard2);
    }
  }

  // Batched flush to bound SD blocking (see README timing note)
  if(sdReady && logFile && nowMs - lastFlush >= SD_FLUSH_MS){
    lastFlush = nowMs;
    logFile.flush();
  }
}
