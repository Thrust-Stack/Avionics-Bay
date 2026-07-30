/*
 * BRC_Firmware -- Flight roll-RATE controller, standalone on ESP32.
 *
 * Port of BRC.py. Drives roll rate to zero (stops the rocket spinning) with a
 * gentle proportional law, gated by an attitude/altitude safety check. This is
 * the proven, simple flight law -- no roll test (see FRCT_Firmware for that).
 *
 * On-board: reads MPU9250/6500 (I2C) + BMP585 (I2C) + GPS (UART2), computes the
 * fin command, drives two canard servos (LEDC PWM), and logs to microSD (CSV).
 *
 * Libraries: FastIMU, Adafruit BMP5xx, Adafruit Unified Sensor, TinyGPSPlus.
 * Wiring: see README.md. Board: NodeMCU-32S. Card must be FAT32.
 *
 * FLIGHT NOTE: SD writes can occasionally block ~tens of ms. This single-loop
 * version batches flushes to bound that. For maximum timing margin, move SD
 * logging to core 0 via a FreeRTOS task/queue (see README).
 */

#include <Wire.h>
#include <FastIMU.h>
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
#define SD_SCK_PIN   14
#define SD_MISO_PIN  27
#define SD_MOSI_PIN  13
#define SD_CS_PIN    33

// ---- Servo PWM ----
#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define NEUTRAL_ANGLE 90.0f

// ---- Control + safety (from BRC.py) ----
#define TARGET_ROLL_RATE       0.0f      // deg/s
#define KP                     0.0025f   // proportional gain on roll rate
#define MAX_FIN_DEFLECTION     7.5f      // degrees
#define MIN_CONTROL_ALTITUDE_M 0.0f
#define MAX_X_ROTATION_DEG     90.0f
#define MAX_Y_ROTATION_DEG     90.0f

// ---- Timing ----
#define CONTROL_PERIOD_MS 10    // 100 Hz (BRC.py used 0.01 s)
#define BMP_PERIOD_MS     100   // 10 Hz
#define SD_FLUSH_MS       250
#define STATUS_PRINT_MS   500

// ---- Objects ----
MPU9250 imu;                 // <-- change to  MPU6500 imu;  if init fails
calData imuCalib = { 0 };
AccelData accelData;
GyroData  gyroData;
Adafruit_BMP5xx bmp585;
TinyGPSPlus gps;
SPIClass sdSPI(HSPI);
File logFile;

bool  mpuReady = false, bmpReady = false, sdReady = false;
float groundPressure = 1013.25f;
float altitudeM = 0.0f;
float lastCanard1 = NEUTRAL_ANGLE, lastCanard2 = NEUTRAL_ANGLE;
unsigned long lastControl = 0, lastBmp = 0, lastFlush = 0, lastStatus = 0;

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
  lastCanard2 = NEUTRAL_ANGLE - fin_command;
  ledcWrite(CANARD1_PIN, angleToPWM16(lastCanard1));
  ledcWrite(CANARD2_PIN, angleToPWM16(lastCanard2));
}

void estimateRotation(float ax, float ay, float az, float &rx, float &ry){
  // The rocket's vertical/roll axis is MPU X. Treat MPU X as the old
  // longitudinal reference while preserving the existing two tilt outputs.
  rx = degrees(atan2f(ay, sqrtf(ax * ax + az * az)));
  ry = degrees(atan2f(-az, sqrtf(ay * ay + ax * ax)));
}

bool controlAllowed(float alt, float rotx, float roty){
  return (alt >= MIN_CONTROL_ALTITUDE_M)
      && (fabsf(rotx) < MAX_X_ROTATION_DEG)
      && (fabsf(roty) < MAX_Y_ROTATION_DEG);
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
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  Serial.println("BRC_Firmware -- flight roll-rate controller");

  int e = imu.init(imuCalib, IMU_ADDRESS);
  if(e != 0){
    Serial.printf("[ERR] IMU init code %d -- if wiring is good, change 'MPU9250 imu;' to 'MPU6500 imu;'\n", e);
  } else {
    mpuReady = true;
    imu.setGyroRange(500);
    imu.setAccelRange(16);
    Serial.println("[OK] IMU");
  }

  if(!bmp585.begin((uint8_t)BMP585_ADDR)){
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

  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if(!SD.begin(SD_CS_PIN, sdSPI)){
    Serial.println("[ERR] SD init -- logging disabled");
  } else {
    String fn = nextLogName("BRC");
    logFile = SD.open(fn.c_str(), FILE_WRITE);
    if(logFile){
      logFile.println("millis,state,fresh,gps_lat,gps_lon,gps_mph,gps_course,alt_m,"
                      "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                      "roll_rate,rot_x,rot_y,control_allowed,fin_cmd,canard1,canard2");
      logFile.flush();
      sdReady = true;
      Serial.printf("[OK] SD logging to %s\n", fn.c_str());
    } else Serial.println("[ERR] SD open failed");
  }

  Serial.println("Running.");
}

void loop(){
  while(Serial2.available()) gps.encode(Serial2.read());
  unsigned long nowMs = millis();

  if(bmpReady && nowMs - lastBmp >= BMP_PERIOD_MS){
    lastBmp = nowMs;
    if(bmp585.performReading())
      altitudeM = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
  }

  if(nowMs - lastControl >= CONTROL_PERIOD_MS){
    lastControl = nowMs;

    bool fresh = mpuReady;   // NOTE: runtime IMU-dropout detection not implemented
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    float rollRate = 0, rotx = 0, roty = 0, cmd = 0;
    bool allowed = false;

    if(fresh){
      imu.update();
      imu.getAccel(&accelData);
      imu.getGyro(&gyroData);
      ax = accelData.accelX; ay = accelData.accelY; az = accelData.accelZ;
      gx = gyroData.gyroX;   gy = gyroData.gyroY;   gz = gyroData.gyroZ;
      // MPU X is the rocket's roll axis. Re-verify the positive sign on the bench.
      rollRate = gx;
      estimateRotation(ax, ay, az, rotx, roty);
      allowed = controlAllowed(altitudeM, rotx, roty);
      cmd = allowed ? clampf(KP * (TARGET_ROLL_RATE - rollRate),
                             -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION)
                    : 0.0f;
    }
    setCanards(cmd);

    const char* state = !fresh ? "STALE" : (allowed ? "CONTROL_ACTIVE" : "IDLE");

    if(sdReady && logFile){
      char line[230];
      snprintf(line, sizeof(line),
        "%lu,%s,%d,%.6f,%.6f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.1f,%.1f\n",
        nowMs, state, fresh ? 1 : 0,
        gps.location.isValid() ? gps.location.lat() : 0.0,
        gps.location.isValid() ? gps.location.lng() : 0.0,
        gps.speed.isValid()    ? gps.speed.mph()    : 0.0,
        gps.course.isValid()   ? gps.course.deg()   : 0.0,
        altitudeM, ax, ay, az, gx, gy, gz,
        rollRate, rotx, roty, allowed ? 1 : 0, cmd, lastCanard1, lastCanard2);
      logFile.print(line);
    }

    if(nowMs - lastStatus >= STATUS_PRINT_MS){
      lastStatus = nowMs;
      Serial.printf("%-14s roll=%.1f rotX=%.1f rotY=%.1f alt=%.1f cmd=%.2f\r",
                    state, rollRate, rotx, roty, altitudeM, cmd);
    }
  }

  if(sdReady && logFile && nowMs - lastFlush >= SD_FLUSH_MS){
    lastFlush = nowMs;
    logFile.flush();
  }
}
