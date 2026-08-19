/*
 * FRCT_Firmware -- Flight Roll-Control + Roll-Test, standalone on ESP32.
 *
 * Port of FRCT.py. Sequence:
 *   PRE_TEST   : roll-RATE damping (drive gyro_x -> 0). Count BMP samples > 200 m.
 *   TRIGGER    : after 4 consecutive $ALT samples > 200 m, LATCH (one-time) and
 *                zero the gyro-integrated roll angle.
 *   ROLL_TEST  : 0 -> +90 deg (right) at 180 deg/s, hold 0.2 s, +90 -> 0.
 *                Phases advance within +/-2 deg. PD tracking of a ramped setpoint.
 *   POST_TEST  : resume roll-RATE damping for the rest of the flight.
 * Fail-safe: roll-test time budget exceeded (or IMU not ready) -> FAILSAFE_LOCK,
 *   canards neutral for the rest of the flight.
 *
 * On-board: reads MPU9250/6500 (I2C) + BMP585 (I2C) + GPS (UART2), drives two
 * canard servos (LEDC PWM), logs everything to microSD (CSV), and forwards
 * flight-format telemetry packets to the avionics Heltec.
 * Heltec downlink lines: GPS NMEA, $IMU, $ALT, and $CTRL.
 *
 * Libraries: FastIMU, Adafruit BMP5xx, Adafruit Unified Sensor, TinyGPSPlus.
 * Wiring: see README.md. Board: NodeMCU-32S. Card must be FAT32.
 *
 * IMPORTANT (flight hardware):
 *   - KP_ANGLE / KD_ANGLE depend on canard authority at airspeed and MUST be
 *     validated in sim/on the bench before flight (same caveat as FRCT.py).
 *   - MPU X is the ROLL axis; re-verify its positive direction on the bench.
 *   - SD writes can block ~tens of ms; for max timing margin move logging to
 *     core 0 (FreeRTOS task/queue). This version batches flushes to bound it.
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
// RC servos provide no position feedback. Set this to the PWM angle that
// matches the canards' physical startup/neutral position.
#ifndef STARTING_CANARD_POSITION_DEG
#define STARTING_CANARD_POSITION_DEG 90.0f
#endif
#ifndef STARTING_CANARD1_POSITION_DEG
#define STARTING_CANARD1_POSITION_DEG STARTING_CANARD_POSITION_DEG
#endif
#ifndef STARTING_CANARD2_POSITION_DEG
#define STARTING_CANARD2_POSITION_DEG STARTING_CANARD_POSITION_DEG
#endif

// MPU reads are converted to g; the Heltec bridge wants m/s^2.
#define G_TO_MS2 9.80665f

// ---- Rate damping (PRE/POST test) ----
#define TARGET_ROLL_RATE   0.0f
#define KP_RATE            0.0025f
#define MAX_FIN_DEFLECTION 7.5f

// ---- Roll test (from FRCT.py) ----
#define ROLL_TEST_ALTITUDE_M     200.0f
#define ROLL_TEST_ARM_SAMPLES    4
#define ROLL_TEST_TARGET_DEG     90.0f
#define ROLL_TEST_SLEW_DEG_S     180.0f
#define ROLL_TEST_HOLD_MS        200      // 0.2 s
#define ROLL_TEST_ANGLE_TOL_DEG  2.0f
#define ROLL_TEST_TIMEOUT_MS     2500     // 2.5 s
#define KP_ANGLE                 0.12f    // TUNABLE -- validate before flight
#define KD_ANGLE                 0.02f    // TUNABLE -- validate before flight
#define MAX_INTEGRATION_DT_S     0.1f

// ---- Timing ----
#define CONTROL_PERIOD_MS 10    // 100 Hz
#define BMP_PERIOD_MS     100   // 10 Hz
#define HELTEC_IMU_MS      50   // 20 Hz, matches the flight telemetry bridge
#define HELTEC_CTRL_MS    100   // 10 Hz controller/canard state downlink
#define SD_FLUSH_MS       250
#define STATUS_PRINT_MS   250
#define GPS_LINE_CAPACITY 96

// ---- State machine ----
enum State { PRE_TEST, ROLL_TEST, POST_TEST, FAILSAFE_LOCK };
enum Phase { RAMP_UP, HOLD, RAMP_DOWN };

const char* stateName(State s){
  switch(s){ case PRE_TEST: return "PRE_TEST"; case ROLL_TEST: return "ROLL_TEST";
             case POST_TEST: return "POST_TEST"; default: return "FAILSAFE_LOCK"; }
}
const char* phaseName(Phase p){
  switch(p){ case RAMP_UP: return "RAMP_UP"; case HOLD: return "HOLD"; default: return "RAMP_DOWN"; }
}

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
float neutralCanard1Angle = STARTING_CANARD1_POSITION_DEG;
float neutralCanard2Angle = STARTING_CANARD2_POSITION_DEG;
float lastCanard1 = STARTING_CANARD1_POSITION_DEG, lastCanard2 = STARTING_CANARD2_POSITION_DEG;

State state = PRE_TEST;
Phase phase = RAMP_UP;
float rollAngle = 0.0f;        // gyro-integrated; only meaningful during the test
int   altAboveCount = 0;
unsigned long testStartMs = 0, holdStartMs = 0;
float setpoint = 0.0f, setpointRate = 0.0f;
unsigned long lastControl = 0, lastBmp = 0, lastHeltecImu = 0, lastHeltecCtrl = 0, lastFlush = 0, lastStatus = 0;
unsigned long lastControlMicros = 0;
char gpsLine[GPS_LINE_CAPACITY + 1] = {};
size_t gpsLineLength = 0;
bool discardOversizeGpsLine = false;

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
  lastCanard1 = clampf(neutralCanard1Angle + fin_command, 0.0f, 180.0f);
  lastCanard2 = clampf(neutralCanard2Angle + fin_command, 0.0f, 180.0f);
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

void sendHeltecControl(unsigned long nowMs, State currentState, Phase currentPhase, bool fresh,
                       float rollRate, float rollAngleDeg, float rollSetpoint,
                       float rollSetpointRate, float cmd){
  const char* phaseText = (currentState == ROLL_TEST) ? phaseName(currentPhase) : "-";
  char ctrlLine[180];
  snprintf(ctrlLine, sizeof(ctrlLine),
           "$CTRL,%lu,FRCT,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f\n",
           nowMs, stateName(currentState), phaseText, fresh ? 1 : 0,
           rollRate, rollAngleDeg, rollSetpoint, rollSetpointRate,
           cmd, lastCanard1, lastCanard2);
  Serial1.print(ctrlLine);
}

float rateDampCommand(float rollRate){
  return clampf(KP_RATE * (TARGET_ROLL_RATE - rollRate),
                -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
}

float angleTrackCommand(float sp, float spRate, float angle, float rate){
  return clampf(KP_ANGLE * (sp - angle) + KD_ANGLE * (spRate - rate),
                -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
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
  Wire.setClock(400000);

  Serial.println("FRCT_Firmware -- flight roll control + 200 m roll test");

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
    String fn = nextLogName("FRCT");
    logFile = SD.open(fn.c_str(), FILE_WRITE);
    if(logFile){
      logFile.println("millis,state,phase,fresh,gps_lat,gps_lon,gps_mph,gps_course,alt_m,"
                      "ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                      "roll_rate,roll_angle,setpoint,fin_cmd,canard1,canard2");
      logFile.flush();
      sdReady = true;
      Serial.printf("[OK] SD logging to %s\n", fn.c_str());
    } else Serial.println("[ERR] SD open failed");
  }

  if(!mpuReady) state = FAILSAFE_LOCK;   // no IMU -> canards locked neutral
  lastControlMicros = micros();
  Serial.println("Running.");
}

void loop(){
  while(Serial2.available()){
    char c = Serial2.read();
    gps.encode(c);
    forwardGpsCharToHeltec(c);
  }

  unsigned long nowMs = millis();

  // BMP altitude at 10 Hz; arm the roll test here on consecutive samples > 200 m
  if(bmpReady && nowMs - lastBmp >= BMP_PERIOD_MS){
    lastBmp = nowMs;
    if(bmp585.performReading()){
      altitudeM = 44330.0f * (1.0f - powf(bmp585.pressure / groundPressure, 0.1903f));
      sendHeltecAlt(altitudeM);
      if(state == PRE_TEST){
        if(altitudeM > ROLL_TEST_ALTITUDE_M) altAboveCount++;
        else                                 altAboveCount = 0;
        if(altAboveCount >= ROLL_TEST_ARM_SAMPLES){
          state = ROLL_TEST; phase = RAMP_UP;
          rollAngle = 0.0f; setpoint = 0.0f; setpointRate = 0.0f;
          testStartMs = nowMs;
          Serial.printf("\n[ARM] roll test @ %.1f m\n", altitudeM);
        }
      }
    }
  }

  if(nowMs - lastControl >= CONTROL_PERIOD_MS){
    lastControl = nowMs;

    unsigned long us = micros();
    float dt = (us - lastControlMicros) / 1000000.0f;
    lastControlMicros = us;
    dt = clampf(dt, 0.0f, MAX_INTEGRATION_DT_S);

    bool fresh = mpuReady;   // NOTE: runtime IMU-dropout detection not implemented
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, rollRate = 0, cmd = 0;

    if(fresh){
      imu.update();
      imu.getAccel(&accelData);
      imu.getGyro(&gyroData);
      ax = accelData.accelX; ay = accelData.accelY; az = accelData.accelZ;
      gx = gyroData.gyroX;   gy = gyroData.gyroY;   gz = gyroData.gyroZ;
      // MPU X is the rocket's roll axis. Re-verify the positive sign on the bench.
      rollRate = gx;
    }

    if(state == FAILSAFE_LOCK){
      cmd = 0.0f;
    } else if(!fresh){
      // Never continue an open-loop maneuver on bad data.
      if(state == ROLL_TEST) state = FAILSAFE_LOCK;
      cmd = 0.0f;
    } else if(state == PRE_TEST){
      cmd = rateDampCommand(rollRate);
    } else if(state == ROLL_TEST){
      rollAngle += rollRate * dt;
      if(nowMs - testStartMs > ROLL_TEST_TIMEOUT_MS){
        state = FAILSAFE_LOCK;
        cmd = 0.0f;
        Serial.printf("\n[FAILSAFE] roll test timeout -- canards locked neutral\n");
      } else {
        if(phase == RAMP_UP){
          setpoint = fminf(ROLL_TEST_TARGET_DEG, setpoint + ROLL_TEST_SLEW_DEG_S * dt);
          setpointRate = ROLL_TEST_SLEW_DEG_S;
          if(setpoint >= ROLL_TEST_TARGET_DEG &&
             fabsf(rollAngle - ROLL_TEST_TARGET_DEG) <= ROLL_TEST_ANGLE_TOL_DEG){
            phase = HOLD; setpoint = ROLL_TEST_TARGET_DEG; setpointRate = 0.0f;
            holdStartMs = nowMs;
          }
        } else if(phase == HOLD){
          setpoint = ROLL_TEST_TARGET_DEG; setpointRate = 0.0f;
          if(nowMs - holdStartMs >= ROLL_TEST_HOLD_MS) phase = RAMP_DOWN;
        } else { // RAMP_DOWN
          setpoint = fmaxf(0.0f, setpoint - ROLL_TEST_SLEW_DEG_S * dt);
          setpointRate = -ROLL_TEST_SLEW_DEG_S;
          if(setpoint <= 0.0f && fabsf(rollAngle) <= ROLL_TEST_ANGLE_TOL_DEG){
            setpoint = 0.0f; setpointRate = 0.0f;
            state = POST_TEST;
            Serial.printf("\n[DONE] roll test complete -- resuming rate damping\n");
          }
        }
        cmd = angleTrackCommand(setpoint, setpointRate, rollAngle, rollRate);
      }
    } else { // POST_TEST
      cmd = rateDampCommand(rollRate);
    }

    setCanards(cmd);
    float axMs2 = ax * G_TO_MS2, ayMs2 = ay * G_TO_MS2, azMs2 = az * G_TO_MS2;

    if(fresh && nowMs - lastHeltecImu >= HELTEC_IMU_MS){
      lastHeltecImu = nowMs;
      sendHeltecImu(axMs2, ayMs2, azMs2, gx, gy, gz);
    }

    if(nowMs - lastHeltecCtrl >= HELTEC_CTRL_MS){
      lastHeltecCtrl = nowMs;
      sendHeltecControl(nowMs, state, phase, fresh, rollRate, rollAngle,
                        setpoint, setpointRate, cmd);
    }

    if(sdReady && logFile){
      const char* ph = (state == ROLL_TEST) ? phaseName(phase) : "-";
      char line[240];
      snprintf(line, sizeof(line),
        "%lu,%s,%s,%d,%.6f,%.6f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f\n",
        nowMs, stateName(state), ph, fresh ? 1 : 0,
        gps.location.isValid() ? gps.location.lat() : 0.0,
        gps.location.isValid() ? gps.location.lng() : 0.0,
        gps.speed.isValid()    ? gps.speed.mph()    : 0.0,
        gps.course.isValid()   ? gps.course.deg()   : 0.0,
        altitudeM, ax, ay, az, gx, gy, gz,
        rollRate, rollAngle, setpoint, cmd, lastCanard1, lastCanard2);
      logFile.print(line);
    }

    if(nowMs - lastStatus >= STATUS_PRINT_MS){
      lastStatus = nowMs;
      Serial.printf("%-13s %-8s rate=%.1f ang=%.1f sp=%.1f alt=%.1f cmd=%.2f\r",
                    stateName(state), (state == ROLL_TEST) ? phaseName(phase) : "-",
                    rollRate, rollAngle, setpoint, altitudeM, cmd);
    }
  }

  if(sdReady && logFile && nowMs - lastFlush >= SD_FLUSH_MS){
    lastFlush = nowMs;
    logFile.flush();
  }
}
