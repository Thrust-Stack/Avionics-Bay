/*
 * Heltec WiFi LoRa 32 V4 - avionics UART/LoRa bridge
 *
 * Direct ESP32D UART wiring:
 *   ESP32D GPIO 19 (TX) -> Heltec GPIO 44 (U0RXD)
 *   ESP32D GND          -> Heltec GND
 *
 * UART packet contract:
 *   ESP32D -> Heltec: raw GPS NMEA, $IMU,...\n, and $ALT,...\n
 *
 * Build this avionics-side sketch with "USB CDC On Boot: Disabled" so Serial
 * is hardware UART0 RX on GPIO 44. Serial is protocol-only in direct mode:
 * never print status or debug messages to it.
 *
 * Requires RadioLib. Select the exact Heltec WiFi LoRa 32 V4 board/revision
 * and attach the correct regional LoRa antenna before transmitting.
 *
 * Tracking-mode bench test:
 *   1. Leave ENABLE_SYNTHETIC_ALTITUDE_TEST at 0 for real UART telemetry.
 *   2. For a no-sensor bench run, set ENABLE_SYNTHETIC_ALTITUDE_TEST to 1 and
 *      flash a ground receiver beside this transmitter. The sketch will inject
 *      synthetic GPS plus altitude samples: slow climb above 20 m, then descent.
 *   3. Expected ground terminal event lines:
 *        EVENT,MODE=FLIGHT,msg=LAUNCH ARMED,...
 *        EVENT,MODE=APOGEE_DETECT,msg=APOGEE DETECTED,...
 *        EVENT,MODE=TRACK,msg=TRACKING MODE ACTIVE (5s packets),...
 *      followed by one TRACK,MODE=TRACK,... line every 5 seconds.
 *
 * Tiny ground terminal parser for receiver USB output:
 *   import serial
 *   ser = serial.Serial("COM9", 115200, timeout=1)
 *   while True:
 *       line = ser.readline().decode(errors="replace").strip()
 *       if not line:
 *           continue
 *       print(line)
 *       if "MODE=TRACK" in line:
 *           print("TRACKING MODE ACTIVE")
 */

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <cmath>
#include <cstdlib>

// Direct ESP32D UART RX: 115200 baud, 8 data bits, no parity, 1 stop bit.
// HardwareSerial.begin() takes RX before TX: GPIO 44 U0RXD, TX disabled.

// Heltec V4 SX1262 and GC1109 front-end pins.
constexpr int LORA_SCK = 9;
constexpr int LORA_MISO = 11;
constexpr int LORA_MOSI = 10;
constexpr int LORA_NSS = 8;
constexpr int LORA_DIO1 = 14;
constexpr int LORA_RST = 12;
constexpr int LORA_BUSY = 13;
constexpr int LORA_FEM_EN = 2;

// Both Heltec radios must use identical settings.
constexpr float LORA_FREQUENCY_MHZ = 915.0;
constexpr float LORA_BANDWIDTH_KHZ = 125.0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;   // 5 means coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12;  // private point-to-point link
constexpr int8_t LORA_TX_POWER_DBM = 22;  // SX1262 practical max for Heltec V4 915 MHz
constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;
// SX1262 LoRa packets are limited to 255 bytes. We use a compact binary
// frame pre-apogee and a compact ASCII TRACK line post-apogee.
constexpr size_t UART_LINE_CAPACITY = 220;
constexpr size_t TELEMETRY_QUEUE_DEPTH = 8;
constexpr size_t FRAME_MAX_SIZE = 128;
constexpr uint32_t TX_LED_PULSE_MS = 25;

// ---- Flight-to-tracking state machine tuning ------------------------
// Set to 1 only for bench testing without the ESP32 sensor bridge. It injects
// synthetic GPS plus altitude into the same parser used by real UART lines.
constexpr bool ENABLE_SYNTHETIC_ALTITUDE_TEST = false;
constexpr uint32_t SYNTHETIC_SAMPLE_MS = 500;

// EMA smoothing alpha for altitude. 0.20 damps GPS/baro noise while still
// responding within a few samples. Lower it for noisy GPS-only altitude; raise
// it if the barometric $ALT source is stable and apogee detection feels late.
constexpr float ALTITUDE_EMA_ALPHA = 0.20f;

// Launch must exceed this smoothed AGL altitude before apogee can be detected.
constexpr float LAUNCH_ARM_ALTITUDE_M = 20.0f;

// Apogee descent trigger: current smoothed altitude must stay at least this far
// below the peak for both the count and hold-time criteria below.
constexpr float APOGEE_DROP_M = 10.0f;
constexpr uint8_t APOGEE_DESCENT_READING_COUNT = 3;
constexpr uint32_t APOGEE_HOLD_MS = 2000;

// TRACKING mode sends only GPS, altitude, timestamp, and MODE=TRACK at 5 s.
constexpr uint32_t TRACKING_TX_INTERVAL_MS = 5000;

// Freshness guard used by the failsafe. If GPS or altitude is stale, the state
// machine will not enter TRACKING and emits occasional SENSOR_ERROR events.
constexpr uint32_t SENSOR_STALE_MS = 2500;
constexpr uint32_t SENSOR_ERROR_LOG_INTERVAL_MS = 2000;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

char uartLine[UART_LINE_CAPACITY + 1] = {};
size_t uartLineLength = 0;
bool discardOversizeLine = false;

uint8_t telemetryQueue[TELEMETRY_QUEUE_DEPTH][FRAME_MAX_SIZE];
size_t telemetryQueueLen[TELEMETRY_QUEUE_DEPTH] = {0};
size_t queueHead = 0;
size_t queueTail = 0;
size_t queueCount = 0;
uint8_t activeTransmission[FRAME_MAX_SIZE];
size_t activeTransmissionLen = 0;

volatile bool radioOperationDone = false;
bool radioTransmitting = false;
bool txLedPulseActive = false;
uint32_t txLedOffAt = 0;

void IRAM_ATTR setRadioFlag() {
  radioOperationDone = true;
}

bool takeRadioFlag() {
  noInterrupts();
  const bool wasSet = radioOperationDone;
  radioOperationDone = false;
  interrupts();
  return wasSet;
}

void pulseTxLed() {
  digitalWrite(LED_BUILTIN, HIGH);
  txLedPulseActive = true;
  txLedOffAt = millis() + TX_LED_PULSE_MS;
}

void updateTxLed() {
  if (txLedPulseActive && static_cast<int32_t>(millis() - txLedOffAt) >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    txLedPulseActive = false;
  }
}

void enqueueTelemetry(const uint8_t* packet, size_t len) {
  if (len == 0 || len > FRAME_MAX_SIZE) return;
  // If LoRa cannot keep up, drop oldest to make room.
  if (queueCount == TELEMETRY_QUEUE_DEPTH) {
    telemetryQueueLen[queueHead] = 0;
    queueHead = (queueHead + 1) % TELEMETRY_QUEUE_DEPTH;
    --queueCount;
  }
  memcpy(telemetryQueue[queueTail], packet, len);
  telemetryQueueLen[queueTail] = len;
  queueTail = (queueTail + 1) % TELEMETRY_QUEUE_DEPTH;
  ++queueCount;
}

bool dequeueTelemetry(uint8_t* outBuf, size_t& outLen) {
  if (queueCount == 0) return false;
  outLen = telemetryQueueLen[queueHead];
  memcpy(outBuf, telemetryQueue[queueHead], outLen);
  telemetryQueueLen[queueHead] = 0;
  queueHead = (queueHead + 1) % TELEMETRY_QUEUE_DEPTH;
  --queueCount;
  return true;
}

bool isEsp32TelemetryLine(const char* line) {
  // GPS NMEA, $IMU, and $ALT packets all begin with '$'. Keep the original
  // bytes and line ending so the ground-side parser sees the ESP32D format.
  return line[0] == '$';
}

void readEsp32Telemetry() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n') {
      if (!discardOversizeLine && uartLineLength > 0) {
        uartLine[uartLineLength] = '\0';
        if (isEsp32TelemetryLine(uartLine)) {
          // Process textual telemetry and build a compact binary frame.
          uint8_t frame[FRAME_MAX_SIZE];
          size_t frame_len = 0;
          processTelemetryLineAndPack((const char*)uartLine, frame, frame_len);
          if (frame_len > 0) {
            enqueueTelemetry(frame, frame_len);
          }
        }
      }

      uartLineLength = 0;
      discardOversizeLine = false;
      continue;
    }

    if (discardOversizeLine) {
      continue;
    }

    if (uartLineLength < UART_LINE_CAPACITY) {
      uartLine[uartLineLength++] = c;
    } else {
      uartLineLength = 0;
      discardOversizeLine = true;
    }
  }
}

// --- telemetry aggregator, packing, CRC -------------------------------
struct TelemetryState {
  uint16_t seq = 0;
  uint32_t ts = 0;
  int32_t lat = 0;
  int32_t lon = 0;
  int32_t alt_mm = 0;
  int32_t gps_alt_mm = 0;
  uint16_t speed_cms = 0;
  uint16_t heading_cd = 0; // centi-degrees
  int16_t pitch_cd = 0;
  int16_t roll_cd = 0;
  int16_t yaw_cd = 0;
  uint16_t battery_mv = 0;
  uint8_t status = 0;
  bool gps_valid = false;
  bool baro_alt_valid = false;
  bool gps_alt_valid = false;
  uint32_t last_gps_ms = 0;
  uint32_t last_baro_alt_ms = 0;
  uint32_t last_gps_alt_ms = 0;
} tele;

enum class FlightMode : uint8_t {
  PRELAUNCH = 0,
  FLIGHT = 1,
  APOGEE_DETECT = 2,
  TRACKING = 3,
};

struct AltitudeFilter {
  bool initialized = false;
  float value_m = 0.0f;

  float update(float raw_m) {
    if (!initialized) {
      initialized = true;
      value_m = raw_m;
    } else {
      value_m += ALTITUDE_EMA_ALPHA * (raw_m - value_m);
    }
    return value_m;
  }
} altitudeFilter;

FlightMode flightMode = FlightMode::PRELAUNCH;
float peakAltitudeM = 0.0f;
uint8_t descentReadingCount = 0;
uint32_t descentHoldStartedMs = 0;
uint32_t lastSensorErrorLogMs = 0;
uint32_t lastTrackingTxMs = 0;
uint16_t trackingSeq = 0;
uint32_t lastSyntheticSampleMs = 0;

// CRC-16-CCITT (0x1021) with 0xFFFF initial value
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static inline void write_u16_be(uint8_t* buf, size_t& off, uint16_t v) {
  buf[off++] = (uint8_t)(v >> 8);
  buf[off++] = (uint8_t)(v & 0xFF);
}
static inline void write_u32_be(uint8_t* buf, size_t& off, uint32_t v) {
  buf[off++] = (uint8_t)(v >> 24);
  buf[off++] = (uint8_t)((v >> 16) & 0xFF);
  buf[off++] = (uint8_t)((v >> 8) & 0xFF);
  buf[off++] = (uint8_t)(v & 0xFF);
}
static inline void write_i32_be(uint8_t* buf, size_t& off, int32_t v) {
  write_u32_be(buf, off, (uint32_t)v);
}
static inline void write_i16_be(uint8_t* buf, size_t& off, int16_t v) {
  buf[off++] = (uint8_t)((v >> 8) & 0xFF);
  buf[off++] = (uint8_t)(v & 0xFF);
}

bool isFresh(uint32_t nowMs, uint32_t sampleMs) {
  return sampleMs != 0 && static_cast<uint32_t>(nowMs - sampleMs) <= SENSOR_STALE_MS;
}

bool hasFreshGps(uint32_t nowMs) {
  return tele.gps_valid && isFresh(nowMs, tele.last_gps_ms);
}

bool hasFreshBaroAltitude(uint32_t nowMs) {
  return tele.baro_alt_valid && isFresh(nowMs, tele.last_baro_alt_ms);
}

bool hasFreshGpsAltitude(uint32_t nowMs) {
  return tele.gps_alt_valid && isFresh(nowMs, tele.last_gps_alt_ms);
}

bool getBestRawAltitude(float& altitudeM, uint32_t nowMs) {
  if (hasFreshBaroAltitude(nowMs)) {
    altitudeM = tele.alt_mm / 1000.0f;
    return true;
  }
  if (hasFreshGpsAltitude(nowMs)) {
    altitudeM = tele.gps_alt_mm / 1000.0f;
    return true;
  }
  return false;
}

bool sensorsReadyForTracking(uint32_t nowMs) {
  float unusedAltitude = 0.0f;
  return hasFreshGps(nowMs) && getBestRawAltitude(unusedAltitude, nowMs);
}

void formatFixed1(float value, char* out, size_t outLen) {
  const bool negative = value < 0.0f;
  const long scaled = lroundf(fabsf(value) * 10.0f);
  snprintf(out, outLen, "%s%ld.%01ld", negative ? "-" : "", scaled / 10, scaled % 10);
}

void formatE7(int32_t valueE7, char* out, size_t outLen) {
  const bool negative = valueE7 < 0;
  const uint32_t absValue = negative ? (uint32_t)(-(int64_t)valueE7) : (uint32_t)valueE7;
  snprintf(out, outLen, "%s%lu.%07lu",
           negative ? "-" : "",
           (unsigned long)(absValue / 10000000UL),
           (unsigned long)(absValue % 10000000UL));
}

void enqueueTextLine(const char* line) {
  const size_t len = strlen(line);
  if (len == 0 || len > FRAME_MAX_SIZE) return;
  enqueueTelemetry(reinterpret_cast<const uint8_t*>(line), len);
}

void enqueueEventLine(const char* mode, const char* message) {
  char peakText[16];
  formatFixed1(peakAltitudeM, peakText, sizeof(peakText));

  char line[FRAME_MAX_SIZE];
  const int n = snprintf(line, sizeof(line), "EVENT,MODE=%s,msg=%s,peak_m=%s\n",
                         mode, message, peakText);
  if (n > 0 && n < (int)sizeof(line)) {
    enqueueTextLine(line);
  }
}

void logSensorError(uint32_t nowMs) {
  if (static_cast<uint32_t>(nowMs - lastSensorErrorLogMs) < SENSOR_ERROR_LOG_INTERVAL_MS) {
    return;
  }
  lastSensorErrorLogMs = nowMs;
  enqueueEventLine("FLIGHT", "SENSOR ERROR GPS/ALT unavailable - staying in FLIGHT");
}

void resetDescentDetection() {
  descentReadingCount = 0;
  descentHoldStartedMs = 0;
}

void enterTrackingMode(uint32_t nowMs) {
  flightMode = FlightMode::TRACKING;
  resetDescentDetection();
  lastTrackingTxMs = nowMs - TRACKING_TX_INTERVAL_MS;
  enqueueEventLine("APOGEE_DETECT", "APOGEE DETECTED");
  enqueueEventLine("TRACK", "TRACKING MODE ACTIVE (5s packets)");
}

void updateFlightState(float rawAltitudeM, uint32_t nowMs) {
  const float smoothAltitudeM = altitudeFilter.update(rawAltitudeM);
  if (smoothAltitudeM > peakAltitudeM) {
    peakAltitudeM = smoothAltitudeM;
  }

  if (flightMode == FlightMode::TRACKING) {
    return;
  }

  if (flightMode == FlightMode::PRELAUNCH) {
    if (peakAltitudeM > LAUNCH_ARM_ALTITUDE_M) {
      flightMode = FlightMode::FLIGHT;
      enqueueEventLine("FLIGHT", "LAUNCH ARMED");
    } else {
      return;
    }
  }

  if (!sensorsReadyForTracking(nowMs)) {
    if (flightMode == FlightMode::APOGEE_DETECT) {
      flightMode = FlightMode::FLIGHT;
      resetDescentDetection();
    }
    flightMode = FlightMode::FLIGHT;
    logSensorError(nowMs);
    return;
  }

  const bool belowPeakDrop = smoothAltitudeM <= (peakAltitudeM - APOGEE_DROP_M);
  if (!belowPeakDrop) {
    if (flightMode == FlightMode::APOGEE_DETECT) {
      flightMode = FlightMode::FLIGHT;
      resetDescentDetection();
    }
    return;
  }

  if (flightMode != FlightMode::APOGEE_DETECT) {
    flightMode = FlightMode::APOGEE_DETECT;
    descentReadingCount = 1;
    descentHoldStartedMs = nowMs;
  } else if (descentReadingCount < 255) {
    ++descentReadingCount;
  }

  const bool countSatisfied = descentReadingCount >= APOGEE_DESCENT_READING_COUNT;
  const bool holdSatisfied = static_cast<uint32_t>(nowMs - descentHoldStartedMs) >= APOGEE_HOLD_MS;
  if (countSatisfied && holdSatisfied) {
    enterTrackingMode(nowMs);
  }
}

void updateStatusBits(uint32_t nowMs) {
  uint8_t status = 0;
  if (hasFreshGps(nowMs)) status |= 0x01;
  if (hasFreshBaroAltitude(nowMs) || hasFreshGpsAltitude(nowMs)) status |= 0x02;
  status |= (static_cast<uint8_t>(flightMode) & 0x03) << 4;
  tele.status = status;
}

// Packet layout (big-endian):
// version(1), type(1), seq(2), ts(4), lat(4:int32,deg*1e7), lon(4:int32,deg*1e7),
// alt_mm(4:int32), speed_cms(2), heading_cd(2), pitch_cd(2), roll_cd(2),
// yaw_cd(2), battery_mv(2), status(1), crc16(2)
void pack_telemetry_frame(uint8_t* buf, size_t& out_len) {
  size_t off = 0;
  buf[off++] = 1; // version
  buf[off++] = 1; // type: 1=telemetry
  write_u16_be(buf, off, tele.seq);
  write_u32_be(buf, off, tele.ts);
  write_i32_be(buf, off, tele.lat);
  write_i32_be(buf, off, tele.lon);
  write_i32_be(buf, off, tele.alt_mm);
  write_u16_be(buf, off, tele.speed_cms);
  write_u16_be(buf, off, tele.heading_cd);
  write_i16_be(buf, off, tele.pitch_cd);
  write_i16_be(buf, off, tele.roll_cd);
  write_i16_be(buf, off, tele.yaw_cd);
  write_u16_be(buf, off, tele.battery_mv);
  buf[off++] = tele.status;
  // compute CRC over bytes so far
  uint16_t crc = crc16_ccitt(buf, off);
  write_u16_be(buf, off, crc);
  out_len = off;
}

// Helpers to parse numeric ascii and NMEA lat/lon -> scaled integers
long nmea_to_e7(const char* s) {
  // ddmm.mmmm or dddmm.mmmm -> degrees * 1e7
  if (!s || *s == '\0') return 0;
  double val = atof(s);
  double degrees = floor(val / 100.0);
  double minutes = val - (degrees * 100.0);
  double d = degrees + minutes / 60.0;
  return (long)round(d * 1e7);
}

void processTelemetryLineAndPack(const char* line, uint8_t* out_frame, size_t& out_len) {
  // Update tele state from the textual line. Keep existing values when absent.
  const uint32_t nowMs = millis();
  bool newAltitudeSample = false;

  if (strncmp(line, "$IMU,", 5) == 0) {
    // $IMU,ax,ay,az,gx,gy,gz -- we don't directly use accel for attitude,
    // but pack a simple scaled version of the accel.x/accel.y/accel.z as pitch/roll/yaw placeholders.
    float ax=0, ay=0, az=0;
    sscanf(line+5, "%f,%f,%f", &ax, &ay, &az);
    tele.pitch_cd = (int16_t)round(ax * 100.0);
    tele.roll_cd = (int16_t)round(ay * 100.0);
    tele.yaw_cd = (int16_t)round(az * 100.0);
  } else if (strncmp(line, "$ALT,", 5) == 0) {
    double alt = atof(line+5);
    if (isfinite(alt)) {
      tele.alt_mm = (int32_t)round(alt * 1000.0);
      tele.baro_alt_valid = true;
      tele.last_baro_alt_ms = nowMs;
      newAltitudeSample = true;
    }
  } else if (strncmp(line, "$CTRL,", 6) == 0) {
    // $CTRL,seq,craft,mode, ... values
    int seq = 0;
    // read seq
    sscanf(line+6, "%d", &seq);
    tele.seq = (uint16_t)seq;
    // heuristically parse heading from the later comma-separated fields
    // find last two comma-separated floats
    const char* p = strrchr(line, ',');
    if (p) {
      double last = atof(p+1);
      tele.heading_cd = (uint16_t)round(last * 100.0);
    }
  } else if (line[0] == '$') {
    // NMEA: look for GPRMC/GPGGA
    if (strncmp(line+1, "GPRMC", 5) == 0 || strncmp(line+1, "GNRMC", 5) == 0) {
      // naive split
      char buf[128];
      strncpy(buf, line, sizeof(buf)-1);
      buf[sizeof(buf)-1] = '\0';
      char* parts[16] = {0};
      int pc = 0;
      char* tk = strtok(buf, ",");
      while (tk && pc < 16) { parts[pc++] = tk; tk = strtok(NULL, ","); }
      if (pc > 6 && parts[2] && parts[2][0] == 'A') {
        // parts[3]=lat, parts[4]=N/S, parts[5]=lon, parts[6]=E/W
        long lat_e7 = nmea_to_e7(parts[3]);
        long lon_e7 = nmea_to_e7(parts[5]);
        if (parts[4] && parts[4][0] == 'S') lat_e7 = -lat_e7;
        if (parts[6] && parts[6][0] == 'W') lon_e7 = -lon_e7;
        tele.lat = (int32_t)lat_e7;
        tele.lon = (int32_t)lon_e7;
        tele.gps_valid = true;
        tele.last_gps_ms = nowMs;
        if (pc > 7) {
          double spd = atof(parts[7]); // knots
          tele.speed_cms = (uint16_t)round(spd * 0.514444 * 100.0);
        }
      }
    } else if (strncmp(line+1, "GPGGA", 5) == 0 || strncmp(line+1, "GNGGA", 5) == 0) {
      char buf[128];
      strncpy(buf, line, sizeof(buf)-1);
      buf[sizeof(buf)-1] = '\0';
      char* parts[16] = {0};
      int pc = 0;
      char* tk = strtok(buf, ",");
      while (tk && pc < 16) { parts[pc++] = tk; tk = strtok(NULL, ","); }
      if (pc > 9 && parts[6] && atoi(parts[6]) > 0) {
        long lat_e7 = nmea_to_e7(parts[2]);
        long lon_e7 = nmea_to_e7(parts[4]);
        if (parts[3] && parts[3][0] == 'S') lat_e7 = -lat_e7;
        if (parts[5] && parts[5][0] == 'W') lon_e7 = -lon_e7;
        tele.lat = (int32_t)lat_e7;
        tele.lon = (int32_t)lon_e7;
        tele.gps_valid = true;
        tele.last_gps_ms = nowMs;
        if (parts[9] && parts[9][0] != '\0') {
          const double gpsAlt = atof(parts[9]);
          if (isfinite(gpsAlt)) {
            tele.gps_alt_mm = (int32_t)round(gpsAlt * 1000.0);
            tele.gps_alt_valid = true;
            tele.last_gps_alt_ms = nowMs;
            if (!hasFreshBaroAltitude(nowMs)) {
              newAltitudeSample = true;
            }
          }
        }
      }
    }
  }

  if (newAltitudeSample) {
    float rawAltitudeM = 0.0f;
    if (getBestRawAltitude(rawAltitudeM, nowMs)) {
      updateFlightState(rawAltitudeM, nowMs);
    }
  }

  // update timestamp/status and pack only before TRACKING. In TRACKING, normal
  // flight telemetry is intentionally suppressed by updateTrackingTransmission().
  tele.ts = (uint32_t)(nowMs / 1000UL);
  updateStatusBits(nowMs);
  if (flightMode == FlightMode::TRACKING) {
    out_len = 0;
    return;
  }

  uint8_t frame[FRAME_MAX_SIZE];
  size_t frame_len = 0;
  pack_telemetry_frame(frame, frame_len);
  if (frame_len > 0 && frame_len <= FRAME_MAX_SIZE) {
    memcpy(out_frame, frame, frame_len);
    out_len = frame_len;
  } else {
    out_len = 0;
  }
}

void enqueueTrackingPacket(uint32_t nowMs) {
  if (!sensorsReadyForTracking(nowMs)) {
    logSensorError(nowMs);
    return;
  }

  float altitudeM = altitudeFilter.initialized ? altitudeFilter.value_m : 0.0f;
  if (!altitudeFilter.initialized) {
    float rawAltitudeM = 0.0f;
    if (getBestRawAltitude(rawAltitudeM, nowMs)) {
      altitudeM = rawAltitudeM;
    }
  }

  char latText[16];
  char lonText[17];
  char altText[16];
  formatE7(tele.lat, latText, sizeof(latText));
  formatE7(tele.lon, lonText, sizeof(lonText));
  formatFixed1(altitudeM, altText, sizeof(altText));

  char line[FRAME_MAX_SIZE];
  const int n = snprintf(line, sizeof(line),
                         "TRACK,MODE=TRACK,ts_ms=%lu,lat=%s,lon=%s,alt_m=%s\n",
                         (unsigned long)nowMs, latText, lonText, altText);
  if (n > 0 && n < (int)sizeof(line)) {
    enqueueTextLine(line);
    ++trackingSeq;
  }
}

void updateTrackingTransmission(uint32_t nowMs) {
  if (flightMode != FlightMode::TRACKING) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - lastTrackingTxMs) < TRACKING_TX_INTERVAL_MS) {
    return;
  }
  lastTrackingTxMs = nowMs;
  enqueueTrackingPacket(nowMs);
}

void updateSensorFailsafe(uint32_t nowMs) {
  if (flightMode == FlightMode::TRACKING || flightMode == FlightMode::PRELAUNCH) {
    return;
  }
  if (!sensorsReadyForTracking(nowMs)) {
    if (flightMode == FlightMode::APOGEE_DETECT) {
      flightMode = FlightMode::FLIGHT;
      resetDescentDetection();
    }
    logSensorError(nowMs);
  }
}

void injectSyntheticTelemetry() {
  if (!ENABLE_SYNTHETIC_ALTITUDE_TEST) {
    return;
  }

  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - lastSyntheticSampleMs) < SYNTHETIC_SAMPLE_MS) {
    return;
  }
  lastSyntheticSampleMs = nowMs;

  static const float profileM[] = {
      0.0f, 4.0f, 10.0f, 18.0f, 24.0f, 35.0f, 50.0f, 68.0f, 84.0f,
      98.0f, 110.0f, 116.0f, 118.0f, 116.0f, 112.0f, 106.0f, 101.0f,
      96.0f, 91.0f, 86.0f, 82.0f, 78.0f, 74.0f
  };
  static size_t index = 0;
  const float altM = profileM[index];
  if (index + 1 < sizeof(profileM) / sizeof(profileM[0])) {
    ++index;
  }

  char gpsLine[96];
  snprintf(gpsLine, sizeof(gpsLine),
           "$GPGGA,120000.00,3746.4940,N,12225.1640,W,1,08,1.0,%.1f,M,0.0,M,,",
           altM);
  uint8_t frame[FRAME_MAX_SIZE];
  size_t frameLen = 0;
  processTelemetryLineAndPack(gpsLine, frame, frameLen);
  if (frameLen > 0) enqueueTelemetry(frame, frameLen);

  char altLine[32];
  snprintf(altLine, sizeof(altLine), "$ALT,%.2f", altM);
  processTelemetryLineAndPack(altLine, frame, frameLen);
  if (frameLen > 0) enqueueTelemetry(frame, frameLen);
}

void haltForRadioError();

void handleCompletedRadioOperation() {
  if (radioTransmitting) {
    const int16_t state = radio.finishTransmit();
    if (state == RADIOLIB_ERR_NONE) {
      pulseTxLed();
    }
    activeTransmissionLen = 0;
  }
  radioTransmitting = false;
}

void startQueuedTransmission() {
  if (radioTransmitting || queueCount == 0) return;
  size_t len = 0;
  if (!dequeueTelemetry(activeTransmission, len)) return;
  activeTransmissionLen = len;
  const int16_t state = radio.startTransmit(activeTransmission, activeTransmissionLen);
  if (state == RADIOLIB_ERR_NONE) {
    radioTransmitting = true;
  } else {
    activeTransmissionLen = 0;
    radioTransmitting = false;
  }
}

void haltForRadioError() {
  // Do not report the error on Serial: UART0 is the ESP32D protocol link.
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(115200, SERIAL_8N1, 44, -1);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(LORA_FEM_EN, OUTPUT);
  digitalWrite(LORA_FEM_EN, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  const int16_t state = radio.begin(
      LORA_FREQUENCY_MHZ,
      LORA_BANDWIDTH_KHZ,
      LORA_SPREADING_FACTOR,
      LORA_CODING_RATE,
      LORA_SYNC_WORD,
      LORA_TX_POWER_DBM,
      LORA_PREAMBLE_SYMBOLS
  );
  if (state != RADIOLIB_ERR_NONE) {
    haltForRadioError();
  }

  radio.setCRC(true);
  radio.explicitHeader();
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(setRadioFlag);
}

void loop() {
  injectSyntheticTelemetry();
  readEsp32Telemetry();
  updateTxLed();

  const uint32_t nowMs = millis();
  updateSensorFailsafe(nowMs);
  updateTrackingTransmission(nowMs);

  if (takeRadioFlag()) {
    handleCompletedRadioOperation();
  }

  startQueuedTransmission();
}
