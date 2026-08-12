/*
 * Heltec WiFi LoRa 32 V4 - ground-station telemetry receiver
 *
 * Pairing:
 *   This sketch is the ground-side match for:
 *     Communication/TransmitterHeltec/TransmitterHeltec.ino
 *
 *   Both Heltec V4 radios must use the same SX1262 pins and LoRa settings
 *   below. The avionics-side transmitter forwards newline-delimited telemetry
 *   packets from the ESP32D over LoRa. This receiver writes each LoRa payload
 *   to the computer over USB serial.
 *
 * Serial protocol:
 *   - Telemetry payloads are forwarded exactly as received when they already
 *     end in '\n'.
 *   - If a payload has no line ending, this sketch appends one newline so the
 *     Python listener can read it as a line.
 *   - Diagnostic messages are prefixed with "# " so they are easy to filter.
 *
 * Build this ground-side sketch with USB CDC On Boot enabled so Serial is the
 * USB connection to the computer. This sketch does not use Meshtastic or any
 * other protocol stack.
 *
 * Requires RadioLib. Select the exact Heltec WiFi LoRa 32 V4 board/revision
 * and attach the correct regional LoRa antenna before powering the radio.
 */

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// Heltec V4 SX1262 and GC1109 front-end pins. These must match the transmitter.
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
constexpr uint8_t LORA_SPREADING_FACTOR = 8;
constexpr uint8_t LORA_CODING_RATE = 5;   // 5 means coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12;  // private point-to-point link
constexpr int8_t LORA_TX_POWER_DBM = 14;
constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;
constexpr uint32_t RX_LED_PULSE_MS = 25;
constexpr uint32_t SERIAL_CONNECT_WAIT_MS = 5000;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool radioPacketReady = false;
bool rxLedPulseActive = false;
uint32_t rxLedOffAt = 0;

void IRAM_ATTR setRadioFlag() {
  radioPacketReady = true;
}

bool takeRadioFlag() {
  noInterrupts();
  const bool wasSet = radioPacketReady;
  radioPacketReady = false;
  interrupts();
  return wasSet;
}

void pulseRxLed() {
  digitalWrite(LED_BUILTIN, HIGH);
  rxLedPulseActive = true;
  rxLedOffAt = millis() + RX_LED_PULSE_MS;
}

void updateRxLed() {
  if (rxLedPulseActive && static_cast<int32_t>(millis() - rxLedOffAt) >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    rxLedPulseActive = false;
  }
}

void haltForRadioError(const char* message, int16_t state) {
  Serial.print("# ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(state);

  while (true) {
    delay(1000);
  }
}

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

bool isPrintableAsciiPayload(const uint8_t* buf, size_t len) {
  if (len == 0) return false;
  for (size_t i = 0; i < len; ++i) {
    const uint8_t c = buf[i];
    if (c == '\r' || c == '\n' || c == '\t') continue;
    if (c < 32 || c > 126) return false;
  }
  return true;
}

void printAsciiPayload(const uint8_t* buf, size_t len) {
  Serial.write(buf, len);
  if (len == 0 || (buf[len - 1] != '\n' && buf[len - 1] != '\r')) {
    Serial.write('\n');
  }
}

void printDecodedBin(const uint8_t* buf, size_t len) {
  // Minimal defensive decode matching the transmitter pack_telemetry_frame layout.
  if (len < 4) return;
  size_t off = 0;
  uint8_t version = buf[off++];
  uint8_t type = buf[off++];
  if (len < 35) return; // minimal expected size
  auto rd_u16 = [&](uint16_t& out) {
    out = (uint16_t)buf[off++] << 8;
    out |= (uint16_t)buf[off++];
  };
  auto rd_u32 = [&](uint32_t& out) {
    out = (uint32_t)buf[off++] << 24;
    out |= (uint32_t)buf[off++] << 16;
    out |= (uint32_t)buf[off++] << 8;
    out |= (uint32_t)buf[off++];
  };
  auto rd_i32 = [&](int32_t& out) {
    uint32_t v; rd_u32(v); out = (int32_t)v; };
  auto rd_i16 = [&](int16_t& out) {
    out = (int16_t)((uint16_t)buf[off++] << 8);
    out |= (uint16_t)buf[off++];
  };

  uint16_t seq; rd_u16(seq);
  uint32_t ts; rd_u32(ts);
  int32_t lat; rd_i32(lat);
  int32_t lon; rd_i32(lon);
  int32_t alt_mm; rd_i32(alt_mm);
  uint16_t speed_cms; rd_u16(speed_cms);
  uint16_t heading_cd; rd_u16(heading_cd);
  int16_t pitch_cd; rd_i16(pitch_cd);
  int16_t roll_cd; rd_i16(roll_cd);
  int16_t yaw_cd; rd_i16(yaw_cd);
  uint16_t battery_mv; rd_u16(battery_mv);
  uint8_t status = buf[off++];
  uint16_t packet_crc = 0;
  rd_u16(packet_crc);

  // Verify CRC over header..status
  uint16_t calc = crc16_ccitt(buf, off - 2);
  if (calc != packet_crc) {
    // silently ignore malformed packets
    return;
  }

  // Print compact machine-parseable line: BIN,key=value,...\n
  Serial.print("BIN,");
  Serial.print("ver="); Serial.print(version); Serial.print(',');
  Serial.print("type="); Serial.print(type); Serial.print(',');
  Serial.print("seq="); Serial.print(seq); Serial.print(',');
  Serial.print("ts="); Serial.print(ts); Serial.print(',');
  Serial.print("lat="); Serial.print(lat); Serial.print(',');
  Serial.print("lon="); Serial.print(lon); Serial.print(',');
  Serial.print("alt_mm="); Serial.print(alt_mm); Serial.print(',');
  Serial.print("speed_cms="); Serial.print(speed_cms); Serial.print(',');
  Serial.print("heading_cd="); Serial.print(heading_cd); Serial.print(',');
  Serial.print("pitch_cd="); Serial.print(pitch_cd); Serial.print(',');
  Serial.print("roll_cd="); Serial.print(roll_cd); Serial.print(',');
  Serial.print("yaw_cd="); Serial.print(yaw_cd); Serial.print(',');
  Serial.print("bat_mv="); Serial.print(battery_mv); Serial.print(',');
  Serial.print("status=0x");
  if (status < 16) Serial.print('0');
  Serial.print(status, HEX);
  Serial.write('\n');
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < SERIAL_CONNECT_WAIT_MS) {
    delay(10);
  }
  delay(250);

  Serial.println("# Heltec V4 telemetry receiver booting");

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
    haltForRadioError("radio.begin failed", state);
  }

  radio.setCRC(true);
  radio.explicitHeader();
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(setRadioFlag);

  const int16_t receiveState = radio.startReceive();
  if (receiveState != RADIOLIB_ERR_NONE) {
    haltForRadioError("radio.startReceive failed", receiveState);
  }

  Serial.println("# Heltec V4 telemetry receiver ready");
}

void loop() {
  updateRxLed();

  if (!takeRadioFlag()) {
    return;
  }

  uint8_t buf[128];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf)) {
    len = sizeof(buf);
  }
  const int16_t state = radio.readData(buf, len);

  if (state == RADIOLIB_ERR_NONE) {
    if (isPrintableAsciiPayload(buf, len)) {
      printAsciiPayload(buf, len);
    } else {
      printDecodedBin(buf, len);
    }
    pulseRxLed();
  } else if (state != RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.print("# radio.readData failed: ");
    Serial.println(state);
  }

  const int16_t receiveState = radio.startReceive();
  if (receiveState != RADIOLIB_ERR_NONE) {
    haltForRadioError("radio.startReceive failed", receiveState);
  }
}
