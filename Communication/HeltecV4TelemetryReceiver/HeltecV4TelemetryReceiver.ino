/*
 * Heltec WiFi LoRa 32 V4 - ground-station telemetry receiver
 *
 * Pairing:
 *   This sketch is the ground-side match for:
 *     Communication/HeltecV4TelemetryTransmitter/HeltecV4TelemetryTransmitter.ino
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
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;   // 5 means coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12;  // private point-to-point link
constexpr int8_t LORA_TX_POWER_DBM = 14;
constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool radioPacketReady = false;

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

void haltForRadioError(const char* message, int16_t state) {
  Serial.print("# ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(state);

  while (true) {
    delay(1000);
  }
}

void printPayloadLine(const String& payload) {
  Serial.print(payload);

  if (!payload.endsWith("\n")) {
    Serial.write('\n');
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("# Heltec V4 telemetry receiver booting");

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
  if (!takeRadioFlag()) {
    return;
  }

  String payload;
  const int16_t state = radio.readData(payload);

  if (state == RADIOLIB_ERR_NONE) {
    printPayloadLine(payload);
  } else if (state != RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.print("# radio.readData failed: ");
    Serial.println(state);
  }

  const int16_t receiveState = radio.startReceive();
  if (receiveState != RADIOLIB_ERR_NONE) {
    haltForRadioError("radio.startReceive failed", receiveState);
  }
}
