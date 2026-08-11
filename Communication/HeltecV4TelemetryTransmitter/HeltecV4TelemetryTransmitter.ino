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
// SX1262 LoRa packets are limited to 255 bytes. Leave room for the newline.
constexpr size_t UART_LINE_CAPACITY = 220;
constexpr size_t TELEMETRY_QUEUE_DEPTH = 8;
constexpr uint32_t TX_LED_PULSE_MS = 25;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

char uartLine[UART_LINE_CAPACITY + 1] = {};
size_t uartLineLength = 0;
bool discardOversizeLine = false;

String telemetryQueue[TELEMETRY_QUEUE_DEPTH];
size_t queueHead = 0;
size_t queueTail = 0;
size_t queueCount = 0;
String activeTransmission;

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

void enqueueTelemetry(const String& packet) {
  // If LoRa cannot keep up with the UART stream, retain the newest complete
  // packets instead of allowing an unbounded queue to exhaust memory.
  if (queueCount == TELEMETRY_QUEUE_DEPTH) {
    telemetryQueue[queueHead] = "";
    queueHead = (queueHead + 1) % TELEMETRY_QUEUE_DEPTH;
    --queueCount;
  }

  telemetryQueue[queueTail] = packet;
  queueTail = (queueTail + 1) % TELEMETRY_QUEUE_DEPTH;
  ++queueCount;
}

bool dequeueTelemetry(String& packet) {
  if (queueCount == 0) {
    return false;
  }

  packet = telemetryQueue[queueHead];
  telemetryQueue[queueHead] = "";
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
          String packet(uartLine);
          packet += '\n';
          enqueueTelemetry(packet);
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

void haltForRadioError();

void handleCompletedRadioOperation() {
  if (radioTransmitting) {
    const int16_t state = radio.finishTransmit();
    if (state == RADIOLIB_ERR_NONE) {
      pulseTxLed();
    }
    activeTransmission = "";
  }
  radioTransmitting = false;
}

void startQueuedTransmission() {
  if (radioTransmitting || queueCount == 0) {
    return;
  }

  if (!dequeueTelemetry(activeTransmission)) {
    return;
  }

  const int16_t state = radio.startTransmit(activeTransmission);
  if (state == RADIOLIB_ERR_NONE) {
    radioTransmitting = true;
  } else {
    activeTransmission = "";
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
  readEsp32Telemetry();
  updateTxLed();

  if (takeRadioFlag()) {
    handleCompletedRadioOperation();
  }

  startQueuedTransmission();
}
