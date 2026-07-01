// Heltec WiFi LoRa 32 V4 - avionics telemetry transmitter
// Requires the RadioLib library (Arduino Library Manager: "RadioLib").
// Select the exact Heltec WiFi LoRa 32 V4 board/revision in Arduino IDE.
// Attach the correct regional LoRa antenna before powering the transmitter.

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

// Heltec V4 SX1262 and GC1109 front-end pins.
constexpr int LORA_SCK = 9;
constexpr int LORA_MISO = 11;
constexpr int LORA_MOSI = 10;
constexpr int LORA_NSS = 8;
constexpr int LORA_DIO1 = 14;
constexpr int LORA_RST = 12;
constexpr int LORA_BUSY = 13;
constexpr int LORA_FEM_EN = 2;

// Both boards must use identical radio settings. 915 MHz is for the US ISM
// band; change this to a legal frequency for your region and hardware variant.
constexpr float LORA_FREQUENCY_MHZ = 915.0;
constexpr float LORA_BANDWIDTH_KHZ = 125.0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;       // 5 means coding rate 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12;      // private point-to-point link
constexpr int8_t LORA_TX_POWER_DBM = 14;      // conservative, check local law
constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;
constexpr uint32_t SEND_INTERVAL_MS = 500;     // target: 2 packets/second
constexpr uint8_t PACKET_START = 0xA5;

// SF7/BW125 is a range/speed compromise. This 20-byte packet has airtime of
// roughly tens of milliseconds, so 2 Hz is comfortable. Higher SF improves
// sensitivity/range but increases airtime sharply and lowers practical rate.

struct __attribute__((packed)) TelemetryPacket {
  uint8_t start;
  uint32_t packetId;
  uint32_t timestampMs;
  int32_t altitudeCm;
  uint16_t batteryMv;
  int16_t temperatureCentiC;
  uint8_t gpsStatus;  // 0=no fix, 1=2D fix, 2=3D fix
  uint16_t checksum;
};

static_assert(sizeof(TelemetryPacket) == 20, "Telemetry packet layout changed");

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
uint32_t nextPacketId = 0;
uint32_t lastSendMs = 0;

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void fillSensorData(TelemetryPacket &packet) {
  // Replace these example values with real sensor reads. Integer scaling keeps
  // the packet fixed-size and avoids sending architecture-dependent floats.
  packet.altitudeCm = 12345;          // 123.45 m
  packet.batteryMv = 3890;            // 3.890 V
  packet.temperatureCentiC = 2375;    // 23.75 C
  packet.gpsStatus = 2;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LORA_FEM_EN, OUTPUT);
  digitalWrite(LORA_FEM_EN, HIGH);  // Enable the V4 external RF front end.
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int16_t state = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ,
                              LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                              LORA_SYNC_WORD, LORA_TX_POWER_DBM,
                              LORA_PREAMBLE_SYMBOLS);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio initialization failed: %d\n", state);
    while (true) delay(1000);
  }
  radio.setCRC(true);            // SX1262 hardware CRC, in addition to packet CRC.
  radio.explicitHeader();
  radio.setDio2AsRfSwitch(true);
  Serial.println("Heltec V4 telemetry transmitter ready");
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastSendMs) < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  TelemetryPacket packet{};
  packet.start = PACKET_START;
  packet.packetId = nextPacketId++;
  packet.timestampMs = now;
  fillSensorData(packet);
  packet.checksum = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet),
                              sizeof(packet) - sizeof(packet.checksum));

  const int16_t state = radio.transmit(reinterpret_cast<uint8_t *>(&packet),
                                       sizeof(packet));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("TX id=%lu time=%lu alt=%.2fm batt=%.3fV temp=%.2fC gps=%u\n",
                  static_cast<unsigned long>(packet.packetId),
                  static_cast<unsigned long>(packet.timestampMs),
                  packet.altitudeCm / 100.0, packet.batteryMv / 1000.0,
                  packet.temperatureCentiC / 100.0, packet.gpsStatus);
  } else {
    Serial.printf("Transmit failed for id=%lu: %d\n",
                  static_cast<unsigned long>(packet.packetId), state);
  }
}
