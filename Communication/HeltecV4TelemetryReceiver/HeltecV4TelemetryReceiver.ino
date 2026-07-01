// Heltec WiFi LoRa 32 V4 - ground telemetry receiver
// Requires the RadioLib library. Radio settings and packet layout must match
// HeltecV4TelemetryTransmitter.ino exactly.

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

constexpr int LORA_SCK = 9;
constexpr int LORA_MISO = 11;
constexpr int LORA_MOSI = 10;
constexpr int LORA_NSS = 8;
constexpr int LORA_DIO1 = 14;
constexpr int LORA_RST = 12;
constexpr int LORA_BUSY = 13;
constexpr int LORA_FEM_EN = 2;

constexpr float LORA_FREQUENCY_MHZ = 915.0;  // US default; obey local rules
constexpr float LORA_BANDWIDTH_KHZ = 125.0;
constexpr uint8_t LORA_SPREADING_FACTOR = 7;
constexpr uint8_t LORA_CODING_RATE = 5;
constexpr uint8_t LORA_SYNC_WORD = 0x12;
constexpr int8_t LORA_TX_POWER_DBM = 14;  // required by begin(), unused in RX
constexpr uint16_t LORA_PREAMBLE_SYMBOLS = 8;
constexpr uint8_t PACKET_START = 0xA5;

struct __attribute__((packed)) TelemetryPacket {
  uint8_t start;
  uint32_t packetId;
  uint32_t timestampMs;
  int32_t altitudeCm;
  uint16_t batteryMv;
  int16_t temperatureCentiC;
  uint8_t gpsStatus;
  uint16_t checksum;
};

static_assert(sizeof(TelemetryPacket) == 20, "Telemetry packet layout changed");

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
bool havePreviousPacket = false;
uint32_t previousPacketId = 0;

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

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LORA_FEM_EN, OUTPUT);
  digitalWrite(LORA_FEM_EN, HIGH);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int16_t state = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ,
                              LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                              LORA_SYNC_WORD, LORA_TX_POWER_DBM,
                              LORA_PREAMBLE_SYMBOLS);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio initialization failed: %d\n", state);
    while (true) delay(1000);
  }
  radio.setCRC(true);
  radio.explicitHeader();
  radio.setDio2AsRfSwitch(true);
  Serial.println("Heltec V4 telemetry receiver ready");
  Serial.println("id,time_ms,altitude_m,battery_v,temp_c,gps,rssi_dbm,snr_db,lost");
}

void loop() {
  TelemetryPacket packet{};
  // Blocking receive keeps this simple and reliable; RadioLib returns after a
  // valid radio packet or an error. Serial output is directly loggable as CSV.
  const int16_t state = radio.receive(reinterpret_cast<uint8_t *>(&packet),
                                      sizeof(packet));
  if (state != RADIOLIB_ERR_NONE) {
    if (state != RADIOLIB_ERR_RX_TIMEOUT && state != RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.printf("Receive error: %d\n", state);
    }
    return;
  }

  const uint16_t expectedChecksum = crc16Ccitt(
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet) - sizeof(packet.checksum));
  if (packet.start != PACKET_START || packet.checksum != expectedChecksum) {
    Serial.println("Rejected packet: bad start byte or packet checksum");
    return;
  }

  uint32_t lost = 0;
  if (havePreviousPacket && packet.packetId > previousPacketId + 1) {
    lost = packet.packetId - previousPacketId - 1;
  }
  previousPacketId = packet.packetId;
  havePreviousPacket = true;

  Serial.printf("%lu,%lu,%.2f,%.3f,%.2f,%u,%.1f,%.1f,%lu\n",
                static_cast<unsigned long>(packet.packetId),
                static_cast<unsigned long>(packet.timestampMs),
                packet.altitudeCm / 100.0, packet.batteryMv / 1000.0,
                packet.temperatureCentiC / 100.0, packet.gpsStatus,
                radio.getRSSI(), radio.getSNR(), static_cast<unsigned long>(lost));
}
