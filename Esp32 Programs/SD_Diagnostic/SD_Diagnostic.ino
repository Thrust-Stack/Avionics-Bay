#include <SPI.h>
#include <SD.h>

#define SD_SCK_PIN   14  // ADA254 CLK
#define SD_MISO_PIN  27  // ADA254 DO
#define SD_MOSI_PIN  13  // ADA254 DI
#define SD_CS_PIN    33  // ADA254 CS

SPIClass sdSPI(HSPI);

static uint8_t spiTx(uint8_t value) {
  return sdSPI.transfer(value);
}

static uint8_t sdCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
  digitalWrite(SD_CS_PIN, LOW);
  spiTx(0xFF);
  spiTx(0x40 | cmd);
  spiTx((uint8_t)(arg >> 24));
  spiTx((uint8_t)(arg >> 16));
  spiTx((uint8_t)(arg >> 8));
  spiTx((uint8_t)arg);
  spiTx(crc);

  for (int i = 0; i < 10; i++) {
    uint8_t r = spiTx(0xFF);
    if ((r & 0x80) == 0) return r;
  }
  return 0xFF;
}

static void endCommand() {
  digitalWrite(SD_CS_PIN, HIGH);
  spiTx(0xFF);
}

static void rawProbe() {
  Serial.println("Raw SPI probe:");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(SD_MISO_PIN, INPUT);
  Serial.printf("  MISO floating/read level: %d\n", digitalRead(SD_MISO_PIN));
  pinMode(SD_MISO_PIN, INPUT_PULLUP);
  delay(5);
  Serial.printf("  MISO with ESP32 pullup: %d\n", digitalRead(SD_MISO_PIN));
  pinMode(SD_MISO_PIN, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("  MISO with ESP32 pulldown: %d\n", digitalRead(SD_MISO_PIN));
  pinMode(SD_MISO_PIN, INPUT_PULLUP);

  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdSPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

  digitalWrite(SD_CS_PIN, HIGH);
  for (int i = 0; i < 10; i++) spiTx(0xFF);  // 80 clocks with CS high

  uint8_t r0 = sdCommand(0, 0, 0x95);
  Serial.printf("  CMD0 response: 0x%02X %s\n", r0, r0 == 0x01 ? "(card entered idle)" : "");
  endCommand();

  uint8_t r8 = sdCommand(8, 0x000001AA, 0x87);
  Serial.printf("  CMD8 response: 0x%02X\n", r8);
  if (r8 != 0xFF) {
    Serial.print("  CMD8 data:");
    for (int i = 0; i < 4; i++) Serial.printf(" 0x%02X", spiTx(0xFF));
    Serial.println();
  }
  endCommand();

  uint8_t last = 0xFF;
  for (int i = 0; i < 20; i++) {
    uint8_t r55 = sdCommand(55, 0, 0x01);
    endCommand();
    last = sdCommand(41, 0x40000000, 0x01);
    endCommand();
    if (last == 0x00) break;
    delay(20);
    if (i == 0) Serial.printf("  CMD55 first response: 0x%02X\n", r55);
  }
  Serial.printf("  ACMD41 final response: 0x%02X\n", last);

  uint8_t r58 = sdCommand(58, 0, 0x01);
  Serial.printf("  CMD58 response: 0x%02X\n", r58);
  if (r58 != 0xFF) {
    Serial.print("  OCR:");
    for (int i = 0; i < 4; i++) Serial.printf(" 0x%02X", spiTx(0xFF));
    Serial.println();
  }
  endCommand();

  sdSPI.endTransaction();
  digitalWrite(SD_CS_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("SD diagnostic -- ADA254 HSPI CLK=14 DO=27 DI=13 CS=33");

  rawProbe();

  Serial.println("Arduino SD.begin probe:");
  if (!SD.begin(SD_CS_PIN, sdSPI, 400000)) {
    Serial.println("[ERR] SD.begin failed at 400 kHz");
    return;
  }

  uint8_t type = SD.cardType();
  if (type == CARD_NONE) {
    Serial.println("[ERR] No SD card detected");
    return;
  }

  Serial.print("[OK] Card type: ");
  if (type == CARD_MMC) Serial.println("MMC");
  else if (type == CARD_SD) Serial.println("SDSC");
  else if (type == CARD_SDHC) Serial.println("SDHC/SDXC");
  else Serial.println("UNKNOWN");

  Serial.printf("[OK] Card size: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  File f = SD.open("/sd_diag.txt", FILE_WRITE);
  if (!f) {
    Serial.println("[ERR] Could not open /sd_diag.txt for writing");
    return;
  }
  f.println("sd diagnostic write ok");
  f.close();
  Serial.println("[OK] Wrote /sd_diag.txt");

  f = SD.open("/sd_diag.txt", FILE_READ);
  if (!f) {
    Serial.println("[ERR] Could not reopen /sd_diag.txt");
    return;
  }
  Serial.print("[OK] Readback: ");
  while (f.available()) Serial.write(f.read());
  f.close();
}

void loop() {
  delay(1000);
}
