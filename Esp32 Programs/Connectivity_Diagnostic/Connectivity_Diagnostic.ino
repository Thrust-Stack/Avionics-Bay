#include <SPI.h>
#include <SD.h>

#define HELTEC_RX_PIN  18
#define HELTEC_TX_PIN  19
#define HELTEC_BAUD    115200

#define SD_SCK_PIN     14  // ADA254 CLK
#define SD_MISO_PIN    27  // ADA254 DO
#define SD_MOSI_PIN    13  // ADA254 DI
#define SD_CS_PIN      33  // ADA254 CS

SPIClass sdSPI(HSPI);
String heltecRxLine;
unsigned long lastHeltecTx = 0;
unsigned long heltecTxCount = 0;

void testSdCard() {
  Serial.println("SD diagnostic -- ADA254 HSPI CLK=14 DO=27 DI=13 CS=33");

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

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

  File f = SD.open("/connectivity_diag.txt", FILE_WRITE);
  if (!f) {
    Serial.println("[ERR] Could not open /connectivity_diag.txt for writing");
    return;
  }
  f.printf("connectivity diagnostic write ok at %lu ms\n", millis());
  f.close();
  Serial.println("[OK] Wrote /connectivity_diag.txt");

  f = SD.open("/connectivity_diag.txt", FILE_READ);
  if (!f) {
    Serial.println("[ERR] Could not reopen /connectivity_diag.txt");
    return;
  }
  Serial.print("[OK] Readback: ");
  while (f.available()) Serial.write(f.read());
  f.close();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(HELTEC_BAUD, SERIAL_8N1, HELTEC_RX_PIN, HELTEC_TX_PIN);
  delay(500);

  Serial.println("ESP32 connectivity diagnostic");
  Serial.println("Heltec UART -- RX=18 TX=19 baud=115200");
  testSdCard();
  Serial.println("Sending one Heltec UART diagnostic packet per second.");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      heltecRxLine.trim();
      if (heltecRxLine.length() > 0) {
        Serial.print("[OK] Heltec UART RX: ");
        Serial.println(heltecRxLine);
      }
      heltecRxLine = "";
    } else {
      heltecRxLine += c;
    }
  }

  if (millis() - lastHeltecTx >= 1000) {
    lastHeltecTx = millis();
    heltecTxCount++;
    Serial1.printf("$DIAG,%lu,%lu\n", millis(), heltecTxCount);
    Serial.printf("[TX] Heltec UART $DIAG count=%lu\n", heltecTxCount);
  }
}
