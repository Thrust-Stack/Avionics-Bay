/*
 * Minimal ESP32 boot/runtime test.
 * Purpose: verify that the board can run a tiny app without brownout resets.
 */

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

unsigned long lastPrintMs = 0;
bool ledState = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("Serial_Boot_Test running");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    Serial.printf("alive ms=%lu\n", now);
  }
}
