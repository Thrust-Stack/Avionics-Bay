/*
 * I2C_Scanner — ESP32 bus diagnostic for the Avionics Sensor Bridge.
 *
 * Uses the SAME I2C pins as Esp32_sensor_bridge.ino:
 *   SDA -> GPIO 23
 *   SCL -> GPIO 32
 * (GPIO 18/19/21/22 are damaged on this board — do not use.)
 *
 * Flash this, open Serial Monitor at 115200. It scans every 7-bit address
 * once per second and prints whatever answers.
 *
 * Expected on a healthy bus:
 *   0x69  -> MPU6050 (AD0 high on this breakout)
 *   0x46  -> BMP585
 *
 * If it finds NOTHING: the bus is dead — check 3.3V, GND, SDA/SCL wiring,
 * and pull-ups before suspecting either sensor.
 */

#include <Wire.h>

#define I2C_SDA_PIN  23
#define I2C_SCL_PIN  32

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);  // 100 kHz — slow/robust for diagnosis
  Serial.println();
  Serial.println("I2C scanner — SDA=23, SCL=32");
  Serial.println("Expecting 0x69 (MPU6050) and 0x46 (BMP585).");
}

void loop() {
  int found = 0;
  Serial.println("Scanning...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  device found at 0x%02X\n", addr);
      found++;
    } else if (error == 4) {
      Serial.printf("  unknown error at 0x%02X\n", addr);
    }
  }
  if (found == 0) {
    Serial.println("  NONE found — bus is dead (check power / wiring / pull-ups).");
  } else {
    Serial.printf("  %d device(s) found.\n", found);
  }
  Serial.println();
  delay(1000);
}
