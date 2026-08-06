/*
 * MPU_Diagnostic - raw I2C test for the GRCT MPU on ESP32 GPIO23/GPIO32.
 *
 * This does not use FastIMU. It reads the chip identity and raw accel/gyro
 * registers directly so driver-class issues can be separated from wiring.
 */

#include <Wire.h>

#define I2C_SDA_PIN 23
#define I2C_SCL_PIN 32
#define MPU_ADDR 0x68

static uint8_t readRegRepeatedStart(uint8_t reg, uint8_t *txError, uint8_t *rxCount) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  *txError = Wire.endTransmission(false);
  if (*txError != 0) {
    return 0xFF;
  }
  *rxCount = Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (*rxCount != 1) {
    return 0xFF;
  }
  return Wire.read();
}

static uint8_t readRegStopStart(uint8_t reg, uint8_t *txError, uint8_t *rxCount) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  *txError = Wire.endTransmission(true);
  if (*txError != 0) {
    return 0xFF;
  }
  delayMicroseconds(100);
  *rxCount = Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (*rxCount != 1) {
    return 0xFF;
  }
  return Wire.read();
}

static void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();
  Serial.printf("write 0x%02X=0x%02X txError=%u\n", reg, value, error);
}

static int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) {
    return 0;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

static void scanBus() {
  int found = 0;
  Serial.println("scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  ACK at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  no I2C ACKs");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(50000);

  Serial.println();
  Serial.println("MPU_Diagnostic -- SDA=23 SCL=32 ADDR=0x68");
  scanBus();

  uint8_t txError = 0;
  uint8_t rxCount = 0;
  uint8_t who = readRegRepeatedStart(0x75, &txError, &rxCount);
  Serial.printf("WHO_AM_I repeated-start: value=0x%02X txError=%u rxCount=%u\n", who, txError, rxCount);
  who = readRegStopStart(0x75, &txError, &rxCount);
  Serial.printf("WHO_AM_I stop/start:      value=0x%02X txError=%u rxCount=%u\n", who, txError, rxCount);
  if (who == 0x68) Serial.println("Detected identity: MPU6050-compatible");
  else if (who == 0x70) Serial.println("Detected identity: MPU6500-compatible");
  else if (who == 0x71) Serial.println("Detected identity: MPU9250-compatible");
  else if (who == 0x73) Serial.println("Detected identity: MPU9255-compatible");
  else Serial.println("Detected identity: unknown or read failed");

  writeReg(0x6B, 0x00);  // PWR_MGMT_1: wake from sleep, internal clock
  delay(100);
  writeReg(0x1B, 0x00);  // GYRO_CONFIG: +/-250 dps
  writeReg(0x1C, 0x00);  // ACCEL_CONFIG: +/-2 g
  uint8_t power = readRegStopStart(0x6B, &txError, &rxCount);
  Serial.printf("PWR_MGMT_1 stop/start: value=0x%02X txError=%u rxCount=%u\n", power, txError, rxCount);
  Serial.println("raw_ax raw_ay raw_az raw_gx raw_gy raw_gz");
}

void loop() {
  int16_t ax = read16(0x3B);
  int16_t ay = read16(0x3D);
  int16_t az = read16(0x3F);
  int16_t gx = read16(0x43);
  int16_t gy = read16(0x45);
  int16_t gz = read16(0x47);
  Serial.printf("%6d %6d %6d %6d %6d %6d\n", ax, ay, az, gx, gy, gz);
  delay(250);
}
