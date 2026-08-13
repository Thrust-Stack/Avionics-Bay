/*
 * Servo_Hold_Zero -- standalone ESP32 canard servo hold sketch.
 *
 * Drives both canard servo signal pins with a fixed 50 Hz PWM command.
 * There is no sweep, sensor input, serial command parser, or control loop.
 *
 * Wiring:
 *   Canard 1 signal: GPIO26
 *   Canard 2 signal: GPIO25
 *
 * Power servos from the external 5V BEC/buck, not from the ESP32.
 * Servo ground and ESP32 ground must be common.
 */

#include <Arduino.h>

#define CANARD1_PIN 26
#define CANARD2_PIN 25

#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400
#define SERVO_PWM_BITS 16

// Literal servo angle to hold. Set to 90.0f if you want the repo's neutral angle.
#define HOLD_ANGLE_DEG 90.0f

#define REASSERT_PERIOD_MS 1000
#define STATUS_PERIOD_MS   2000

unsigned long lastReassertMs = 0;
unsigned long lastStatusMs = 0;

uint32_t angleToPWM16(float angleDeg) {
  if (angleDeg < 0.0f) angleDeg = 0.0f;
  if (angleDeg > 180.0f) angleDeg = 180.0f;

  const float pulseUs = SERVO_MIN_US + (angleDeg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);
  const float periodUs = 1000000.0f / (float)SERVO_FREQ_HZ;
  return (uint32_t)((pulseUs / periodUs) * 65535.0f);
}

void writeHoldAngle() {
  const uint32_t duty = angleToPWM16(HOLD_ANGLE_DEG);
  ledcWrite(CANARD1_PIN, duty);
  ledcWrite(CANARD2_PIN, duty);
}

void setup() {
  Serial.begin(115200);
  delay(250);

  ledcAttach(CANARD1_PIN, SERVO_FREQ_HZ, SERVO_PWM_BITS);
  ledcAttach(CANARD2_PIN, SERVO_FREQ_HZ, SERVO_PWM_BITS);

  writeHoldAngle();
  lastReassertMs = millis();
  lastStatusMs = millis();

  Serial.println("[SERVO_HOLD_ZERO] both servos holding fixed angle");
  Serial.print("[SERVO_HOLD_ZERO] angle_deg=");
  Serial.print(HOLD_ANGLE_DEG, 1);
  Serial.print(" canard1_gpio=");
  Serial.print(CANARD1_PIN);
  Serial.print(" canard2_gpio=");
  Serial.println(CANARD2_PIN);
}

void loop() {
  const unsigned long now = millis();

  if (now - lastReassertMs >= REASSERT_PERIOD_MS) {
    lastReassertMs = now;
    writeHoldAngle();
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    Serial.println("[SERVO_HOLD_ZERO] holding");
  }
}
