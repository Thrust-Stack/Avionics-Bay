/*
 * Servo_Sweep_Test -- ESP32 canard actuator bench test.
 *
 * This version is set up as a PWM diagnostic. It drives the two servo signal
 * pins with a simple sweep of pulse widths so you can verify the ESP32 is
 * actually outputting a valid servo PWM signal.
 *
 * Board: NodeMCU-32S
 * Canard 1 signal: GPIO26
 * Canard 2 signal: GPIO25
 *
 * Power servos from the 8.4 V 2S LiPo with common ground to the ESP32.
 */

#define CANARD1_PIN     26
#define CANARD2_PIN     25
#define SERVO_FREQ_HZ   50
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2400
#define SERVO_STEP_US   100
#define STEP_PERIOD_MS  500

unsigned long lastStepMs = 0;
uint32_t pulseWidthUs = SERVO_MIN_US;
int pulseDirection = 1;

uint32_t pulseWidthToPWM16(uint32_t pulseWidth) {
  pulseWidth = constrain(pulseWidth, SERVO_MIN_US, SERVO_MAX_US);
  return (uint32_t)(pulseWidth / 20000.0f * 65535.0f);
}

void applyPulseWidth(uint32_t pulseWidth) {
  uint32_t duty = pulseWidthToPWM16(pulseWidth);
  ledcWrite(CANARD1_PIN, duty);
  ledcWrite(CANARD2_PIN, duty);

  Serial.printf("[PWM_TEST] pulse_us=%lu duty=%lu\n", pulseWidth, duty);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Servo PWM Diagnostic");
  Serial.println("  GPIO26 / GPIO25, 50 Hz servo pulses");
  Serial.println("========================================");

  ledcAttach(CANARD1_PIN, SERVO_FREQ_HZ, 16);
  ledcAttach(CANARD2_PIN, SERVO_FREQ_HZ, 16);

  pinMode(LED_BUILTIN, OUTPUT);
  applyPulseWidth(pulseWidthUs);
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  if (now - lastStepMs < STEP_PERIOD_MS) {
    return;
  }

  lastStepMs = now;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  applyPulseWidth(pulseWidthUs);

  pulseWidthUs += (uint32_t)(pulseDirection * SERVO_STEP_US);
  if (pulseWidthUs >= SERVO_MAX_US) {
    pulseWidthUs = SERVO_MAX_US;
    pulseDirection = -1;
  } else if (pulseWidthUs <= SERVO_MIN_US) {
    pulseWidthUs = SERVO_MIN_US;
    pulseDirection = 1;
  }
}
