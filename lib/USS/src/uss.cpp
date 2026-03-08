#include <Arduino.h>
#include "pin_map.h"
#include "uss_config.h"

// Unnamed namespace is private
namespace {

struct UssSnapshot {
  float left_front_cm = 0.0f;
  float left_rear_cm = 0.0f;
  float front_cm = 0.0f;
  bool left_front_triggered = false;
  bool left_rear_triggered = false;
  bool front_triggered = false;
};

// Create UssSnapShot instance
UssSnapshot g_last{};
unsigned long g_last_left_read_ms = 0UL;
unsigned long g_last_front_read_ms = 0UL;
uint8_t g_left_uss_index = 0;       // round-robin sensor index (0, 1)

#if USS_DEBUG_PRINT
unsigned long g_last_debug_ms = 0UL;
#endif

float readDistanceCm(uint8_t trig_pin, uint8_t echo_pin, unsigned long timeout_us) {
    // sends a short trigger pulse on trig_pin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Measure how long the echo_pin stays high in microsecond
  const unsigned long pulse_us = pulseIn(echo_pin, HIGH, timeout_us);
  if (pulse_us == 0) {
    return 0.0f;
  }
  // distance = 0.0343 * t/2 = t/58.3 in cm
  return pulse_us / 58.3f;
}

bool isTriggered(float distance_cm, float threshold_cm) {
  return (distance_cm > 0.0f) && (distance_cm < threshold_cm);
}

}  // namespace

// init all pins for USS
void initUss() {
  pinMode(USS_LEFT_FRONT_TRIG, OUTPUT);
  pinMode(USS_LEFT_FRONT_ECHO, INPUT);
  pinMode(USS_LEFT_REAR_TRIG, OUTPUT);
  pinMode(USS_LEFT_REAR_ECHO, INPUT);
  pinMode(USS_FRONT_TRIG, OUTPUT);
  pinMode(USS_FRONT_ECHO, INPUT);

  digitalWrite(USS_LEFT_FRONT_TRIG, LOW);
  digitalWrite(USS_LEFT_REAR_TRIG, LOW);
  digitalWrite(USS_FRONT_TRIG, LOW);
}

// Update ONE left USS sensor per call (round-robin) to avoid 90ms blocking.
// Both left sensors are refreshed within 2 × USS_LEFT_READ_INTERVAL_MS.
void updateLeftUss(unsigned long timeout_us) {
  const unsigned long now_ms = millis();
  if ((now_ms - g_last_left_read_ms) < USS_LEFT_READ_INTERVAL_MS) {
    return;
  }
  g_last_left_read_ms = now_ms;

  switch (g_left_uss_index) {
    case 0:
      g_last.left_front_cm = readDistanceCm(USS_LEFT_FRONT_TRIG, USS_LEFT_FRONT_ECHO, timeout_us);
      g_last.left_front_triggered = isTriggered(g_last.left_front_cm, USS_LEFT_THRESHOLD_CM);
      break;
    case 1:
      g_last.left_rear_cm = readDistanceCm(USS_LEFT_REAR_TRIG, USS_LEFT_REAR_ECHO, timeout_us);
      g_last.left_rear_triggered = isTriggered(g_last.left_rear_cm, USS_LEFT_THRESHOLD_CM);
      break;
  }

  g_left_uss_index = (g_left_uss_index + 1) % 2;

#if USS_DEBUG_PRINT
  if ((now_ms - g_last_debug_ms) >= USS_DEBUG_INTERVAL_MS) {
    g_last_debug_ms = now_ms;
    Serial.print(F("[USS] LF="));
    Serial.print(g_last.left_front_cm, 1);
    Serial.print(F(" LR="));
    Serial.print(g_last.left_rear_cm, 1);
    Serial.print(F(" FR="));
    Serial.print(g_last.front_cm, 1);
    Serial.print(F("  Trig: LF="));
    Serial.print(g_last.left_front_triggered);
    Serial.print(F(" LR="));
    Serial.print(g_last.left_rear_triggered);
    Serial.print(F(" FR="));
    Serial.println(g_last.front_triggered);
  }
#endif
}

// Update the front USS sensor independently with a lower frequency
void updateFrontUss() {
  const unsigned long now_ms = millis();
  if ((now_ms - g_last_front_read_ms) < USS_FRONT_READ_INTERVAL_MS) {
    return;
  }
  g_last_front_read_ms = now_ms;

  g_last.front_cm = readDistanceCm(USS_FRONT_TRIG, USS_FRONT_ECHO, USS_FRONT_PULSE_TIMEOUT_US);
  g_last.front_triggered = isTriggered(g_last.front_cm, USS_FRONT_THRESHOLD_CM);
}

float ussLeftFrontCm() {
  return g_last.left_front_cm;
}

float ussLeftRearCm() {
  return g_last.left_rear_cm;
}

float ussFrontCm() {
  return g_last.front_cm;
}

bool ussLeftFrontTriggered() {
  return g_last.left_front_triggered;
}

bool ussLeftRearTriggered() {
  return g_last.left_rear_triggered;
}

bool ussFrontTriggered() {
  return g_last.front_triggered;
}
