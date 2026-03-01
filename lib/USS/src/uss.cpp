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
unsigned long g_last_read_ms = 0UL;

float readDistanceCm(uint8_t trig_pin, uint8_t echo_pin) {
    // sends a short trigger pulse on trig_pin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  // Measure how long the echo_pin stays high in microsecond
  const unsigned long pulse_us = pulseIn(echo_pin, HIGH, USS_PULSE_TIMEOUT_US);
  if (pulse_us == 0) {
    return 0.0f;
  }
  // distance = 0.0343 * t/2 = t/58.3 in cm
  return pulse_us / 58.3f;
}

bool isTriggered(float distance_cm) {
  return (distance_cm > 0.0f) && (distance_cm < USS_THRESHOLD_CM);
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

// Update readings of USS
void updateUss() {
    // Don't change g_last if interval is short
  const unsigned long now_ms = millis();
  if ((now_ms - g_last_read_ms) < USS_READ_INTERVAL_MS) {
    return;
  }
  g_last_read_ms = now_ms;

  g_last.left_front_cm = readDistanceCm(USS_LEFT_FRONT_TRIG, USS_LEFT_FRONT_ECHO);
  g_last.left_rear_cm = readDistanceCm(USS_LEFT_REAR_TRIG, USS_LEFT_REAR_ECHO);
  g_last.front_cm = readDistanceCm(USS_FRONT_TRIG, USS_FRONT_ECHO);

  g_last.left_front_triggered = isTriggered(g_last.left_front_cm);
  g_last.left_rear_triggered = isTriggered(g_last.left_rear_cm);
  g_last.front_triggered = isTriggered(g_last.front_cm);
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
