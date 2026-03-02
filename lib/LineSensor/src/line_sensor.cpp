#include <Arduino.h>
#include "pin_map.h"
#include "line_sensor_config.h"

namespace {

const uint8_t kLinePins[LINE_SENSOR_COUNT] = {
  IR_PIN_LEFT,
  IR_PIN_LEFT2,
  IR_PIN_MIDDLE,
  IR_PIN_RIGHT2,
  IR_PIN_RIGHT
};

// Cached 5-bit sensor pattern.
// bit4 = leftmost sensor ... bit0 = rightmost sensor.
// This is updated by updateLineSensor() once per loop.
uint8_t g_line_mask_5b = 0;

uint8_t sampleLineMask5b() {
  uint8_t mask = 0; // start with 0b00000

  // Convert 5 digital reads into a compact bit-mask.
  for (uint8_t i = 0; i < LINE_SENSOR_COUNT; ++i) {
    const uint8_t bit_index = 4 - i;
    const bool is_black = (digitalRead(kLinePins[i]) == LOW);
    if (is_black) {
      mask |= static_cast<uint8_t>(1U << bit_index);
    }
  }

  return mask;
}

}  // namespace

void initLineSensor() {
  for (uint8_t i = 0; i < LINE_SENSOR_COUNT; ++i) {
    pinMode(kLinePins[i], INPUT);
  }

  // Initialize cache once so other modules can safely read after init.
  g_line_mask_5b = sampleLineMask5b();
}

void updateLineSensor() {
  // Read all 5 sensors and update one cached value.
  g_line_mask_5b = sampleLineMask5b();

#if LINE_DEBUG_PRINT
  static unsigned long lastDebugMs = 0UL;
  const unsigned long now = millis();
  if ((now - lastDebugMs) >= LINE_DEBUG_INTERVAL_MS) {
    lastDebugMs = now;
    Serial.print(F("[LINE] "));
    for (int8_t b = 4; b >= 0; --b) {
      Serial.print((g_line_mask_5b >> b) & 1);
    }
    Serial.print(F("  L="));
    Serial.print((g_line_mask_5b >> 4) & 1);
    Serial.print(F(" L2="));
    Serial.print((g_line_mask_5b >> 3) & 1);
    Serial.print(F(" M="));
    Serial.print((g_line_mask_5b >> 2) & 1);
    Serial.print(F(" R2="));
    Serial.print((g_line_mask_5b >> 1) & 1);
    Serial.print(F(" R="));
    Serial.println(g_line_mask_5b & 1);
  }
#endif
}

// Return the latest cached 5-bit line pattern.
uint8_t lineMask5b() {
  return g_line_mask_5b;
}

bool isAllBlack() {
  return lineMask5b() == 0b11111;
}

bool isCentered() {
  const uint8_t mask = lineMask5b();
  return (mask == 0b01110) || (mask == 0b00100);
}
