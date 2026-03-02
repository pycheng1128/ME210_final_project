/**
 * @file stepper_motor.cpp
 * @brief Timer-interrupt driven stepper motor for precise, non-blocking step pulses.
 *
 * Rewritten to match the tested standalone code. Key changes from previous version:
 *   - ENABLE pin control (HIGH = driver OFF, LOW = driver ON)
 *   - No PENDING / startup-delay state — stepping starts immediately
 *   - Step frequency bumped to 500 Hz (configurable in stepper_config.h)
 *
 * Uses Timer3 on the ATmega2560 to generate step pulses at STEPPER_STEP_FREQ_HZ.
 *
 * Lifecycle:
 *   1. startStepperLaunchCycle() → sets direction, enables driver, starts Timer3
 *   2. Timer3 ISR               → generates step pulses until target reached
 *   3. ISR auto-stops           → disables driver, sets cycle_complete flag
 */

#include <Arduino.h>
#include "pin_map.h"
#include "stepper_config.h"
#include "stepper_motor.h"

// Must define USE_TIMER_3 before including the library
#define USE_TIMER_3   true
#include <TimerInterrupt.h>

// ── Private state ──────────────────────────────────────────────────

namespace {

volatile int      g_step_count     = 0;
volatile int      g_target_steps   = 0;
volatile bool     g_running        = false;
volatile bool     g_cycle_complete = false;

// Timer3 ISR — fires at STEPPER_STEP_FREQ_HZ, generates one step pulse per call
void stepperTimerISR() {
  if (!g_running) return;

  if (g_step_count >= g_target_steps) {
    // Done — stop the timer, disable driver, set completion flag
    ITimer3.detachInterrupt();
    g_running = false;
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // driver OFF
    g_cycle_complete = true;
    return;
  }

  // Generate one step pulse (rising edge triggers the driver)
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  digitalWrite(STEPPER_STEP_PIN, LOW);

  g_step_count++;
}

}  // namespace

// ── Public API ─────────────────────────────────────────────────────

void initStepperMotor() {
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, HIGH);        // default direction
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);     // driver OFF until needed

  ITimer3.init();
}

void startStepperLaunchCycle(bool clockwise, uint16_t steps) {
  // Guard against 0 steps
  if (steps == 0) steps = 1;

  // Set direction
  digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

  // Enable driver
  digitalWrite(STEPPER_ENABLE_PIN, LOW);      // driver ON

  // Reset counters and start timer immediately (no startup delay)
  noInterrupts();
  g_step_count     = 0;
  g_target_steps   = steps;
  g_cycle_complete = false;
  g_running        = true;
  interrupts();

  ITimer3.setFrequency((float)STEPPER_STEP_FREQ_HZ, stepperTimerISR);
}

void updateStepperMotor() {
  // No-op — stepping is fully handled by the Timer3 ISR.
  // Kept for API compatibility with the state machine.
}

bool isStepperMotorBusy() {
  return g_running;
}

bool checkStepperLaunchCycleComplete() {
  if (!g_cycle_complete) return false;
  g_cycle_complete = false;
  return true;
}

void stopStepperMotor() {
  ITimer3.detachInterrupt();
  noInterrupts();
  g_running        = false;
  g_cycle_complete = false;
  interrupts();
  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);     // driver OFF
}
