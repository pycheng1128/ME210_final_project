/**
 * @file stepper_motor.cpp
 * @brief Timer-interrupt driven stepper motor for precise, non-blocking step pulses.
 *
 * Uses Timer3 on the ATmega2560 to generate step pulses at STEPPER_STEP_FREQ_HZ.
 * This guarantees exact step timing regardless of main loop latency (USS, PID, etc.),
 * providing consistent torque and reliable stepping.
 *
 * Lifecycle:
 *   1. startStepperLaunchCycle() → sets direction, enters PENDING state
 *   2. updateStepperMotor()      → after STEPPER_STARTUP_DELAY_MS, starts Timer3
 *   3. Timer3 ISR               → generates step pulses until target reached
 *   4. ISR auto-stops           → sets cycle_complete flag
 *
 * Timer3 was chosen because:
 *   - Timer0: used by Arduino millis()/micros()
 *   - Timer1: may be used by Servo library or other subsystems
 *   - Timer2: 8-bit only, limited resolution
 *   - Timer4/5: available but Timer3 is sufficient
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

enum class StepperState : uint8_t {
  IDLE,       // not running
  PENDING,    // direction set, waiting for startup delay
  RUNNING     // Timer3 ISR is stepping
};

volatile StepperState g_state          = StepperState::IDLE;
volatile uint16_t     g_step_count     = 0;
volatile uint16_t     g_target_steps   = 0;
volatile bool         g_cycle_complete = false;

unsigned long         g_pending_start_ms = 0;

// Timer3 ISR — fires at STEPPER_STEP_FREQ_HZ, generates one step pulse per call
void stepperTimerISR() {
  if (g_state != StepperState::RUNNING) return;

  if (g_step_count >= g_target_steps) {
    // Done — stop the timer and set completion flag
    ITimer3.detachInterrupt();
    g_state = StepperState::IDLE;
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

  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, HIGH);  // default direction

  ITimer3.init();
}

void startStepperLaunchCycle(bool clockwise, uint16_t steps) {
  // Guard against 0 steps
  if (steps == 0) steps = 1;

  // Set direction
  digitalWrite(STEPPER_DIR_PIN, clockwise ? HIGH : LOW);

  // Enter PENDING — wait for startup delay before stepping
  noInterrupts();
  g_step_count     = 0;
  g_target_steps   = steps;
  g_cycle_complete = false;
  g_state          = StepperState::PENDING;
  interrupts();

  g_pending_start_ms = millis();
}

void updateStepperMotor() {
  // Non-blocking startup delay: wait for driver to settle, then start timer.
  if (g_state == StepperState::PENDING) {
    if ((millis() - g_pending_start_ms) >= STEPPER_STARTUP_DELAY_MS) {
      noInterrupts();
      g_state = StepperState::RUNNING;
      interrupts();
      ITimer3.setFrequency((float)STEPPER_STEP_FREQ_HZ, stepperTimerISR);
    }
  }
}

bool isStepperMotorBusy() {
  return (g_state != StepperState::IDLE);
}

bool checkStepperLaunchCycleComplete() {
  if (!g_cycle_complete) return false;
  g_cycle_complete = false;
  return true;
}

void stopStepperMotor() {
  ITimer3.detachInterrupt();
  noInterrupts();
  g_state          = StepperState::IDLE;
  g_cycle_complete = false;
  interrupts();
  digitalWrite(STEPPER_STEP_PIN, LOW);
}
