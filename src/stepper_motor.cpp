#include <Arduino.h>
#include "pin_map.h"
#include "stepper_motor.h"

// Private namespace
namespace {

// Non-blocking step interval. Tune this for your driver/mechanics.
constexpr unsigned long kStepIntervalUs = 2000UL;

struct Runtime {
  bool active = false;
  bool step_pin_high = false;
  bool cycle_complete = false;

  uint16_t target_steps = 0;
  uint16_t steps_done = 0;
  unsigned long last_toggle_us = 0UL;
};

Runtime g_rt;

// conterclockwise if clockwise == true, else CCW
void setDirection(bool clockwise) {
  digitalWrite(Pins::DIR_PIN, clockwise ? HIGH : LOW);
}

}  // namespace

void initStepperMotor() {
  pinMode(Pins::STEP_PIN, OUTPUT);
  pinMode(Pins::DIR_PIN, OUTPUT);

  digitalWrite(Pins::STEP_PIN, LOW);
  setDirection(true);
}

// Starts a stepper launch motion (prepares for updateStepperMotor())
void startStepperLaunchCycle(bool clockwise, uint16_t steps) {
  // Avoid 0 steps
  if (steps == 0) {
    steps = 1;
  }

  g_rt.active = true;           // motor is running
  g_rt.step_pin_high = false;   // pulse starts from LOW
  g_rt.cycle_complete = false;  // completion flag
  g_rt.target_steps = steps;    // how many steps to do
  g_rt.steps_done = 0;          // reset count 
  g_rt.last_toggle_us = micros();

  setDirection(clockwise);
  digitalWrite(Pins::STEP_PIN, LOW); // STEP_PIN starts as LOW
}

void updateStepperMotor() {
  // if stepper is not active, no need to update
  if (!g_rt.active) {
    return;
  }

  // check if stepper active time exceeds predefined limit
  // replace the functionality of delayMicroseconds()
  const unsigned long now_us = micros();
  if ((now_us - g_rt.last_toggle_us) < kStepIntervalUs) {
    return;
  }
  // update time
  g_rt.last_toggle_us = now_us;

  // if STEP PIN is low, toggle to high
  if (!g_rt.step_pin_high) {
    digitalWrite(Pins::STEP_PIN, HIGH);
    g_rt.step_pin_high = true;
    return;
  }

  digitalWrite(Pins::STEP_PIN, LOW);
  g_rt.step_pin_high = false;
  g_rt.steps_done++;

  // if cycle complete
  if (g_rt.steps_done >= g_rt.target_steps) {
    g_rt.active = false;
    g_rt.cycle_complete = true;
  }
}

bool isStepperMotorBusy() {
  return g_rt.active;
}

// cycle complete -> true, else false
bool checkStepperLaunchCycleComplete() {
  if (!g_rt.cycle_complete) {
    return false;
  }
  g_rt.cycle_complete = false;
  return true;
}

void stopStepperMotor() {
  g_rt.active = false;
  digitalWrite(Pins::STEP_PIN, LOW);
}
