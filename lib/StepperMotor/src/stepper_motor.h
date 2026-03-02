#pragma once

#include <Arduino.h>
#include "stepper_config.h"

// Initialize stepper GPIO pins and hardware timer.
void initStepperMotor();

// Start one launch cycle.
void startStepperLaunchCycle(bool clockwise = true, uint16_t steps = STEPPER_LAUNCH_STEPS);

// No-op — stepping is fully handled by Timer3 ISR. Kept for API compatibility.
void updateStepperMotor();

// True while a launch cycle is running.
bool isStepperMotorBusy();

// Returns true once when a launch cycle finishes, then clears the flag.
bool checkStepperLaunchCycleComplete();

// Immediate stop (detaches timer, no further stepping).
void stopStepperMotor();
