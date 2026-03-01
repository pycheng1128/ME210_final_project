#pragma once

#include <Arduino.h>

// Initialize stepper GPIO pins.
void initStepperMotor();

// Start one launch cycle (default 200 steps ~= 1 rev for common 1.8 deg steppers).
void startStepperLaunchCycle(bool clockwise = true, uint16_t steps = 200);

// Non-blocking update; call this every loop.
void updateStepperMotor();

// True while a launch cycle is running.
bool isStepperMotorBusy();

// Returns true once when a launch cycle finishes, then clears the flag.
bool checkStepperLaunchCycleComplete();

// Immediate stop (keeps pin outputs valid, no further stepping).
void stopStepperMotor();
