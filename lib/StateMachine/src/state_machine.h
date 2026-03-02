#pragma once

#include <Arduino.h>

// Initialize state machine runtime.
void initStateMachine();

// Run one non-blocking FSM update (call every loop).
void updateStateMachine();
