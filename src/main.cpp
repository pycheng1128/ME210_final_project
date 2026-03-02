#include <Arduino.h>
#include <line_sensor.h>
#include <mobility_driver.h>
#include <state_machine.h>
#include <stepper_motor.h>
#include <uss.h>

#ifndef ENABLE_MOBILITY_TEST_APP
#define ENABLE_MOBILITY_TEST_APP 0
#endif

#if !ENABLE_MOBILITY_TEST_APP

void setup() {
  Mobility_Init();
  initUss();
  initLineSensor();
  initStepperMotor();
  initStateMachine();
}

void loop() {
  updateStateMachine();
  Mobility_Update();
}

#endif  // !ENABLE_MOBILITY_TEST_APP
