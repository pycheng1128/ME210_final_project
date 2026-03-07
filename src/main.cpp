#include <Arduino.h>
#include <line_sensor.h>
#include <mobility_driver.h>
#include <state_machine.h>
#include <stepper_motor.h>
#include <uss.h>
#include "config.h"

#ifndef ENABLE_MOBILITY_TEST_APP
#define ENABLE_MOBILITY_TEST_APP 0
#endif

#if !ENABLE_MOBILITY_TEST_APP

void setup() {
  Serial.begin(SERIAL_BAUD);
  Mobility_Init();
  initUss();
  initLineSensor();
  initStepperMotor();
  initStateMachine();
}

void loop() {
  Mobility_Update();
  updateStateMachine();
}

#endif  // !ENABLE_MOBILITY_TEST_APP
