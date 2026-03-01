#include <Arduino.h>
#include <TimerInterrupt.h>
#include <uss.h>
#include <stepper_motor.h>
#include <state_machine.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);

  
  initUss();
  initStepperMotor();
  initLineSensor();
  initStateMachine());
}

void loop() {
  updateStateMachine();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}