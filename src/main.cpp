#include <Arduino.h>
#include <TimerInterrupt.h>
#include "state_machine.cpp"

// put function declarations here:
int myFunction(int, int);

void setup() {

  initStateMachine());
}

void loop() {
  updateStateMachine();
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}