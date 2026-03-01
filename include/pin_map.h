#pragma once

#include <Arduino.h>

// Shared hardware pin map for the whole project.
// constexpr: compile-time constant, immutable
namespace Pins {

// Ultrasonic sensors (Arduino Mega digital pins).
constexpr uint8_t kUssLeftFrontTrig = 22;
constexpr uint8_t kUssLeftFrontEcho = 23;

constexpr uint8_t kUssLeftRearTrig = 24;
constexpr uint8_t kUssLeftRearEcho = 25;

constexpr uint8_t kUssFrontTrig = 26;
constexpr uint8_t kUssFrontEcho = 27;

// Stepper motor pins
constexpr uint8_t STEP_PIN = 4;
constexpr uint8_t DIR_PIN = 5;

// Line sensor pins
constexpr uint8_t irPinsLeft = 8;
constexpr uint8_t irPinsLeft2 = 9;
constexpr uint8_t irPinsMiddle = 10;
constexpr uint8_t irPinsRight2 = 11;
constexpr uint8_t irPinsRight = 12;

// Add motor / launcher / line-sensor pins below as modules are implemented.


// constexpr uint8_t kMotorLeftPwm = ...;

}  // namespace Pins
