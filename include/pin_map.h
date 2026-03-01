/**
 * @file pin_map.h
 * @brief Centralized pin map for the entire ME210 Final Project
 *        (Arduino Mega 2560)
 *
 * Every subsystem's GPIO assignments live here so that pin conflicts
 * are easy to spot and resolve.  All modules should #include this
 * file instead of hard-coding pin numbers.
 *
 * ┌───────────────────────────────────────────────────────────────────┐
 * │                      PIN ASSIGNMENT SUMMARY                      │
 * ├──────────────┬──────────────────────────────────┬────────────────┤
 * │  Subsystem   │  Function                        │  Pin(s)        │
 * ├──────────────┼──────────────────────────────────┼────────────────┤
 * │  Mobility    │  Motor 1 IN1/IN2                 │  D22, D23      │
 * │              │  Motor 2 IN1/IN2                 │  D24, D25      │
 * │              │  Motor 3 IN1/IN2                 │  D26, D27      │
 * │              │  Motor 4 IN1/IN2                 │  D28, D29      │
 * │              │  Motor 1-4 ENA (PWM)             │  D4–D7         │
 * │              │  Encoder 1-4 A (INT)             │  D18–D21       │
 * │              │  Encoder 1-4 B                   │  D30–D33       │
 * │              │  E-Stop                          │  D42           │
 * ├──────────────┼──────────────────────────────────┼────────────────┤
 * │  Stepper     │  STEP / DIR                      │  D34, D35      │
 * ├──────────────┼──────────────────────────────────┼────────────────┤
 * │  USS         │  Ultra 1  Trig / Echo            │  D36, D37      │
 * │              │  Ultra 2  Trig / Echo            │  D38, D39      │
 * │              │  Ultra 3  Trig / Echo            │  D40, D41      │
 * ├──────────────┼──────────────────────────────────┼────────────────┤
 * │  Line Sensor │  IR Left / Left2 / Mid / R2 / R │  A0–A4         │
 * └──────────────┴──────────────────────────────────┴────────────────┘
 *
 * Notes:
 *   - Mega interrupt pins: 2, 3, 18, 19, 20, 21
 *   - Mega PWM pins:       2–13, 44–46
 *   - Analog pins:         A0–A15  (also digital 54–69)
 */

#ifndef PIN_MAP_H
#define PIN_MAP_H

#include <Arduino.h>

/* =====================================================================
 *  MOBILITY — 4× DC Motor + Encoder + E-Stop
 * ===================================================================== */

#define MOB_NUM_MOTORS          4

/* Motor Driver Direction (IN1 / IN2) */
#define MOB_DRIVER_1_IN1        22      /* Motor 1 — U1-A */
#define MOB_DRIVER_1_IN2        23
#define MOB_DRIVER_2_IN1        24      /* Motor 2 — U1-B */
#define MOB_DRIVER_2_IN2        25
#define MOB_DRIVER_3_IN1        26      /* Motor 3 — U2-A */
#define MOB_DRIVER_3_IN2        27
#define MOB_DRIVER_4_IN1        28      /* Motor 4 — U2-B */
#define MOB_DRIVER_4_IN2        29

/* Motor Driver PWM / Enable (ENA) — must be PWM-capable */
#define MOB_DRIVER_1_ENA        4
#define MOB_DRIVER_2_ENA        5
#define MOB_DRIVER_3_ENA        6
#define MOB_DRIVER_4_ENA        7

/* Encoder A — interrupt-capable (ISR pulse counting) */
#define MOB_MOTOR_1_ENCA        18
#define MOB_MOTOR_2_ENCA        19
#define MOB_MOTOR_3_ENCA        20
#define MOB_MOTOR_4_ENCA        21

/* Encoder B — digital input (direction sensing) */
#define MOB_MOTOR_1_ENCB        30
#define MOB_MOTOR_2_ENCB        31
#define MOB_MOTOR_3_ENCB        32
#define MOB_MOTOR_4_ENCB        33

/* E-Stop (active LOW, internal pull-up) */
#define MOB_ESTOP_PIN           42

/* =====================================================================
 *  ULTRASONIC SENSORS (USS) — 3× HC-SR04
 * ===================================================================== */
#define USS_LEFT_FRONT_TRIG     36      /* Ultra 1 */
#define USS_LEFT_FRONT_ECHO     37
#define USS_LEFT_REAR_TRIG      38      /* Ultra 2 */
#define USS_LEFT_REAR_ECHO      39
#define USS_FRONT_TRIG          40      /* Ultra 3 */
#define USS_FRONT_ECHO          41

/* =====================================================================
 *  LINE SENSOR — 5-channel IR reflectance array
 * ===================================================================== */
#define IR_PIN_LEFT             A0      /* leftmost sensor */
#define IR_PIN_LEFT2            A1
#define IR_PIN_MIDDLE           A2
#define IR_PIN_RIGHT2           A3
#define IR_PIN_RIGHT            A4      /* rightmost sensor */

/* =====================================================================
 *  STEPPER MOTOR — A4988 / DRV8825 style driver
 * ===================================================================== */
#define STEPPER_STEP_PIN        34
#define STEPPER_DIR_PIN         35

#endif  /* PIN_MAP_H */
