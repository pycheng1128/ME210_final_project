/**
 * @file stepper_config.h
 * @brief Low-level configuration for the stepper motor subsystem
 *
 * Uses Timer3 hardware interrupt for precise step timing.
 * This provides consistent torque at all speeds regardless of
 * main loop timing (USS blocking, PID, etc.)
 */

#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

#include "config.h"

/** Step frequency in Hz (steps per second).
 *  Higher = faster but less torque. Lower = slower but more torque.
 *  Typical range for NEMA 17: 200–800 Hz.
 */
#define STEPPER_STEP_FREQ_HZ      500

/** Default steps per launch cycle (~2 revs for 1.8° stepper = 400 steps) */
#define STEPPER_LAUNCH_STEPS      400

#endif /* STEPPER_CONFIG_H */
