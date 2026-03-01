/**
 * @file stepper_config.h
 * @brief Low-level configuration for the stepper motor subsystem
 */

#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

#include "config.h"

/** Step interval in microseconds (controls stepper speed) */
#define STEPPER_STEP_INTERVAL_US  2000UL

/** Default steps per launch cycle (~1 rev for 1.8° stepper) */
#define STEPPER_LAUNCH_STEPS      200

#endif /* STEPPER_CONFIG_H */
