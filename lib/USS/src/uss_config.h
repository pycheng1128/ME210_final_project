/**
 * @file uss_config.h
 * @brief Low-level configuration for the ultrasonic sensor subsystem
 */

#ifndef USS_CONFIG_H
#define USS_CONFIG_H

#include "config.h"

/** Minimum interval between USS readings (ms) */
#define USS_READ_INTERVAL_MS    60

/** Trigger distance for isTriggered() check (cm) */
#define USS_THRESHOLD_CM        60.0f

/** Pulse timeout — max echo wait (~517 cm) */
#define USS_PULSE_TIMEOUT_US    5500UL

#endif /* USS_CONFIG_H */
