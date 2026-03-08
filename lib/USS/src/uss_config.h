/**
 * @file uss_config.h
 * @brief Low-level configuration for the ultrasonic sensor subsystem
 */

#ifndef USS_CONFIG_H
#define USS_CONFIG_H

#include "config.h"

/** Minimum interval between Left USS readings (ms) */
#define USS_LEFT_READ_INTERVAL_MS    60

/** Minimum interval between Front USS readings (ms) */
#define USS_FRONT_READ_INTERVAL_MS   200

/** Trigger distance for left isTriggered() check (cm) */
#define USS_LEFT_THRESHOLD_CM        50.0f

/** Trigger distance for front isTriggered() check (cm) */
#define USS_FRONT_THRESHOLD_CM       2000.0f

/** Pulse timeout — Max echo wait for Left USS during Pre-Align (~171 cm = 10000us) */
#define USS_LEFT_PULSE_TIMEOUT_DEFAULT_US    80000UL

/** Pulse timeout — Fast echo wait for Left USS during Align (~85 cm = 5000us) */
#define USS_LEFT_PULSE_TIMEOUT_ALIGN_US      5000UL

/** Pulse timeout — Max echo wait for Front USS (~343 cm = 20000us) */
#define USS_FRONT_PULSE_TIMEOUT_US   12000UL

#endif /* USS_CONFIG_H */
