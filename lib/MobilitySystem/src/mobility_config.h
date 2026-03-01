/**
 * @file mobility_config.h
 * @brief Low-level hardware configuration for the mobility subsystem
 *
 * Motor wiring, encoder specs, PWM limits, and safety thresholds.
 * High-level tuning (PID gains, speed limits, debug) lives in
 * the project-level include/config.h.
 */

#ifndef MOBILITY_CONFIG_H
#define MOBILITY_CONFIG_H

#include "config.h"

/* =====================================================================
 *  Motor Direction Correction
 *  (1.0 = normal, -1.0 = inverted wiring)
 * ===================================================================== */
#define MOB_MOTOR_1_INV        -1.0f   /* Front-Left  */
#define MOB_MOTOR_2_INV         1.0f   /* Front-Right */
#define MOB_MOTOR_3_INV        -1.0f   /* Back-Left   */
#define MOB_MOTOR_4_INV         1.0f   /* Back-Right  */

/* =====================================================================
 *  Motor Specifications
 * ===================================================================== */
#define MOB_ENCODER_PPR         1176.0f
#define MOB_MAX_RPM             60.0f
#define MOB_DEFAULT_TARGET_RPM  50.0f

/* =====================================================================
 *  Low-Pass Filter (measured speed smoothing)
 * ===================================================================== */
#define MOB_LPF_ALPHA           0.4f

/* =====================================================================
 *  Anti-Windup: Integral Clamping
 * ===================================================================== */
#define MOB_PID_INTEGRAL_MAX    255.0f

/* =====================================================================
 *  Safety
 * ===================================================================== */
#define MOB_ESTOP_ACTIVE_LEVEL  LOW
#define MOB_STOPPED_RPM_THRESH  2.0f
#define MOB_REVERSAL_RPM_THRESH 5.0f

#endif /* MOBILITY_CONFIG_H */
