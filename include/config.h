/**
 * @file config.h
 * @brief Project-level tuning parameters (high-level control knobs)
 *
 * Only parameters you'd tweak during testing live here.
 * Low-level motor/hardware specs stay in their respective library configs.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* =====================================================================
 *  Serial / Debug
 * ===================================================================== */
#define SERIAL_BAUD             115200
#define MOB_DEBUG_PRINT         0
#define MOB_DEBUG_INTERVAL_MS   100
#define USS_DEBUG_PRINT         1
#define USS_DEBUG_INTERVAL_MS   200
#define LINE_DEBUG_PRINT        1
#define LINE_DEBUG_INTERVAL_MS  200

/* =====================================================================
 *  Mobility — PID Gains
 * ===================================================================== */
#define MOB_PID_KP              12.0f
#define MOB_PID_KI              35.0f
#define MOB_PID_KD              0.0f
#define MOB_PID_INTERVAL_MS     20

/* =====================================================================
 *  Mobility — PWM Configuration
 * ===================================================================== */
#define MOB_PWM_MAX             255
#define MOB_PWM_DEADZONE        60

/* =====================================================================
 *  USS — Detection Threshold
 * ===================================================================== */
#define USS_THRESHOLD_CM        50.0f

/* =====================================================================
 *  State Machine — Timing
 * ===================================================================== */
#define FSM_LOAD_IDLE_MS        5000UL
#define FSM_STATE_TIMEOUT_MS    70000UL
#define FSM_PARALLEL_TOLERANCE_CM  0.6f

#endif  /* CONFIG_H */
