/**
 * @file state_machine_config.h
 * @brief Tuning parameters for the state machine subsystem
 *
 * All values referenced by state_machine.cpp live here so they can
 * be tweaked in one place without touching logic.
 */

#ifndef STATE_MACHINE_CONFIG_H
#define STATE_MACHINE_CONFIG_H

#include "config.h"

/* ── Launch ────────────────────────────────────────────────────────── */

/** Stepper launch direction (1 = clockwise, 0 = counter-clockwise) */
#define FSM_LAUNCH_CLOCKWISE          1

/** Stepper steps per launch cycle */
#define FSM_LAUNCH_CYCLE_STEPS        1600U

/* ── Alignment ─────────────────────────────────────────────────────── */

/** Rotation speed while aligning to left wall (RPM) */
#define FSM_ALIGN_ROTATE_RPM          15.0f

/** Consecutive wall-detected readings in PRE_ALIGN before entering TURN_ALIGN */
#define FSM_PRE_ALIGN_CONSEC_REQUIRED 5

/** Consecutive aligned USS readings required before transition (debounce) */
#define FSM_ALIGN_CONSEC_REQUIRED     5

/** Minimum ms between debounce counter increments (≥ 1 full USS cycle) */
#define FSM_ALIGN_DEBOUNCE_MS         50UL

/** Rotation speed used while searching for wall when both left USS read no-echo */
#define FSM_ALIGN_SEARCH_ROTATE_RPM   40.0f

/** Max continuous no-echo search time before entering FAULT (ms) */
#define FSM_ALIGN_NO_ECHO_TIMEOUT_MS  80000UL

/* ── Forward After Align ───────────────────────────────────────────── */

/** Duration to drive forward after aligning to wall (ms) */
#define FSM_FORWARD_AFTER_ALIGN_MS    2000UL

/** Forward speed after align (RPM) */
#define FSM_FORWARD_AFTER_ALIGN_RPM   20.0f

/* ── Shift Right ───────────────────────────────────────────────────── */

/** Right-strafe speed while searching for center line (RPM) */
#define FSM_SHIFT_RIGHT_RPM           15.0f

/** Rotational correction applied during right shift (RPM) */
#define FSM_SHIFT_ROTATE_RPM           30.0f

/** Re-align with TURN_ALIGN when left USS mismatch exceeds this (cm) */
#define FSM_SHIFT_REALIGN_DIFF_CM      1.2f

/* ── Forward / Line Follow ─────────────────────────────────────────── */

/** Forward speed while approaching hog line (RPM) */
#define FSM_FORWARD_TO_HOG_RPM        15.0f

/** Lateral correction magnitude for line following (RPM) */
#define FSM_LINE_FOLLOW_STRAFE_RPM     16.0f

/* ── Return to End Zone ────────────────────────────────────────────── */

/** Front USS target distance to trigger return maneuver (cm) */
#define FSM_RETURN_FRONT_TARGET_CM    15.0f

/** Return line-follow forward speed (RPM) */
#define FSM_RETURN_LINE_FOLLOW_RPM   15.0f

/** Consecutive left-side line detections before transitioning to shift-left */
#define FSM_RETURN_LINE_CONSEC_REQUIRED 3

/** Rotation speed during 180-degree turn (RPM) */
#define FSM_RETURN_TURN_RPM           15.0f

/** Duration for 180-degree turn (ms, tune on robot) */
#define FSM_RETURN_TURN_180_MS       1300UL

/** Left strafe speed for final lateral move (RPM) */
#define FSM_RETURN_SHIFT_LEFT_RPM     15.0f

/** Duration for ~30 cm left strafe (ms, tune on robot) */
#define FSM_RETURN_SHIFT_LEFT_30CM_MS 3500UL

/* ── Speed-Up Phase ───────────────────────────────────────────────── */

/** High speed during initial forward/backward burst (RPM) */
#define FSM_SPEEDUP_RPM             58.0f

/** Distance to travel at high speed before slowing down (cm) */
#define FSM_SPEEDUP_DISTANCE_CM     145.0f

/** Encoder counts for the speed-up distance (computed from wheel diameter & PPR) */
#define FSM_SPEEDUP_ENCODER_COUNTS  ((long)( \
    (FSM_SPEEDUP_DISTANCE_CM / (3.14159265f * MOB_WHEEL_DIAMETER_MM / 10.0f)) \
    * MOB_ENCODER_PPR))

/* ── Lateral Shift (hogline approach offset per cycle) ────────────── */

/** Lateral shift distance after speed-up (cm, cycles 2 & 3 only) */
#define FSM_LATERAL_SHIFT_DISTANCE_CM   12.0f

/** Strafe speed during lateral shift (RPM) */
#define FSM_LATERAL_SHIFT_RPM           15.0f

/** Encoder counts for lateral shift distance */
#define FSM_LATERAL_SHIFT_ENCODER_COUNTS ((long)( \
    (FSM_LATERAL_SHIFT_DISTANCE_CM / (3.14159265f * MOB_WHEEL_DIAMETER_MM / 10.0f)) \
    * MOB_ENCODER_PPR))

/* ── USS Fault ─────────────────────────────────────────────────────── */

/** Max consecutive invalid USS reads before entering FAULT */
#define FSM_USS_FAULT_MAX_CONSEC      10

/* ── FAULT ─────────────────────────────────────────────────────────── */

/** FAULT status print period while waiting for recovery (ms) */
#define FSM_FAULT_LOG_INTERVAL_MS    1000UL

#endif /* STATE_MACHINE_CONFIG_H */
