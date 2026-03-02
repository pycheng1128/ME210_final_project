/**
 * @file state_machine_config.h
 * @brief Low-level configuration for the state machine subsystem
 */

#ifndef STATE_MACHINE_CONFIG_H
#define STATE_MACHINE_CONFIG_H

#include "config.h"

/** Stepper launch direction (1 = clockwise, 0 = counter-clockwise) */
#define FSM_LAUNCH_CLOCKWISE      1

/** Stepper steps per launch cycle */
#define FSM_LAUNCH_CYCLE_STEPS    200U

/** Rotation speed used while aligning to left wall */
#define FSM_ALIGN_ROTATE_RPM       15.0f

/** Right shift speed while searching for center line */
#define FSM_SHIFT_RIGHT_RPM        15.0f

/** Small rotational correction applied during right shift */
#define FSM_SHIFT_ROTATE_RPM        6.0f

/** Re-align with TURN_ALIGN when left USS mismatch exceeds this */
#define FSM_SHIFT_REALIGN_DIFF_CM   1.2f

/** Forward speed while approaching hog line */
#define FSM_FORWARD_TO_HOG_RPM      18.0f

/** Lateral correction magnitude for simple line following */
#define FSM_LINE_FOLLOW_STRAFE_RPM   8.0f

/** Front USS target distance to trigger final return maneuver (cm) */
#define FSM_RETURN_FRONT_TARGET_CM   15.0f

/** Rotation speed during hardcoded 180-degree turn */
#define FSM_RETURN_TURN_RPM          15.0f

/** Hardcoded duration for 180-degree turn (ms, tune on robot) */
#define FSM_RETURN_TURN_180_MS     1300UL

/** Left strafe speed for final lateral move */
#define FSM_RETURN_SHIFT_LEFT_RPM    15.0f

/** Hardcoded duration for ~30 cm left strafe (ms, tune on robot) */
#define FSM_RETURN_SHIFT_LEFT_30CM_MS 1800UL

#endif /* STATE_MACHINE_CONFIG_H */
