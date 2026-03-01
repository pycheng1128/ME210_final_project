/**
 * @file mobility_driver.h
 * @brief Public API for 4-wheel Mecanum mobility system
 *
 * Features:
 *   - Mecanum inverse kinematics (forward/strafe/rotate)
 *   - PI speed control with anti-windup (conditional integration)
 *   - Deadzone compensation (minimum PWM feedforward)
 *   - First-order low-pass filter on measured RPM
 *   - E-Stop input: immediate shutdown, recoverable
 *   - Safety brake on direction reversal
 *   - Non-blocking 100 ms serial debug output
 *
 * Usage:
 *   1. Call Mobility_Init() in setup()
 *   2. Drive with Mobility_Drive(vx, vy, rotation), or use
 *      convenience functions (Mobility_MoveForward, etc.)
 *      Alternatively, use per-motor Mobility_SetMotorSpeed().
 *   3. Call Mobility_Update() every loop() iteration
 *          (PID runs internally at MOB_PID_INTERVAL_MS,
 *           debug prints at MOB_DEBUG_INTERVAL_MS)
 *   4. Read measured speed with Mobility_GetRPM()
 */

#ifndef MOBILITY_DRIVER_H
#define MOBILITY_DRIVER_H

#include <Arduino.h>
#include "pin_map.h"
#include "mobility_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =====================================================================
 *  Motor Index Constants (0-based)
 *
 *  Wheel layout (top view, front of robot facing up):
 *
 *        FRONT
 *    ┌────────────┐
 *    │ FL(0) FR(1)│
 *    │   //  \\   │   Mecanum rollers form an O pattern
 *    │   \\  //   │
 *    │ BL(2) BR(3)│
 *    └────────────┘
 *        BACK
 *
 *  Motor 1 = Front-Left  (FL)
 *  Motor 2 = Front-Right (FR)
 *  Motor 3 = Back-Left   (BL)
 *  Motor 4 = Back-Right  (BR)
 * ===================================================================== */
#define MOB_MOTOR_1     0
#define MOB_MOTOR_2     1
#define MOB_MOTOR_3     2
#define MOB_MOTOR_4     3

/* Readable position aliases */
#define MOB_FL          MOB_MOTOR_1
#define MOB_FR          MOB_MOTOR_2
#define MOB_BL          MOB_MOTOR_3
#define MOB_BR          MOB_MOTOR_4

/* =====================================================================
 *  Motor Direction Enum
 * ===================================================================== */
typedef enum {
    MOB_DIR_STOP = 0,
    MOB_DIR_CW,
    MOB_DIR_CCW
} MobDirection_t;

/* =====================================================================
 *  Per-Motor State Structure
 * ===================================================================== */
typedef struct {
    /* Pin assignments */
    uint8_t pinIN1;
    uint8_t pinIN2;
    uint8_t pinENA;
    uint8_t pinENCA;
    uint8_t pinENCB;

    /* Encoder state */
    volatile long encoderCount;
    long          prevCount;

    /* Speed measurement */
    float rawRPM;               /**< Unfiltered RPM */
    float filteredRPM;          /**< Low-pass filtered RPM */

    /* PID state */
    float setpointRPM;
    float integral;
    float prevError;
    int   lastPWM;              /**< Last applied PWM value (for debug) */

    /* Direction / safety */
    MobDirection_t direction;
    bool  isReversing;
    float pendingSetpointRPM;
} MobMotor_t;

/* =====================================================================
 *  Public API
 * ===================================================================== */

/**
 * @brief Initialize all motor driver pins, encoder interrupts,
 *        E-Stop pin, and internal state.  Call once in setup().
 */
void Mobility_Init(void);

/**
 * @brief Master update — call every loop() iteration.
 *        Internally handles:
 *          - PID control at MOB_PID_INTERVAL_MS
 *          - E-Stop polling
 *          - Debug serial prints at MOB_DEBUG_INTERVAL_MS
 */
void Mobility_Update(void);

/**
 * @brief Set target speed for a single motor.
 * @param motorIdx  Motor index (MOB_MOTOR_1 .. MOB_MOTOR_4)
 * @param rpm       Signed target RPM (+CW, -CCW, 0=stop)
 */
void Mobility_SetMotorSpeed(uint8_t motorIdx, float rpm);

/**
 * @brief Set target speed for all four motors at once.
 */
void Mobility_SetAllMotors(float rpm1, float rpm2, float rpm3, float rpm4);

/**
 * @brief Immediately stop a single motor (brake).
 */
void Mobility_StopMotor(uint8_t motorIdx);

/**
 * @brief Immediately stop all motors.
 */
void Mobility_StopAll(void);

/**
 * @brief Get low-pass filtered RPM for a motor.
 */
float Mobility_GetRPM(uint8_t motorIdx);

/**
 * @brief Get raw (unfiltered) RPM for a motor.
 */
float Mobility_GetRawRPM(uint8_t motorIdx);

/**
 * @brief Get the encoder tick count for a motor.
 */
long Mobility_GetEncoderCount(uint8_t motorIdx);

/**
 * @brief Reset encoder count for a motor to zero.
 */
void Mobility_ResetEncoder(uint8_t motorIdx);

/**
 * @brief Check whether E-Stop is currently active.
 * @return true if E-Stop is engaged.
 */
bool Mobility_IsEStopped(void);

/* =====================================================================
 *  Mecanum Drive API
 *
 *  Coordinate system (robot-centric):
 *    vx       > 0 = forward,     < 0 = backward
 *    vy       > 0 = strafe left, < 0 = strafe right
 *    rotation > 0 = CCW (left),  < 0 = CW (right)
 *
 *  All speed values are in RPM.  The mixing function computes
 *  per-wheel RPMs using standard Mecanum inverse kinematics:
 *    FL = vx - vy - rotation
 *    FR = vx + vy + rotation
 *    BL = vx + vy - rotation
 *    BR = vx - vy + rotation
 *  Results are scaled so no wheel exceeds MOB_MAX_RPM.
 * ===================================================================== */

/**
 * @brief General Mecanum drive — set forward, strafe, and rotation.
 * @param vx        Forward/backward speed in RPM (+forward)
 * @param vy        Left/right strafe speed in RPM (+left)
 * @param rotation  Rotational speed in RPM (+CCW)
 */
void Mobility_Drive(float vx, float vy, float rotation);

/** @brief Move forward at the given RPM. */
void Mobility_MoveForward(float rpm);

/** @brief Move backward at the given RPM. */
void Mobility_MoveBackward(float rpm);

/** @brief Strafe left at the given RPM. */
void Mobility_StrafeLeft(float rpm);

/** @brief Strafe right at the given RPM. */
void Mobility_StrafeRight(float rpm);

/** @brief Rotate counter-clockwise at the given RPM. */
void Mobility_RotateCCW(float rpm);

/** @brief Rotate clockwise at the given RPM. */
void Mobility_RotateCW(float rpm);

#ifdef __cplusplus
}
#endif

#endif /* MOBILITY_DRIVER_H */
