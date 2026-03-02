/**
 * @file mobility_driver.cpp
 * @brief Implementation of the 4-motor mobility driver HAL
 *
 * Architecture:
 *   ┌─────────────────────────────────────────────────┐
 *   │              Mobility_Update()                  │
 *   │  called every loop() — internally rate-limited  │
 *   ├─────────────────────────────────────────────────┤
 *   │  1. E-Stop check (every call)                   │
 *   │  2. PID loop (every MOB_PID_INTERVAL_MS)        │
 *   │     a. Read encoder counts (atomic)             │
 *   │     b. Compute raw RPM → low-pass filter        │
 *   │     c. Safety brake logic                       │
 *   │     d. PI error → integral (conditional windup) │
 *   │     e. PWM = clamp(|pidOut|) + deadzone comp    │
 *   │     f. Apply direction + PWM to driver          │
 *   │  3. Debug print (every MOB_DEBUG_INTERVAL_MS)   │
 *   └─────────────────────────────────────────────────┘
 *
 *   Encoder ISRs (4×):
 *     - Minimal: increment/decrement volatile counter
 *     - No Serial, no floating point, no long operations
 */

#include "mobility_driver.h"
#include <math.h>

/* =====================================================================
 *  Module-Private Data
 * ===================================================================== */

/** Array of per-motor state — indexed by MOB_MOTOR_1 .. MOB_MOTOR_4 */
static MobMotor_t motors[MOB_NUM_MOTORS];

/** Timestamp of last PID update */
static unsigned long prevPIDTime   = 0;

/** Timestamp of last debug print */
static unsigned long prevDebugTime = 0;

/** E-Stop state flag */
static volatile bool eStopActive   = false;

/* =====================================================================
 *  Pin Lookup Tables (indexed by motor number)
 * ===================================================================== */
static const uint8_t pinIN1[MOB_NUM_MOTORS]  = {
    MOB_DRIVER_1_IN1, MOB_DRIVER_2_IN1, MOB_DRIVER_3_IN1, MOB_DRIVER_4_IN1
};
static const uint8_t pinIN2[MOB_NUM_MOTORS]  = {
    MOB_DRIVER_1_IN2, MOB_DRIVER_2_IN2, MOB_DRIVER_3_IN2, MOB_DRIVER_4_IN2
};
static const uint8_t pinENA[MOB_NUM_MOTORS]  = {
    MOB_DRIVER_1_ENA, MOB_DRIVER_2_ENA, MOB_DRIVER_3_ENA, MOB_DRIVER_4_ENA
};
static const uint8_t pinENCA[MOB_NUM_MOTORS] = {
    MOB_MOTOR_1_ENCA, MOB_MOTOR_2_ENCA, MOB_MOTOR_3_ENCA, MOB_MOTOR_4_ENCA
};
static const uint8_t pinENCB[MOB_NUM_MOTORS] = {
    MOB_MOTOR_1_ENCB, MOB_MOTOR_2_ENCB, MOB_MOTOR_3_ENCB, MOB_MOTOR_4_ENCB
};

/* =====================================================================
 *  Encoder ISRs
 *
 *  RULES:  - Only do counter increment/decrement
 *          - No Serial, no float, no long computation
 *          - Uses 'volatile' counter in MobMotor_t
 * ===================================================================== */

static void encoderISR_Motor1(void) {
    motors[MOB_MOTOR_1].encoderCount +=
        (digitalRead(MOB_MOTOR_1_ENCB) == HIGH) ? 1 : -1;
}

static void encoderISR_Motor2(void) {
    motors[MOB_MOTOR_2].encoderCount +=
        (digitalRead(MOB_MOTOR_2_ENCB) == HIGH) ? 1 : -1;
}

static void encoderISR_Motor3(void) {
    motors[MOB_MOTOR_3].encoderCount +=
        (digitalRead(MOB_MOTOR_3_ENCB) == HIGH) ? 1 : -1;
}

static void encoderISR_Motor4(void) {
    motors[MOB_MOTOR_4].encoderCount +=
        (digitalRead(MOB_MOTOR_4_ENCB) == HIGH) ? 1 : -1;
}

/** ISR function pointer table */
static void (*const encoderISRs[MOB_NUM_MOTORS])(void) = {
    encoderISR_Motor1,
    encoderISR_Motor2,
    encoderISR_Motor3,
    encoderISR_Motor4
};

/* =====================================================================
 *  Private Helper: Force Stop One Motor (PWM=0, clear PID state)
 * ===================================================================== */

static void forceStopMotor(uint8_t idx) {
    MobMotor_t *m = &motors[idx];
    digitalWrite(m->pinIN1, LOW);
    digitalWrite(m->pinIN2, LOW);
    analogWrite(m->pinENA, 0);
    m->integral  = 0;
    m->prevError = 0;
    m->lastPWM   = 0;
}

/* =====================================================================
 *  Private Helper: Apply Direction + PWM (with deadzone compensation)
 * ===================================================================== */

/* Pre-apply inversion coefficients from config */
static const float motorInverts[MOB_NUM_MOTORS] = {
    MOB_MOTOR_1_INV, MOB_MOTOR_2_INV, MOB_MOTOR_3_INV, MOB_MOTOR_4_INV
};

static void applyMotorOutput(uint8_t idx, float pidOutput) {
    MobMotor_t *m = &motors[idx];

    /* Factor in the software inversion */
    float invertedOutput = pidOutput * motorInverts[idx];

    int pwmVal = abs((int)invertedOutput);

    /* Deadzone compensation */
    if (pwmVal > 0 && pwmVal < MOB_PWM_DEADZONE) {
        pwmVal = MOB_PWM_DEADZONE;
    }

    pwmVal = constrain(pwmVal, 0, MOB_PWM_MAX);

    /* Set H-bridge direction */
    if (invertedOutput >= 0) {
        digitalWrite(m->pinIN1, HIGH);
        digitalWrite(m->pinIN2, LOW);
    } else {
        digitalWrite(m->pinIN1, LOW);
        digitalWrite(m->pinIN2, HIGH);
    }

    analogWrite(m->pinENA, pwmVal);
    
    /* signed for debug: indicates intended direction after inversion */
    m->lastPWM = (invertedOutput >= 0) ? pwmVal : -pwmVal;
}

/* =====================================================================
 *  Private Helper: E-Stop Check
 * ===================================================================== */

static void checkEStop(void) {
    bool triggered = (digitalRead(MOB_ESTOP_PIN) == MOB_ESTOP_ACTIVE_LEVEL);

    if (triggered && !eStopActive) {
        /* Rising edge: just triggered */
        eStopActive = true;
        Mobility_StopAll();
#if MOB_DEBUG_PRINT
        Serial.println(F("[MOB] *** E-STOP ACTIVATED *** All motors stopped."));
#endif
    } else if (!triggered && eStopActive) {
        /* Falling edge: released */
        eStopActive = false;
        /* Reset PID timer and encoder counters to prevent integral windup from huge dt */
        prevPIDTime = millis();
        for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) {
            noInterrupts();
            motors[i].prevCount = motors[i].encoderCount;
            interrupts();
        }
#if MOB_DEBUG_PRINT
        Serial.println(F("[MOB] E-Stop released. Control resumed."));
#endif
    }
}

/* =====================================================================
 *  Private Helper: Debug Print (100 ms interval)
 *
 *  Format per motor:  [M1] Set=XX.X Act=XX.X PWM=XXX Err=XX.X
 * ===================================================================== */

static void debugPrint(void) {
#if MOB_DEBUG_PRINT
    unsigned long now = millis();
    if (now - prevDebugTime < MOB_DEBUG_INTERVAL_MS) return;
    prevDebugTime = now;

    static const char* const labels[MOB_NUM_MOTORS] = {"FL", "FR", "BL", "BR"};

    for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) {
        MobMotor_t *m = &motors[i];
        float error = m->setpointRPM - m->filteredRPM;

        Serial.print(F("["));
        Serial.print(labels[i]);
        Serial.print(F("] Set="));
        Serial.print(m->setpointRPM, 1);
        Serial.print(F(" Act="));
        Serial.print(m->filteredRPM, 1);
        Serial.print(F(" PWM="));
        Serial.print(m->lastPWM);
        Serial.print(F(" Err="));
        Serial.print(error, 1);

        if (i < MOB_NUM_MOTORS - 1)
            Serial.print(F("  |  "));
    }
    Serial.println();
#endif
}

/* =====================================================================
 *  Public API: Initialization
 * ===================================================================== */

void Mobility_Init(void) {
    for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) {
        MobMotor_t *m = &motors[i];

        /* Store pin assignments */
        m->pinIN1  = pinIN1[i];
        m->pinIN2  = pinIN2[i];
        m->pinENA  = pinENA[i];
        m->pinENCA = pinENCA[i];
        m->pinENCB = pinENCB[i];

        /* Configure driver outputs */
        pinMode(m->pinIN1, OUTPUT);
        pinMode(m->pinIN2, OUTPUT);
        pinMode(m->pinENA, OUTPUT);
        digitalWrite(m->pinIN1, LOW);
        digitalWrite(m->pinIN2, LOW);
        analogWrite(m->pinENA, 0);

        /* Configure encoder inputs */
        pinMode(m->pinENCA, INPUT_PULLUP);
        pinMode(m->pinENCB, INPUT_PULLUP);

        /* Zero state */
        m->encoderCount       = 0;
        m->prevCount          = 0;
        m->rawRPM             = 0;
        m->filteredRPM        = 0;
        m->setpointRPM        = 0;
        m->integral           = 0;
        m->prevError          = 0;
        m->lastPWM            = 0;
        m->direction          = MOB_DIR_STOP;
        m->isReversing        = false;
        m->pendingSetpointRPM = 0;

        /* Attach encoder interrupt (RISING edge on ENCA) */
        attachInterrupt(digitalPinToInterrupt(m->pinENCA),
                        encoderISRs[i], RISING);
    }

    /* E-Stop pin: INPUT_PULLUP (button shorts to GND when pressed) */
    pinMode(MOB_ESTOP_PIN, INPUT_PULLUP);
    eStopActive = false;

    prevPIDTime   = millis();
    prevDebugTime = millis();

#if MOB_DEBUG_PRINT
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("======================================"));
    Serial.println(F("[MOB] Mobility driver initialized"));
    Serial.println(F("      4 motors, PI control, E-Stop"));
    Serial.print(F("      Kp="));  Serial.print(MOB_PID_KP);
    Serial.print(F("  Ki="));      Serial.print(MOB_PID_KI);
    Serial.print(F("  Kd="));      Serial.println(MOB_PID_KD);
    Serial.print(F("      LPF alpha="));  Serial.println(MOB_LPF_ALPHA);
    Serial.print(F("      Deadzone PWM="));Serial.println(MOB_PWM_DEADZONE);
    Serial.println(F("======================================"));
#endif
}

/* =====================================================================
 *  Public API: PID Update (call every loop iteration)
 * ===================================================================== */

void Mobility_Update(void) {
    /* --- E-Stop: check every call (instantaneous response) --- */
    checkEStop();
    if (eStopActive) return;    /* Skip PID while E-Stop is active */

    /* --- Rate-limit PID --- */
    unsigned long now = millis();
    unsigned long dt  = now - prevPIDTime;
    if (dt < MOB_PID_INTERVAL_MS) {
        /* Still print debug even if PID hasn't run */
        debugPrint();
        return;
    }
    prevPIDTime = now;
    float dt_sec = dt / 1000.0f;

    for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) {
        MobMotor_t *m = &motors[i];

        /* --- 1. Read encoder count (atomic) --- */
        noInterrupts();
        long count = m->encoderCount;
        interrupts();

        long deltaCounts = count - m->prevCount;
        m->prevCount = count;

        /* --- 2. Compute raw RPM (factor in software inversion) --- */
        float pulsesPerSec = (float)deltaCounts * 1000.0f / (float)dt;
        float measuredRPM  = (pulsesPerSec * 60.0f) / MOB_ENCODER_PPR;
        
        /* Ensure the measured speed direction matches the software's setpoint direction */
        m->rawRPM = measuredRPM * motorInverts[i];

        /* --- 3. Low-pass filter for measurement noise smoothing --- */
        m->filteredRPM += MOB_LPF_ALPHA * (m->rawRPM - m->filteredRPM);

        /* --- 4. Safety brake: stop before reversing --- */
        if (m->isReversing) {
            if (fabs(m->filteredRPM) < MOB_STOPPED_RPM_THRESH) {
                m->setpointRPM = m->pendingSetpointRPM;
                m->isReversing = false;
            } else {
                m->setpointRPM = 0;     /* Force braking */
            }
        }

        /* --- 5. PI(D) Calculation --- */
        float error = m->setpointRPM - m->filteredRPM;

        /* Conditional integration (anti-windup):
         * Do NOT accumulate integral if:
         *   - PID output is saturated (|output| >= PWM_MAX), AND
         *   - error has the same sign as the integral (making it worse)
         * This prevents the integral from growing when the actuator
         * is already at its limit. */
        float tentativeIntegral = m->integral + error * dt_sec;
        bool saturated = (fabs(m->integral * MOB_PID_KI) >= MOB_PWM_MAX);
        bool sameSign  = ((error > 0 && m->integral > 0) ||
                          (error < 0 && m->integral < 0));

        if (!(saturated && sameSign)) {
            m->integral = tentativeIntegral;
        }
        /* Clamp integral as a hard safety net */
        m->integral = constrain(m->integral,
                                -MOB_PID_INTEGRAL_MAX, MOB_PID_INTEGRAL_MAX);

        float derivative = 0;
        if (MOB_PID_KD != 0.0f) {
            derivative = (dt_sec > 0) ? (error - m->prevError) / dt_sec : 0;
        }
        m->prevError = error;

        float pidOut = MOB_PID_KP * error
                     + MOB_PID_KI * m->integral
                     + MOB_PID_KD * derivative;

        /* --- 6. Apply output --- */
        if (m->direction != MOB_DIR_STOP) {
            applyMotorOutput(i, pidOut);
        } else {
            forceStopMotor(i);
        }
    }

    /* --- Debug print (separate rate) --- */
    debugPrint();
}

/* =====================================================================
 *  Public API: Set Speed
 * ===================================================================== */

void Mobility_SetMotorSpeed(uint8_t motorIdx, float rpm) {
    if (motorIdx >= MOB_NUM_MOTORS) return;

    MobMotor_t *m = &motors[motorIdx];

    /* Determine requested direction */
    MobDirection_t newDir;
    if (rpm > 0)       newDir = MOB_DIR_CW;
    else if (rpm < 0)  newDir = MOB_DIR_CCW;
    else               newDir = MOB_DIR_STOP;

    /* Direction reversal safety brake */
    bool needsBrake = false;
    if (m->direction == MOB_DIR_CW  && newDir == MOB_DIR_CCW &&
        m->filteredRPM > MOB_REVERSAL_RPM_THRESH)
        needsBrake = true;
    if (m->direction == MOB_DIR_CCW && newDir == MOB_DIR_CW  &&
        m->filteredRPM < -MOB_REVERSAL_RPM_THRESH)
        needsBrake = true;

    if (needsBrake) {
        m->isReversing        = true;
        m->pendingSetpointRPM = rpm;
    } else {
        m->setpointRPM = rpm;
    }

    m->direction = newDir;
}

void Mobility_SetAllMotors(float rpm1, float rpm2, float rpm3, float rpm4) {
    Mobility_SetMotorSpeed(MOB_MOTOR_1, rpm1);
    Mobility_SetMotorSpeed(MOB_MOTOR_2, rpm2);
    Mobility_SetMotorSpeed(MOB_MOTOR_3, rpm3);
    Mobility_SetMotorSpeed(MOB_MOTOR_4, rpm4);
}

/* =====================================================================
 *  Public API: Stop
 * ===================================================================== */

void Mobility_StopMotor(uint8_t motorIdx) {
    if (motorIdx >= MOB_NUM_MOTORS) return;

    MobMotor_t *m   = &motors[motorIdx];
    m->direction    = MOB_DIR_STOP;
    m->setpointRPM  = 0;
    m->integral     = 0;
    m->isReversing  = false;
    forceStopMotor(motorIdx);
}

void Mobility_StopAll(void) {
    for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) {
        Mobility_StopMotor(i);
    }
}

/* =====================================================================
 *  Public API: Getters
 * ===================================================================== */

float Mobility_GetRPM(uint8_t motorIdx) {
    if (motorIdx >= MOB_NUM_MOTORS) return 0;
    return motors[motorIdx].filteredRPM;
}

float Mobility_GetRawRPM(uint8_t motorIdx) {
    if (motorIdx >= MOB_NUM_MOTORS) return 0;
    return motors[motorIdx].rawRPM;
}

long Mobility_GetEncoderCount(uint8_t motorIdx) {
    if (motorIdx >= MOB_NUM_MOTORS) return 0;
    noInterrupts();
    long c = motors[motorIdx].encoderCount;
    interrupts();
    return c;
}

void Mobility_ResetEncoder(uint8_t motorIdx) {
    if (motorIdx >= MOB_NUM_MOTORS) return;
    noInterrupts();
    motors[motorIdx].encoderCount = 0;
    interrupts();
    motors[motorIdx].prevCount   = 0;
    motors[motorIdx].filteredRPM = 0;
    motors[motorIdx].rawRPM      = 0;
}

bool Mobility_IsEStopped(void) {
    return eStopActive;
}

/* =====================================================================
 *  Mecanum Drive API
 *
 *  X-type Mecanum inverse kinematics:
 *    FL = vx - vy - rotation
 *    FR = vx + vy + rotation
 *    BL = vx + vy - rotation
 *    BR = vx - vy + rotation
 *
 *  If any wheel speed exceeds MOB_MAX_RPM, all wheels are scaled
 *  proportionally to preserve the motion vector.
 * ===================================================================== */

void Mobility_Drive(float vx, float vy, float rotation) {
    /* Compute individual wheel RPMs for X-type configuration:
     *  FL = vx - vy - rotation
     *  FR = vx + vy + rotation
     *  BL = vx + vy - rotation
     *  BR = vx - vy + rotation
     */
    float fl = vx - vy - rotation;
    float fr = vx + vy + rotation;
    float bl = vx + vy - rotation;
    float br = vx - vy + rotation;

    /* Auto-scale: if any wheel exceeds MAX_RPM, scale all down */
    float maxVal = fabs(fl);
    if (fabs(fr) > maxVal) maxVal = fabs(fr);
    if (fabs(bl) > maxVal) maxVal = fabs(bl);
    if (fabs(br) > maxVal) maxVal = fabs(br);

    if (maxVal > MOB_MAX_RPM) {
        float scale = MOB_MAX_RPM / maxVal;
        fl *= scale;
        fr *= scale;
        bl *= scale;
        br *= scale;
    }

    /* Apply to individual motors */
    Mobility_SetMotorSpeed(MOB_FL, fl);
    Mobility_SetMotorSpeed(MOB_FR, fr);
    Mobility_SetMotorSpeed(MOB_BL, bl);
    Mobility_SetMotorSpeed(MOB_BR, br);
}

void Mobility_MoveForward(float rpm) {
    Mobility_Drive(rpm, 0, 0);
}

void Mobility_MoveBackward(float rpm) {
    Mobility_Drive(-rpm, 0, 0);
}

void Mobility_StrafeLeft(float rpm) {
    Mobility_Drive(0, rpm, 0);
}

void Mobility_StrafeRight(float rpm) {
    Mobility_Drive(0, -rpm, 0);
}

void Mobility_RotateCCW(float rpm) {
    Mobility_Drive(0, 0, rpm);
}

void Mobility_RotateCW(float rpm) {
    Mobility_Drive(0, 0, -rpm);
}
