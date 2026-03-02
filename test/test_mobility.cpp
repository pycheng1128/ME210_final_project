/**
 * @file test_mobility.cpp
 * @brief Interactive test for the MobilitySystem library (Mecanum drive)
 *
 * Serial commands (115200 baud):
 *   w  — Move forward   (50 RPM)
 *   s  — Move backward  (50 RPM)
 *   a  — Strafe left    (50 RPM)
 *   d  — Strafe right   (50 RPM)
 *   q  — Rotate CCW     (30 RPM)
 *   e  — Rotate CW      (30 RPM)
 *   x  — Drive diagonal (vx=30, vy=20, rot=0)
 *   p  — Stop all motors
 *   r  — Reset all encoders
 *   h  — Print help
 *
 * Debug output prints every 100 ms:
 *   [FL] Set=XX Act=XX PWM=XX Err=XX | [FR] ... | [BL] ... | [BR] ...
 */

#include <Arduino.h>
#include <mobility_driver.h>

#ifndef ENABLE_MOBILITY_TEST_APP
#define ENABLE_MOBILITY_TEST_APP 1
#endif

#if ENABLE_MOBILITY_TEST_APP

/* Test speed (RPM) — adjust as needed */
#define TEST_DRIVE_RPM    30.0f
#define TEST_ROTATE_RPM   15.0f

static void printHelp(void) {
    Serial.println(F(""));
    Serial.println(F("=== MobilitySystem Test ==="));
    Serial.println(F("  w — Forward"));
    Serial.println(F("  s — Backward"));
    Serial.println(F("  a — Strafe Left"));
    Serial.println(F("  d — Strafe Right"));
    Serial.println(F("  q — Rotate CCW"));
    Serial.println(F("  e — Rotate CW"));
    Serial.println(F("  x — Diagonal (forward + left)"));
    Serial.println(F("  1...4 — Test single motor (FL, FR, BL, BR)"));
    Serial.println(F("  p — STOP all"));
    Serial.println(F("  r — Reset encoders"));
    Serial.println(F("  h — This help"));
    Serial.println(F("==========================="));
}

void setup() {
    Serial.begin(115200); 
    Mobility_Init();     /* Initializes pins, ISRs, E-Stop */
    printHelp();
}

/** 
 * Simple state tracking for the test app to avoid redundant calls 
 * and only update speed when the commanded action changes.
 */
enum class TestState { STOP, FORWARD, BACKWARD, LEFT, RIGHT, CCW, CW, DIAGONAL, SINGLE_MOTOR };
static TestState currentTestState = TestState::STOP;

void loop() {
    /* PID + E-Stop + debug prints happen inside Update() */
    Mobility_Update();

    /* Handle serial commands (Events) */
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'w':
                if (currentTestState != TestState::FORWARD) {
                    Serial.println(F(">> Forward"));
                    Mobility_MoveForward(TEST_DRIVE_RPM);
                    currentTestState = TestState::FORWARD;
                }
                break;

            case 's':
                if (currentTestState != TestState::BACKWARD) {
                    Serial.println(F(">> Backward"));
                    Mobility_MoveBackward(TEST_DRIVE_RPM);
                    currentTestState = TestState::BACKWARD;
                }
                break;

            case 'a':
                if (currentTestState != TestState::LEFT) {
                    Serial.println(F(">> Strafe Left"));
                    Mobility_StrafeLeft(TEST_ROTATE_RPM);
                    currentTestState = TestState::LEFT;
                }
                break;

            case 'd':
                if (currentTestState != TestState::RIGHT) {
                    Serial.println(F(">> Strafe Right"));
                    Mobility_StrafeRight(TEST_ROTATE_RPM);
                    currentTestState = TestState::RIGHT;
                }
                break;

            case 'q':
                if (currentTestState != TestState::CCW) {
                    Serial.println(F(">> Rotate CCW"));
                    Mobility_RotateCCW(TEST_ROTATE_RPM);
                    currentTestState = TestState::CCW;
                }
                break;

            case 'e':
                if (currentTestState != TestState::CW) {
                    Serial.println(F(">> Rotate CW"));
                    Mobility_RotateCW(TEST_ROTATE_RPM);
                    currentTestState = TestState::CW;
                }
                break;

            case 'x':
                if (currentTestState != TestState::DIAGONAL) {
                    Serial.println(F(">> Diagonal (vx=30, vy=20, rot=0)"));
                    Mobility_Drive(30.0, 20.0, 0);
                    currentTestState = TestState::DIAGONAL;
                }
                break;

            case 'p':
                if (currentTestState != TestState::STOP) {
                    Serial.println(F(">> STOP"));
                    Mobility_StopAll();
                    currentTestState = TestState::STOP;
                }
                break;

            case '1': case '2': case '3': case '4': {
                    uint8_t mIdx = cmd - '1'; 
                    Serial.print(F(">> Testing motor: ")); Serial.println(mIdx + 1);
                    Mobility_StopAll();
                    Mobility_SetMotorSpeed(mIdx, TEST_DRIVE_RPM);
                    currentTestState = TestState::SINGLE_MOTOR;
                }
                break;

            case 'r':
                Serial.println(F(">> Encoders reset"));
                for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++) Mobility_ResetEncoder(i);
                break;

            case 'h':
                printHelp();
                break;

            default:
                break;
        }
    }
}

#endif  // ENABLE_MOBILITY_TEST_APP
