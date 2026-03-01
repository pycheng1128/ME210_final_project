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
    Mobility_Init();     /* Initializes Serial, pins, ISRs, E-Stop */
    printHelp();
}

void loop() {
    /* PID + E-Stop + debug prints happen inside Update() */
    Mobility_Update();

    /* Handle serial commands */
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'w':
                Serial.println(F(">> Forward"));
                Mobility_MoveForward(TEST_DRIVE_RPM);
                break;

            case 's':
                Serial.println(F(">> Backward"));
                Mobility_MoveBackward(TEST_DRIVE_RPM);
                break;

            case 'a':
                Serial.println(F(">> Strafe Left"));
                Mobility_StrafeLeft(TEST_ROTATE_RPM);
                break;

            case 'd':
                Serial.println(F(">> Strafe Right"));
                Mobility_StrafeRight(TEST_ROTATE_RPM);
                break;

            case 'q':
                Serial.println(F(">> Rotate CCW"));
                Mobility_RotateCCW(TEST_ROTATE_RPM);
                break;

            case 'e':
                Serial.println(F(">> Rotate CW"));
                Mobility_RotateCW(TEST_ROTATE_RPM);
                break;

            case 'x':
                Serial.println(F(">> Diagonal (vx=30, vy=20, rot=0)"));
                Mobility_Drive(30.0, 20.0, 0);
                break;

            /* --- Individual Motor Diagnostics --- */
            case '1':
                Serial.println(F(">> Testing: FRONT-LEFT (Motor 1)"));
                Mobility_StopAll();
                Mobility_SetMotorSpeed(MOB_MOTOR_1, TEST_DRIVE_RPM);
                break;
            case '2':
                Serial.println(F(">> Testing: FRONT-RIGHT (Motor 2)"));
                Mobility_StopAll();
                Mobility_SetMotorSpeed(MOB_MOTOR_2, TEST_DRIVE_RPM);
                break;
            case '3': 
                Serial.println(F(">> Testing: BACK-LEFT (Motor 3)")); 
                Mobility_StopAll(); Mobility_SetMotorSpeed(MOB_MOTOR_3, TEST_DRIVE_RPM); 
                break;
            case '4': 
                Serial.println(F(">> Testing: BACK-RIGHT (Motor 4)")); 
                Mobility_StopAll(); Mobility_SetMotorSpeed(MOB_MOTOR_4, TEST_DRIVE_RPM); 
                break;
                Serial.println(F(">> Test Motor 4 (BR)"));
                Mobility_StopAll();
                Mobility_SetMotorSpeed(MOB_BR, TEST_DRIVE_RPM);
                break;

            case 'p':
                Serial.println(F(">> STOP"));
                Mobility_StopAll();
                break;

            case 'r':
                Serial.println(F(">> Encoders reset"));
                for (uint8_t i = 0; i < MOB_NUM_MOTORS; i++)
                    Mobility_ResetEncoder(i);
                break;

            case 'h':
                printHelp();
                break;

            default:
                break;
        }
    }
}
