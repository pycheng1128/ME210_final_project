/**
 * @file test_stepper.cpp
 * @brief Interactive test for the StepperMotor library (Timer3 ISR-driven)
 *
 * Serial commands (115200 baud):
 *   f  — Fire launch cycle (CW, default steps)
 *   b  — Fire launch cycle (CCW, default steps)
 *   s  — Stop stepper immediately
 *   +  — Increase step frequency by 50 Hz
 *   -  — Decrease step frequency by 50 Hz
 *   i  — Print current status (busy, step freq, target steps)
 *   h  — Print help
 *
 * Enable by adding to platformio.ini build_flags:
 *   -DENABLE_STEPPER_TEST_APP=1
 *
 * The test will print "[STEPPER] Cycle complete!" when a launch
 * cycle finishes, polling checkStepperLaunchCycleComplete().
 */

#include <Arduino.h>
#include <stepper_motor.h>
#include <stepper_config.h>

#ifndef ENABLE_STEPPER_TEST_APP
#define ENABLE_STEPPER_TEST_APP 1
#endif

#if ENABLE_STEPPER_TEST_APP

/* ── Adjustable parameters (runtime via serial) ─────────────────── */

static uint16_t g_step_freq   = STEPPER_STEP_FREQ_HZ;
static uint16_t g_total_steps = STEPPER_LAUNCH_STEPS;

/* ── Helpers ─────────────────────────────────────────────────────── */

static void printHelp() {
    Serial.println(F(""));
    Serial.println(F("=== StepperMotor Test ==="));
    Serial.println(F("  f — Fire CW launch cycle"));
    Serial.println(F("  b — Fire CCW launch cycle"));
    Serial.println(F("  s — Stop immediately"));
    Serial.println(F("  + — Step freq +50 Hz"));
    Serial.println(F("  - — Step freq -50 Hz"));
    Serial.println(F("  ] — Steps +100"));
    Serial.println(F("  [ — Steps -100"));
    Serial.println(F("  i — Print status"));
    Serial.println(F("  h — This help"));
    Serial.println(F("========================"));
}

static void printStatus() {
    Serial.print(F("[STEPPER] Freq="));
    Serial.print(g_step_freq);
    Serial.print(F(" Hz  Steps="));
    Serial.print(g_total_steps);
    Serial.print(F("  Busy="));
    Serial.println(isStepperMotorBusy() ? F("yes") : F("no"));
}

/* ── Arduino entry points ───────────────────────────────────────── */

void setup() {
    Serial.begin(115200);
    delay(200);

    initStepperMotor();

    Serial.println(F("[STEPPER] Test ready."));
    printHelp();
    printStatus();
}

void loop() {
    // Poll for cycle completion.
    if (checkStepperLaunchCycleComplete()) {
        Serial.println(F("[STEPPER] Cycle complete!"));
    }

    // Handle serial commands.
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'f':
                Serial.println(F(">> Fire CW"));
                startStepperLaunchCycle(true, g_total_steps);
                break;

            case 'b':
                Serial.println(F(">> Fire CCW"));
                startStepperLaunchCycle(false, g_total_steps);
                break;

            case 's':
                Serial.println(F(">> STOP"));
                stopStepperMotor();
                break;

            case '+':
                g_step_freq += 50;
                Serial.print(F(">> Freq -> "));
                Serial.print(g_step_freq);
                Serial.println(F(" Hz"));
                break;

            case '-':
                if (g_step_freq > 50) g_step_freq -= 50;
                Serial.print(F(">> Freq -> "));
                Serial.print(g_step_freq);
                Serial.println(F(" Hz"));
                break;

            case ']':
                g_total_steps += 100;
                Serial.print(F(">> Steps -> "));
                Serial.println(g_total_steps);
                break;

            case '[':
                if (g_total_steps > 100) g_total_steps -= 100;
                Serial.print(F(">> Steps -> "));
                Serial.println(g_total_steps);
                break;

            case 'i':
                printStatus();
                break;

            case 'h':
                printHelp();
                break;

            default:
                break;
        }
    }
}

#endif  // ENABLE_STEPPER_TEST_APP
