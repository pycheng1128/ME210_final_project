#pragma once

// Ultrasonic sensor module API (matches current src/uss.cpp).

void initUss();
void updateUss();

float ussLeftFrontCm();
float ussLeftRearCm();
float ussFrontCm();

bool ussLeftFrontTriggered();
bool ussLeftRearTriggered();
bool ussFrontTriggered();
