#pragma once

#include <Arduino.h>

void initUss();
void updateUss();

float ussLeftFrontCm();
float ussLeftRearCm();
float ussFrontCm();

bool ussLeftFrontTriggered();
bool ussLeftRearTriggered();
bool ussFrontTriggered();
