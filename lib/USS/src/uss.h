#pragma once

#include <Arduino.h>
#include "uss_config.h"

void initUss();
void updateLeftUss(unsigned long timeout_us = USS_LEFT_PULSE_TIMEOUT_DEFAULT_US);
void updateFrontUss();

float ussLeftFrontCm();
float ussLeftRearCm();
float ussFrontCm();

bool ussLeftFrontTriggered();
bool ussLeftRearTriggered();
bool ussFrontTriggered();
