#pragma once

#include <Arduino.h>

void initLineSensor();
void updateLineSensor();
uint8_t lineMask5b();
bool isAllBlack();
bool isCentered();
