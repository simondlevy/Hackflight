/*
   Declarations for functions called directly by Arduino setup() and loop()

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#pragma once

#include <stdint.h>

void copilot_startLed(uint8_t pin);

void copilot_startReceiver(void);

void copilot_updateReceiver(void);

void copilot_startSerial();

void copilot_updateSerial();

void copilot_startImu();

void copilot_updateImu();
