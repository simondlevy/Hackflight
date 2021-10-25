/*
   Hackflight stream-based serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#pragma once

#include <stdint.h>

void stream_startSerial(void);

void stream_serialWrite(uint8_t byte);

void stream_serialRead(void);

void stream_serialUpdate(void);
