/*
   Hackflight stream-based serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#pragma once

#ifdef NONEXTERN
#define EXTERN
#else
#define EXTERN extern
#endif

#include <stdint.h>

EXTERN uint8_t stream_serialByte; 

void stream_startSerial(void);

void stream_serialWrite(uint8_t byte);

void stream_serialRead(void);
