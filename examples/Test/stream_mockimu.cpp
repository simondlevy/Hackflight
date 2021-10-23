/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */


#define _EXTERN
#include "copilot.h"

#include <Arduino.h>

void stream_updateImu(void)
{
    stream_imuGotQuaternion = true;

    stream_imuQuaternionW = 1;
    stream_imuQuaternionX = 0;
    stream_imuQuaternionY = 0;
    stream_imuQuaternionZ = 0;
}
