/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */


#define _EXTERN
#include "copilot.h"

#include <Arduino.h>

void stream_startImu(void)
{
}

void stream_updateImu(void)
{
    stream_imuGotGyrometer = true;

    stream_imuGyrometerX = 0;
    stream_imuGyrometerY = 0;
    stream_imuGyrometerZ = 0;

    stream_imuGotQuaternion = true;

    stream_imuQuaternionW = 1;
    stream_imuQuaternionX = 0;
    stream_imuQuaternionY = 0;
    stream_imuQuaternionZ = 0;
}
