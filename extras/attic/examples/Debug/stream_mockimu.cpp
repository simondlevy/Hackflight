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

    static float _x;
    static int8_t _dir;

    _dir = _dir == 0 ? +1
          : _x > .1 ? -1
          : _x < -.1 ? +1
          : _dir;

    _x += .001 * _dir;

    stream_imuQuaternionW = 1;
    stream_imuQuaternionX = _x;
    stream_imuQuaternionY = 0.05;
    stream_imuQuaternionZ = 0;
}
