/*
   Support for USFS IMU in Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */


#define _EXTERN
#include "copilot.h"

#include <Wire.h>
#include <USFS_Master.h>

static USFS_Master _usfs;

void copilot_startUsfs(void)
{
    delay(100);
    if (!_usfs.begin()) {
        while (true) {
            Serial.println(_usfs.getErrorString());
            delay(500);
        }
    }
}

void copilot_updateUsfs(void)
{
    _usfs.checkEventStatus();

    if (_usfs.gotGyrometer()) {
        _usfs.readGyrometer(
                copilot_gyrometerX,
                copilot_gyrometerY,
                copilot_gyrometerZ);
    }

    if (_usfs.gotQuaternion()) {
        _usfs.readQuaternion(
                copilot_quaternionW,
                copilot_quaternionX,
                copilot_quaternionY,
                copilot_quaternionZ);
    }
}
