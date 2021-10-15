/*
   Streaming support for USFS IMU

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#define _EXTERN
#include "copilot.h"

#define NONEXTERN
#include "stream_imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <USFS_Master.h>

static USFS_Master usfs;

void stream_startImu(void)
{
    // Start the USFS in master mode, no interrupt
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
            delay(100);
        }
    }
}

void stream_updateImu(void)
{
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        while (true) {
            Serial.print("ERROR: ");
            Serial.println(usfs.getErrorString());
        }
    }

    stream_imuGotGyrometer = usfs.gotGyrometer();

    if (stream_imuGotGyrometer) {
        // Returns degrees / sec
        usfs.readGyrometer(
             stream_imuGyrometerX,
             stream_imuGyrometerY,
             stream_imuGyrometerZ);
    }

    stream_imuGotQuaternion = usfs.gotQuaternion();

    if (stream_imuGotQuaternion) {
        usfs.readQuaternion(
             stream_imuQuaternionW,
             stream_imuQuaternionX,
             stream_imuQuaternionY,
             stream_imuQuaternionZ);
    }
}
