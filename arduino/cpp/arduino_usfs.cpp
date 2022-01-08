/*
   Streaming support for USFS IMU

   Copyright (C) 2021 Simon D. Levy

   MIT License
 */


#define _EXTERN
#include "hackflight.h"

#include <Arduino.h>
#include <USFS_Master.h>

static USFS_Master usfs;

void usfsStart(void)
{
    // Start the USFS in master mode, no interrupt
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
            delay(100);
        }
    }
}

void usfsUpdate(void)
{
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        while (true) {
            Serial.print("ERROR: ");
            Serial.println(usfs.getErrorString());
        }
    }

    imuGotGyrometer = usfs.gotGyrometer();

    if (imuGotGyrometer) {
        // Returns degrees / sec
        usfs.readGyrometer(
             imuGyrometerX,
             imuGyrometerY,
             imuGyrometerZ);
    }

    imuGotQuaternion = usfs.gotQuaternion();

    if (imuGotQuaternion) {
        usfs.readQuaternion(
             imuQuaternionW,
             imuQuaternionX,
             imuQuaternionY,
             imuQuaternionZ);
    }
}
