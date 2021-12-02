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

void stream_startUsfs(void)
{
    // Start the USFS in master mode, no interrupt
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
            delay(100);
        }
    }
}

void stream_updateUsfs(void)
{
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        while (true) {
            Serial.print("ERROR: ");
            Serial.println(usfs.getErrorString());
        }
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
