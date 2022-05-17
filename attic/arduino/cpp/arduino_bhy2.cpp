/*
   Streaming support for BHY2 sensor

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */


#define _EXTERN
#include "hackflight.h"

#include <Arduino.h>
#include <Arduino_BHY2.h>

static  SensorQuaternion quaternion(SENSOR_ID_RV);

void stream_startBhy2(void)
{
    BHY2.begin();
    quaternion.begin();
}

void stream_updateBhy2(void)
{
    BHY2.update();

    static uint32_t _usec;

    uint32_t usec = micros();

    stream_imuQuaternionReady = false;

    if (usec - _usec > 10000) {

        stream_imuGotQuaternion = true;

        stream_imuQuaternionW = quaternion.w() / 10000.;
        stream_imuQuaternionX = quaternion.x() / 10000.;
        stream_imuQuaternionY = quaternion.y() / 10000.;
        stream_imuQuaternionZ = quaternion.z() / 10000.;

        _usec = usec;
    }

}
