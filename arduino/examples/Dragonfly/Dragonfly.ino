#include <Arduino.h>

#include "hackflight.h"

void setup(void)
{
    hackflightInit();
}

void loop(void)
{
    hackflightStep();

    /*
    static uint16_t _channelData[18];
    static timeUs_t _lastFrameTimeUs;

    if (sbusUpdate(_channelData, &_lastFrameTimeUs)) {
        Serial.println(_channelData[0]);
    }

    delay(5);
    */
}
