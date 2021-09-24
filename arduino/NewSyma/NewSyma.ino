/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Wire.h>

#include "copilot.h"

#include "copilot_arduino.h"

static uint8_t LED_PIN = 18;

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    copilot_startLed(LED_PIN);
    copilot_startReceiver();
    copilot_startImu();
}

void loop(void)
{
    copilot_micros = micros();

    copilot_updateReceiver();
    copilot_updateImu();

    step();
}
