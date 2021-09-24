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

#include "copilot.h"
#include <Wire.h>
// #include "copilot_usfs.h"

static uint8_t LED_PIN = 18;

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    void copilot_startUsfs(void);
    void copilot_startLed(uint8_t pin);
    void copilot_startDsmrx(void);

    copilot_startLed(LED_PIN);
    copilot_startUsfs();
    copilot_startDsmrx();
}

void loop(void)
{
    copilot_micros = micros();

    step();
}
