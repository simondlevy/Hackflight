/*
   Nicla Sense ME LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Nicla_System.h>

void stream_startNiclaLed(void)
{
    nicla::begin();
    nicla::leds.begin();
}

void stream_writeNiclaLed(bool value)
{
    nicla::leds.setColor(value ? green : off);
}
