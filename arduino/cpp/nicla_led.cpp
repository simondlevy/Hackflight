/*
   Nicla Sense ME LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Nicla_System.h>

void stream_startLed(void)
{
    nicla::begin();
    nicle::leds.begin();
}

void stream_writeLed(bool value)
{
    nicla::leds.setColor(value ? green : off);
}
