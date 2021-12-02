/*
   TinyPICO LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <TinyPICO.h>

void stream_writeLed(const uint8_t pin, bool value)
{
    static TinyPICO tp;

    tp.DotStar_SetPixelColor(0, isOn?255:0, 0);
}
