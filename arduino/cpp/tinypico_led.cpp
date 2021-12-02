/*
   TinyPICO LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <TinyPICO.h>

void stream_writeLed(bool value)
{
    static TinyPICO tp;

    tp.DotStar_SetPixelColor(0, value?255:0, 0);
}
