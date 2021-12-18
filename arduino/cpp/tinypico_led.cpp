/*
   TinyPICO LED support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <TinyPICO.h>

void stream_ledWrite(bool value)
{
    static TinyPICO tp;

    // NB: This takes a non-trivial amount of time, so don't do it in a loop!
    tp.DotStar_SetPixelColor(0, value?255:0, 0);
}
