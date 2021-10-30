/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "debugger.hpp"
#include "copilot.h"

void setup(void)
{
    Serial1.begin(115200);
}

void loop(void)
{
    step();
}

void stream_debug(float m1, float m2, float m3, float m4)
{
    Debugger::printf(Serial1, "m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n", m1, m2, m3, m4);
    delay(5);
}
