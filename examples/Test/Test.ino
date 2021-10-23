/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "newparser.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "copilot.h"

void setup(void)
{
    Serial1.begin(115200);
}

void loop(void)
{
    step();
}

void stream_run(
        float state_phi,
        float state_theta,
        float state_psi,
        bool armed)
{
    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    bool avail = false;
    uint8_t byte = 0;

    parse(stream_serialByte, state_phi, state_theta, state_psi, avail, byte);

    if (avail) {
        //Serial1.println(byte);
        stream_serialWrite(byte);
    }
}
