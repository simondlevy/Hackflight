/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "debugger.hpp"
#include "stream_serial.h"
#include "stream_receiver.h"
#include "copilot.h"

void setup(void)
{
    Serial1.begin(115200);

    for (uint8_t k=0; k<10; ++k) {
        Serial1.println();
    }
}

void loop(void)
{
    step();
}

void stream_run(
        bool avail,
        uint8_t byteval,
        uint8_t pstate,
        uint8_t psize,
        uint8_t msgtype,
        uint8_t crc,
        bool sending)
{
    if (avail) {
        Debugger::printf(Serial1,
                "byte=x%02x  state=%d size=%d msgtype=%d crc=x%02x sending=%d\n",
                byteval, pstate, psize, msgtype, crc, sending);
    }

    delay(10);
}
