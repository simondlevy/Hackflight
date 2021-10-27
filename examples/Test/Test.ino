/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "serial.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "stream_receiver.h"
#include "copilot.h"

//float stream_receiverThrottle;
//float stream_receiverRoll;
//float stream_receiverPitch;
//float stream_receiverYaw;
//float stream_receiverAux1;
//float stream_receiverAux2;

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
        bool sending,
        bool receiving)
{
    /*
    if (avail) {
        Debugger::printf(Serial1,
                "byte=x%02x state=%d size=%d msgtype=%d crc=x%02x sending=%d receiving=%d\n",
                byteval, pstate, psize, msgtype, crc, sending, receiving);
    }*/

    if (receiving) {
        Debugger::printf(Serial1, "byte=x%02x msgtype=%d\n", byteval);
    }

    static float state_phi;
    static float state_theta;
    static float state_psi;

    if (sending) {
        handleSerialInput(msgtype, state_phi, state_theta, state_psi);
    }

    updateSerialOutput();

    delay(10);
}
