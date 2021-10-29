/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "serial.hpp"
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
          uint8_t pbyte
        , uint8_t msgtype
        , bool sending
        , uint8_t payindex
        , uint8_t pstate
        , uint8_t psize
        , uint8_t crc
        , bool checked
        )
{
    static float state_phi = -0.1;
    float state_theta = 0.2;
    static float state_psi;

    static uint8_t motor_index;
    static uint8_t motor_percent;

    if (pbyte != 0xff) {
        if (pbyte == 0x24) {
            Debugger::printf(Serial1, "\n");
        }
        Debugger::printf(Serial1,
                "byte=%03d state=%d size=%d crc=%03d checked=%d\n",
                pbyte, pstate, psize, crc, checked);
    }

    if (sending) {
        handleSerialInput(msgtype, state_phi, state_theta, state_psi);
    }

    else if (msgtype == 215) {

        if (payindex == 1) {
            motor_index = pbyte;
        }

        if (payindex == 2) {
            motor_percent = pbyte;
        }
    }

    updateSerialOutput();

    /*
    float m1_val = motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = motor_index == 4 ? motor_percent/100. : 0;

    Debugger::printf(Serial1, "m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n",
              m1_val, m2_val, m3_val, m4_val);
    */

    delay(10);
}
