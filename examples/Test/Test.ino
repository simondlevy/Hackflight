/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "debugger.hpp"
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
          uint8_t msgtype
        , bool sending
        , uint8_t paybyte
        , uint8_t payindex
        , bool checked
        )
{
    static uint8_t _motor_index;
    static uint8_t _motor_percent;

    _motor_index   = msgtype == 215 && payindex == 1 ? paybyte : _motor_index;
    _motor_percent = msgtype == 215 && payindex == 2 ? paybyte : _motor_percent;

    float m1_val = _motor_index == 1 ? _motor_percent/100. : 0;
    float m2_val = _motor_index == 2 ? _motor_percent/100. : 0;
    float m3_val = _motor_index == 3 ? _motor_percent/100. : 0;
    float m4_val = _motor_index == 4 ? _motor_percent/100. : 0;

    Debugger::printf(Serial1, "m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n", m1_val, m2_val, m3_val, m4_val);

    delay(10);
}
