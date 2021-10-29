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

void stream_run(uint8_t motor_index, uint8_t motor_percent)
{
    float m1_val = motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = motor_index == 4 ? motor_percent/100. : 0;

    Debugger::printf(Serial1, "m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n", m1_val, m2_val, m3_val, m4_val);

    delay(10);
}
