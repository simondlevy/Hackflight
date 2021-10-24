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

void stream_run( float state_phi, float state_theta, float state_psi, bool armed)
{
    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    bool data_available = false;
    uint8_t motor_index = 0;
    uint8_t motor_percent = 0;

    parser_parse(
            state_phi,
            state_theta,
            state_psi,
            data_available,
            motor_index,
            motor_percent);

    if (data_available) {
        stream_serialWrite(parser_read());
    }

    delay(10);
}
