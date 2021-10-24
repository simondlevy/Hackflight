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
    uint8_t buffer_size = 0;

    uint8_t motor_index = 0;
    uint8_t motor_percent = 0;

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    parser_parse(
            buffer_size,
            state_phi,
            state_theta,
            state_psi,
            motor_index,
            motor_percent);

    if (buffer_size > 0) {
        stream_serialWrite(parser_read(buffer_size));
    }

    delay(10);
}
