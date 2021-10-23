/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "parser.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "copilot.h"

void setup(void)
{
}

void loop(void)
{
    step();
}

void stream_run(
        float state_phi,
        float state_theta,
        float state_psi,
        bool armed,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1_flying,
        float m2_flying,
        float m3_flying,
        float m4_flying)
{
    static uint8_t serial_buffer[128];
    static uint8_t buffer_index;
    uint8_t buffer_size = 0;
    uint8_t gcs_motor_index = 0;
    uint8_t gcs_motor_percent = 0;

    parser_parse(
            serial_buffer,
            buffer_size,
            buffer_index,
            state_phi,
            state_theta,
            state_psi,
            gcs_motor_index,
            gcs_motor_percent);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (buffer_size > 0) {
        stream_serialWrite(parser_read(serial_buffer, buffer_size, buffer_index));
    }
}
