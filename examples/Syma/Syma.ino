/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

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
        float m1_val,
        float m2_val,
        float m3_val,
        float m4_val)
{
    static uint8_t serial_buffer[128];
    static uint8_t buffer_index;
    uint8_t buffer_size = 0;

    parser_parse(
            serial_buffer,
            buffer_size,
            buffer_index,
            state_phi,
            state_theta,
            state_psi,
            armed,
            m1_val,
            m2_val,
            m3_val,
            m4_val);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (buffer_size > 0) {
        stream_serialWrite(parser_read(serial_buffer, buffer_size, buffer_index));
    }

    void stream_writeBrushedMotors(
            uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin,
            float m1_val, float m2_val, float m3_val, float m4_val);

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
}
