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

static Parser parser;

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
    parser.parse(state_phi, state_theta, state_psi, armed, m1_val, m2_val, m3_val, m4_val);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (parser.available()) {
        stream_serialWrite(parser.read());
    }

    void stream_writeBrushedMotors(const uint8_t * pins, float * values, const uint8_t count=4);

    static const uint8_t motor_pins[4] = {m1_pin, m2_pin, m3_pin, m4_pin};

    float motor_values[4] = {m1_val, m2_val, m3_val, m4_val};

    stream_writeBrushedMotors(motor_pins, motor_values);
}
