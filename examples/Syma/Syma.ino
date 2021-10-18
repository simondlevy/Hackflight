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

#include "serial.hpp"

#include "stream_serial.h"
#include "stream_motors.h"

#include "copilot.h"

static SerialComms serial;

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
        bool mready,
        bool mcut,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1,
        float m2,
        float m3,
        float m4)
{
    motors_t motors = {};
    motors.ready = mready;
    motors.values[0] = mcut ? 0 : m1;
    motors.values[1] = mcut ? 0 : m2;
    motors.values[2] = mcut ? 0 : m3;
    motors.values[3] = mcut ? 0 : m4;

    serial.parse(state_phi, state_theta, state_psi, motors);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (serial.available()) {
        stream_serialWrite(serial.read());
    }

    if (motors.ready) {
        static const uint8_t motor_pins[4] = {m1_pin, m2_pin, m3_pin, m4_pin};
        stream_writeBrushedMotors(motor_pins, motors.values);
    }
}
