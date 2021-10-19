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
#include "stream_motors.h"

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
        bool mrunning,
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
    motors.running = mrunning;
    motors.values[0] = m1;
    motors.values[1] = m2;
    motors.values[2] = m3;
    motors.values[3] = m4;

    //Debugger::printf("%+3.3f %+3.3f %+3.3f %+3.3f | %+3.3f\n", 
    //        m1, m2, m3, m4, mmax);

    bool gotSerialMotors = false;

    parser.parse(state_phi, state_theta, state_psi, motors);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (parser.available()) {
        stream_serialWrite(parser.read());
    }

    if (motors.running) {
        static const uint8_t motor_pins[4] = {m1_pin, m2_pin, m3_pin, m4_pin};
        stream_writeBrushedMotors(motor_pins, motors.values);
    }
}
