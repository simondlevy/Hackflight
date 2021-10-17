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

#include "HF_full.hpp"
#include "HF_debugger.hpp"
#include "hf_mixers/quad/xmw.hpp"

#include "stream_serial.h"
#include "stream_motors.h"

#include "copilot.h"

static const uint8_t MOTOR_PINS[4] = {13, 16, 3, 11};

static hf::MixerQuadXMW mixer;

static hf::HackflightFull h(&mixer);

static bool running;

void setup(void)
{
}

void loop(void)
{
    step();
}

void stream_runHackflight(
        bool mready,
        bool mcut,
        float state_phi,
        float state_theta,
        float state_psi,
        float tdmd,
        float rdmd,
        float pdmd,
        float ydmd)

{
    hf::motors_t motors = {};

    h.update(
            mready,
            mcut,
            state_phi,
            state_theta,
            state_psi,
            tdmd,
            rdmd,
            pdmd,
            ydmd,
            motors);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (h.serialAvailable() > 0) {
        stream_serialWrite(h.serialRead());
    }

    /*
    hf::Debugger::printf("%d | %3.3f %3.3f %3.3f %3.3f\n",
            motors.ready,
            motors.values[0],
            motors.values[1],
            motors.values[2],
            motors.values[3]);*/

    if (motors.ready) {
        stream_writeBrushedMotors(MOTOR_PINS, motors.values);
    }
}
