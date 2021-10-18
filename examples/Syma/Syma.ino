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
        float state_phi,
        float state_theta,
        float state_psi,
        bool mready,
        bool mcut,
        float m1,
        float m2,
        float m3,
        float m4)
{
    hf::motors_t motors = {};

    h.update(
            state_phi,
            state_theta,
            state_psi,
            mready,
            mcut,
            m1,
            m2,
            m3,
            m4,
            motors);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (h.serialAvailable() > 0) {
        stream_serialWrite(h.serialRead());
    }


    if (motors.ready) {
        stream_writeBrushedMotors(MOTOR_PINS, motors.values);
    }
}
