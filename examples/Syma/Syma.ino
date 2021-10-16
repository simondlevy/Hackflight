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
#include "hf_pidcontrollers/rate.hpp"

#include "stream_serial.h"
#include "stream_motors.h"

#include "copilot.h"

static const uint8_t MOTOR_PINS[4] = {13, 16, 3, 11};

static hf::MixerQuadXMW mixer;

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);

static hf::HackflightFull h(&mixer);

static bool running;

void setup(void)
{
    h.addPidController(&ratePid);
}

void loop(void)
{
    step();
}

void stream_runHackflight(
        float rxtdmd,
        float rxrdmd,
        float rxrxpdmd,
        float rxydmd,
        bool rxarmed,
        bool rxtdown,
        float state_phi,
        float state_theta,
        float state_psi,
        float state_dphi,
        float state_dtheta,
        float state_dpsi, 
        bool pready,
        float pidtdmd,
        float pidrdmd,
        float pidpdmd,
        float pidydmd)

{
    hf::motors_t motors = {};

    h.update(micros(),
            rxtdmd,
            rxrdmd,
            rxrxpdmd,
            rxydmd,
            rxarmed,
            rxtdown,
            state_phi,
            state_theta,
            state_psi,
            state_dphi,
            state_dtheta,
            state_dpsi,
            pready,
            pidtdmd,
            pidrdmd,
            pidpdmd,
            pidydmd,
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
