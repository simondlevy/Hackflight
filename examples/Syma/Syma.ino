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
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"

#include "stream_serial.h"
#include "stream_motors.h"
#include "stream_led.h"

#include "copilot.h"

static const uint8_t MOTOR_PINS[4] = {13, 16, 3, 11};

static uint32_t LED_PIN = 18;

static hf::MixerQuadXMW mixer;

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

static hf::HackflightFull h(&mixer);

static bool running;

void setup(void)
{
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);
}

void loop(void)
{
    step();
}

void stream_runHackflight(
        float tdmd,
        float rdmd,
        float pdmd,
        float ydmd,
        bool rxarmed,
        bool rxtdown,
        float state_phi,
        float state_theta,
        float state_psi,
        float state_dphi,
        float state_dtheta,
        float state_dpsi)
{
    bool ledval = false;
    hf::motors_t motors = {};

    h.update(micros(),
            tdmd,
            rdmd,
            pdmd,
            ydmd,
            rxarmed,
            rxtdown,
            state_phi,
            state_theta,
            state_psi,
            state_dphi,
            state_dtheta,
            state_dpsi,
            motors,
            ledval);

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

    stream_writeLed(LED_PIN, ledval);
}
