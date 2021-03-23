/*
   Hackflight sketch for TinyPICO with USFSMAX IMU, DSMX receiver, and standard motors

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/DSMRX

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <RoboFirmwareToolkit.hpp>
#include <rft_motors/mock.hpp>

#include "hackflight.hpp"
#include "boards/tinypico.hpp"
#include "mixers/quadxcf.hpp"
#include "sensors/usfs_rotated.hpp"

#include "receivers/mock.hpp"

// Receiver --------------------------------------------------------------

hf::MockReceiver receiver;


// Motors ----------------------------------------------------------------

rft::MockMotor motors;

// -----------------------------------------------------------------------

hf::TinyPico board;

static hf::UsfsGyro gyro;
static hf::UsfsQuat quat;
static hf::MixerQuadXCF mixer(&motors);

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Add gyro, quaternion sensors
    h.addSensor(&gyro);
    h.addSensor(&quat);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
