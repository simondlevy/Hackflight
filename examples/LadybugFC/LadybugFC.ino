/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

#include "hackflight.hpp"
#include "boards/ladybugfc.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"

static hf::LadybugFC board;

void setup(void)
{
    // Add gyro, quaternion sensors
    // h.addSensor(&gyro);
    // h.addSensor(&quat);

    // Add PID controllers
    // h.addPidController(&levelPid);
    // h.addPidController(&ratePid);

    // Initialize Hackflight firmware
    // h.begin();
}

void loop(void)
{
    // h.update();
}
