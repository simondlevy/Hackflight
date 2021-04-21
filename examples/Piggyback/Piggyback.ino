/*
   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <RoboFirmwareToolkit.hpp>
#include <rft_boards/realboards/arduino/butterfly.hpp>
#include <rft_motors/mock.hpp>
#include <rft_actuators/mock.hpp>

#include "hackflight.hpp"
#include "receivers/mock.hpp"
#include "sensors/usfs.hpp"

static rft::Butterfly board;

static hf::MockReceiver receiver;

static rft::MockActuator actuator;

static hf::Hackflight h(&board, &receiver, &actuator);

void setup(void)
{
    h.begin();
}

void loop(void)
{
    h.update();
}
