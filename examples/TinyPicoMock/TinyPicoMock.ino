/*
   Hackflight sketch with TinyPICO board and mock receiver, motors

   Additional libraries needed:

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <RoboFirmwareToolkit.hpp>
#include <rft_boards/realboards/arduino/teensy.hpp>
#include <rft_motors/mock.hpp>

#include "hackflight.hpp"
#include "actuators/mixers/quadxmw.hpp"
#include "receivers/mock.hpp"
#include "sensors/usfsmax.hpp"


static rft::Teensy40 board;

static hf::MockReceiver receiver;

static rft::MockMotor motors;

static hf::MixerQuadXMW mixer(&motors);

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    h.begin();
}

void loop(void)
{
    h.update();
}
