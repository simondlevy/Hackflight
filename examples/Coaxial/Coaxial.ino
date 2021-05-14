/*
   Hackflight sketch for experimental coaxial copter with Teensy 4.0 and
   Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/SpektrumDSM 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "actuators/coaxial.hpp"
#include "sensors/usfs.hpp"

#include <rft_boards/realboards/arduino/teensy.hpp>
#include <rft_closedloops/passthru.hpp>

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static rft::Teensy40 board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::CoaxialActuator actuator;

static hf::Hackflight h(&board, &receiver, &actuator);

static rft::PassthruController passthru;

void setup(void)
{
    h.addClosedLoopController(&passthru);

    h.begin();
}

void loop(void)
{
    h.update();
}
