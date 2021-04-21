/*
   Hackflight sketch for Tlera Butterfly development board with DSMX receiver
   and pass-thru to SBUS

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/SpektrumDSM 
       https://github.com/simondlevy/SBUS

   Hardware support for Butterfly development board:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <RoboFirmwareToolkit.hpp>
#include <rft_boards/realboards/arduino/butterfly.hpp>
#include <rft_closedloops/passthru.hpp>

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "actuators/dsmx2sbus.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static rft::Butterfly board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::Dsmx2Sbus actuator;

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
