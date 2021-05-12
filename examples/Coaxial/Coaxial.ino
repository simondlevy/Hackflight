/*
   Hackflight sketch for experimental coaxial copter with Teensy 4.0 and
   Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "actuators/mixers/coaxial.hpp"
#include "sensors/usfs.hpp"

#include <rft_boards/realboards/arduino/teensy.hpp>

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::LadybugFC board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::MixerCoaxial mixer(&hf::ladybugFcNewMotors);

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
