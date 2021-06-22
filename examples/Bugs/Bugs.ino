/*
   Hackflight sketch for TinyPICO with USFS IMU, DSMX receiver, and standard motors

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

    Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_esp32_serial1.hpp"
#include "mixers/quadxmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

#include <rft_boards/realboards/tinypico.hpp>
#include <rft_motors/rotary.hpp>

static const uint8_t SERIAL1_RX = 32;
static const uint8_t SERIAL1_TX = 33; // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 8.0;

static rft::TinyPico board;

static hf::DSMX_ESP32_Serial1 receiver = hf::DSMX_ESP32_Serial1(CHANNEL_MAP, DEMAND_SCALE, SERIAL1_RX, SERIAL1_TX);  

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(hf::Mixer::BRUSHLESS, 25, 26, 27, 15);

static hf::Hackflight h = hf::Hackflight(&board, &receiver, &mixer);

static hf::RatePid ratePid = hf::RatePid(0.04, 0.002, 0.01);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.20);

static hf::UsfsGyrometer gyrometer;
static hf::UsfsQuaternion quaternion; // not really a sensor, but we treat it like one!

void setup(void)
{
    // Start I^2C
    Wire.begin();

    // Add sensors
    h.addSensor(&quaternion);
    h.addSensor(&gyrometer);

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Adjust trim
    receiver.setTrim(0, 0.05, 0);

    // Start Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
