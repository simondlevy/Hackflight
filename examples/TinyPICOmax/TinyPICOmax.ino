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

#include "hackflight.hpp"
#include "actuators/mixers/quadxmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "receivers/arduino/dsmx/dsmx_esp32_serial1.hpp"
#include "sensors/usfsmax.hpp"

#include <rft_motors/realmotors/standard.hpp>
#include <rft_boards/realboards/tinypico.hpp>
#include <rft_closedloops/passthru.hpp>

static rft::TinyPico board;

// Motors ----------------------------------------------------------------
static const uint8_t MOTOR_PINS[4] = {15, 27, 26 ,25};
static rft::StandardMotor motors = rft::StandardMotor(MOTOR_PINS, 4);

// Receiver --------------------------------------------------------------

static const uint8_t SERIAL1_RX = 4;
static const uint8_t SERIAL1_TX = 14;  // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

hf::DSMX_ESP32_Serial1 receiver = hf::DSMX_ESP32_Serial1(CHANNEL_MAP, DEMAND_SCALE, SERIAL1_RX, SERIAL1_TX);  

// -----------------------------------------------------------------------


static hf::MixerQuadXMW mixer(&motors);

static hf::UsfsMaxGyrometer gyrometer;
static hf::UsfsMaxQuaternion quaternion; // not really a sensor, but we treat it like one!

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(2, 0.1);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

static rft::PassthruController passthru;

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Add sensors
    h.addSensor(&quaternion);
    h.addSensor(&gyrometer);

    // Add PID controllers
    // h.addClosedLoopController(&levelPid);
    // h.addClosedLoopController(&ratePid);
    // h.addClosedLoopController(&yawPid);


    h.addClosedLoopController(&passthru);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
