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
#include <rft_motors/realmotors/standard.hpp>
#include <rft_boards/realboards/tinypico.hpp>

#include "hackflight.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "pidcontrollers/yaw.hpp"
#include "sensors/usfsmax.hpp"

#include "receivers/dsmx/dsmx_esp32_serial1.hpp"


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

rft::TinyPico board;
static hf::UsfsQuat quat;
static hf::UsfsGyro gyro;
static hf::RatePid ratePid = hf::RatePid(0.05f, 0.00f, 0.00f); 
static hf::YawPid yawPid = hf::YawPid(0.10f, 0.01f); 
static hf::LevelPid levelPid = hf::LevelPid(0.20f);
static hf::MixerQuadXCF mixer(&motors);

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Add gyro, quaternion sensors
    h.addSensor(&quat);
    h.addSensor(&gyro);

    // Add PID controllers
    //h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&yawPid);
    h.addClosedLoopController(&ratePid);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
