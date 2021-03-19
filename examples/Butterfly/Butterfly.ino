/*
   Hackflight sketch for Butterfly board with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Butterfly flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <Arduino.h>

#include <RoboFirmwareToolkit.hpp>
#include <rft_motors/standard.hpp>

#include "hackflight.hpp"
#include "boards/butterfly_piggyback.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

static const uint8_t MOTOR_PINS[4] = {5, 8 , 9, 11};
static rft::StandardMotor motors = rft::StandardMotor(MOTOR_PINS, 4);

static rft::Butterfly board;

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.58f;

static hf::UsfsGyro gyro;
static hf::UsfsQuat quat;
static hf::RatePid ratePid = hf::RatePid( 0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 
static hf::LevelPid levelPid = hf::LevelPid(0.20f);
static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  
static hf::MixerQuadXCF mixer(&motors);

// hf::MockReceiver receiver; 
// hf::MockMotor motors;

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Add gyro, quaternion sensors
    h.addSensor(&gyro);
    h.addSensor(&quat);

    // Add rate and level PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
