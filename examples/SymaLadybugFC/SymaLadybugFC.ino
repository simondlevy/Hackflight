/*
   Hackflight sketch for Syma X5C copter with Ladybug Flight Controller with
   Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"
#include "boards/ladybugfc.hpp"
#include "actuators/mixers/quads/quadxmw.hpp"

#include <rft_motors/realmotors/brushed.hpp>

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::LadybugFC board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static rft::BrushedMotor motor1 = rft::BrushedMotor(hf::LadybugFC::MOTOR1_PIN);
static rft::BrushedMotor motor2 = rft::BrushedMotor(hf::LadybugFC::MOTOR2_PIN);
static rft::BrushedMotor motor3 = rft::BrushedMotor(hf::LadybugFC::MOTOR3_PIN);
static rft::BrushedMotor motor4 = rft::BrushedMotor(hf::LadybugFC::MOTOR4_PIN);

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(2, 0.1);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

static hf::Hackflight h(&board, &receiver, &mixer);

static hf::UsfsGyrometer gyrometer;
static hf::UsfsQuaternion quaternion; // not really a sensor, but we treat it like one!

void setup(void)
{

    // Add sensors
    h.addSensor(&quaternion);
    h.addSensor(&gyrometer);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Adjust trim
    receiver.setTrim(0.03, 0.12, 0.02);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
