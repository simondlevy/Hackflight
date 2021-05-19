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

#include "boards/ladybugfc_new.hpp"
#include "actuators/mixers_new/quads/quadxmw_new.hpp"
#include "motors_new/brushed.hpp"

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::NewLadybugFC board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::NewBrushedMotor motor1 = hf::NewBrushedMotor(hf::NewLadybugFC::MOTOR1_PIN);
static hf::NewBrushedMotor motor2 = hf::NewBrushedMotor(hf::NewLadybugFC::MOTOR2_PIN);
static hf::NewBrushedMotor motor3 = hf::NewBrushedMotor(hf::NewLadybugFC::MOTOR3_PIN);
static hf::NewBrushedMotor motor4 = hf::NewBrushedMotor(hf::NewLadybugFC::MOTOR4_PIN);

static hf::NewMixerQuadXMW mixer = hf::NewMixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

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
