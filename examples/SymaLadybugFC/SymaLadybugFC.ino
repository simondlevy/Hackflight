/*
   Hackflight sketch for Syma X5C copter with Ladybug Flight Controller with
   Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Simon D. Levy
 */

#include "hackflight.hpp"
#include "boards/realboards/arduino/ladybugfc.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"


static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::LadybugFC board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::MixerQuadXCF mixer(&hf::ladybugFcNewMotors);

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(2, 0.1);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

static hf::Hackflight h(&board, &receiver, &mixer);

static hf::UsfsGyrometer gyrometer;
static hf::UsfsQuaternion quaternion; // not really a sensor, but we treat it like one!

static hf::USFS imu;
 

void setup(void)
{

    // XXX simplify
    h.addSensor(&quaternion);
    quaternion.imu = &imu;
    h.addSensor(&gyrometer);
    gyrometer.imu = &imu;

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Adjust trim
    receiver.setTrim(0.03, 0.12, 0.02);

    // Initialize Hackflight firmware
    h.begin();

    imu.begin();
}

void loop(void)
{
    h.update();
}
