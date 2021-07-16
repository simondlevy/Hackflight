/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "HF_full.hpp"
#include "hf_boards/ladybugfc.hpp"
#include "hf_receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "hf_mixers/quad/xmw.hpp"
#include "hf_pidcontrollers/rate.hpp"
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"
#include "hf_sensors/usfs.hpp"

#include <rft_motors/arduino/brushed.hpp>

// Receiver ============================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;
static constexpr float SOFTWARE_TRIM[3] = {0, 0.05, 0.035};

static hf::DSMX_Receiver_Serial1 receiver = 
    hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE, SOFTWARE_TRIM);  

// Board ================================================================================

static hf::LadybugFC board = hf::LadybugFC(&Serial2);  // Bluetooth comms over Serial2

// Motors  ==============================================================================

static rft::ArduinoBrushedMotor motor1 = rft::ArduinoBrushedMotor(13);
static rft::ArduinoBrushedMotor motor2 = rft::ArduinoBrushedMotor(A2);
static rft::ArduinoBrushedMotor motor3 = rft::ArduinoBrushedMotor(3);
static rft::ArduinoBrushedMotor motor4 = rft::ArduinoBrushedMotor(11);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// Sensors ==============================================================================

static hf::USFS imu;

// Serial tasks =========================================================================

hf::SerialTask gcsTask;

hf::SerialTask telemetryTask = hf::SerialTask(true);

// Hackflight object ====================================================================

static hf::Hackflight h(&board, &receiver, &mixer);

// Setup ================================================================================

void setup(void)
{
    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Add serial tasks
    h.addSerialTask(&gcsTask);
    h.addSerialTask(&telemetryTask);

    // Start Hackflight firmware
    h.begin();
}

// Loop ===============================================================================

void loop(void)
{
    // Update Hackflight firmware
    h.update();
}
