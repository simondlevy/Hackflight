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

#include "hackflight.hpp"
#include "boards/ladybugfc.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "mixers/quad/xmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

#include <rft_motors/rotary/brushed.hpp>

// Receiver ============================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

// Board ================================================================================

static hf::LadybugFC board = hf::LadybugFC(&Serial2);  // Bluetooth comms over Serial2

// Motors  ==============================================================================

static rft::BrushedMotor motor1 = rft::BrushedMotor(13);
static rft::BrushedMotor motor2 = rft::BrushedMotor(A2);
static rft::BrushedMotor motor3 = rft::BrushedMotor(3);
static rft::BrushedMotor motor4 = rft::BrushedMotor(11);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h;

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// Sensors ==============================================================================

static hf::USFS imu;

// Vehicle state ========================================================================

static hf::State state;

// Setup ==============================================================================

void setup(void)
{
    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Adjust trim
    receiver.setTrim(0, 0.05, 0.05);

    // Start Hackflight firmware
    h.begin(&board, &receiver, &mixer, &state);
}

// Loop ===============================================================================

void loop(void)
{
    h.update(&board, &receiver, &mixer, &state);
}
