/*
   Hackflight sketch for Tlera Butterfly with USFS IMU, DSMX receiver, and
   standard motors

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

    Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "hf_full.hpp"
#include "mixers/quad/xmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"

#include <rft_boards/realboards/arduino_serial/arduino/butterfly.hpp>
#include <rft_motors/rotary/brushless.hpp>


// Receiver ===========================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);

// Board ================================================================================

static rft::Butterfly board;

// Motors  ==============================================================================

static rft::BrushlessMotor motor1 = rft::BrushlessMotor(5);  // XXX 3
static rft::BrushlessMotor motor2 = rft::BrushlessMotor(8);  // XXX 4
static rft::BrushlessMotor motor3 = rft::BrushlessMotor(9);  // XXX 5
static rft::BrushlessMotor motor4 = rft::BrushlessMotor(11); // XXX 8

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h;

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.05, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.40);

// Sensors ==============================================================================

static hf::USFS imu;

// Vehicle state ========================================================================

static hf::State state;

// Serial tasks =========================================================================

hf::SerialTask gcsTask = hf::SerialTask(&receiver, &mixer, &state); 

// Setup ==============================================================================

void setup(void)
{
    // Start I^2C
    Wire.begin();

    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Add serial tasks
    h.addSerialTask(&gcsTask);

    // Start Hackflight firmware
    h.begin(&board, &receiver, &mixer);
}

// Loop ===============================================================================

void loop(void)
{
    h.update(&board, &receiver, &mixer, &state);
}
