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
#include "mixers/quad/xmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

#include <rft_boards/realboards/tinypico.hpp>
#include <rft_motors/rotary/brushless.hpp>

// Receiver ============================================================================

static const uint8_t SERIAL1_RX = 32;
static const uint8_t SERIAL1_TX = 33; // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 4.0;

static hf::DSMX_ESP32_Serial1 receiver = hf::DSMX_ESP32_Serial1(CHANNEL_MAP, DEMAND_SCALE, SERIAL1_RX, SERIAL1_TX);  

// Board ================================================================================

static rft::TinyPico board;

// Motors  ==============================================================================

static rft::BrushlessMotor motor1 = rft::BrushlessMotor(25);
static rft::BrushlessMotor motor2 = rft::BrushlessMotor(26);
static rft::BrushlessMotor motor3 = rft::BrushlessMotor(27);
static rft::BrushlessMotor motor4 = rft::BrushlessMotor(15);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h = hf::Hackflight(&board, &receiver, &mixer);

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.04, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.20, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.80);

// Sensors ==============================================================================

static hf::UsfsGyrometer gyrometer;
static hf::UsfsQuaternion quaternion; // not really a sensor, but we treat it like one!

// Setup ==============================================================================

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
    receiver.setTrim(0, 0, -.01);

    // Start Hackflight firmware
    h.begin();
}

// Loop ===============================================================================

void loop(void)
{
    h.update();
}
