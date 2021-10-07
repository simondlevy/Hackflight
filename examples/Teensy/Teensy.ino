/*
   Hackflight sketch for Teensy4.0 with USFS IMU, DSMX receiver, and
   standard motors

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

    Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "HF_full.hpp"
#include "hf_mixers/quad/xmw.hpp"
#include "hf_pidcontrollers/rate.hpp"
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"
#include "hf_sensors/usfs.hpp"
#include "hf_receivers/arduino/dsmx/dsmx_serial1.hpp"

#include <rft_boards/realboards/arduino_serial/arduino/teensy40.hpp>
#include <rft_motors/arduino/brushless.hpp>


// Receiver ===========================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);

// Board ================================================================================

static rft::Teensy40 board = rft::Teensy40(&Serial3); // Use Serial3 for telemetry

// Motors  ==============================================================================

static rft::ArduinoBrushlessMotor motor1 = rft::ArduinoBrushlessMotor(5); 
static rft::ArduinoBrushlessMotor motor2 = rft::ArduinoBrushlessMotor(8); 
static rft::ArduinoBrushlessMotor motor3 = rft::ArduinoBrushlessMotor(9); 
static rft::ArduinoBrushlessMotor motor4 = rft::ArduinoBrushlessMotor(11);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h = hf::Hackflight(&board, &receiver, &mixer);

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.05, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.40);

// Sensors ==============================================================================

static hf::USFS imu;

// Setup ==============================================================================

void setup(void)
{
    // USFS piggyback
    rft::ArduinoBoard::powerPins(21, 22);
    delay(100);

    // Start I^2C
    Wire.begin();

    delay(100);

    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Start Hackflight firmware
    h.begin();
}

// Loop ===============================================================================

void loop(void)
{
    h.update();
}
