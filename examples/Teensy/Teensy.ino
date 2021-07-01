/*
   Hackflight sketch for Teensy4.0 with USFS IMU, DSMX receiver, and
   standard motors

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

    Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "hackflight.hpp"
#include "mixers/quad/xmw.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"
#include "sensors/vl53l1x.hpp"
#include "sensors/pmw3901.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"

#include <rft_boards/realboards/arduino_serial/arduino/teensy.hpp>
#include <rft_motors/rotary/brushless.hpp>


// Receiver ===========================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);

// Board ================================================================================

static rft::Teensy40 board = rft::Teensy40(&Serial3); // Use Serial3 for telemetry

// Motors  ==============================================================================

static rft::BrushlessMotor motor1 = rft::BrushlessMotor(5); 
static rft::BrushlessMotor motor2 = rft::BrushlessMotor(8); 
static rft::BrushlessMotor motor3 = rft::BrushlessMotor(9); 
static rft::BrushlessMotor motor4 = rft::BrushlessMotor(11);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h = hf::Hackflight(&board, &receiver, &mixer);

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.05, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.40);

// Sensors ==============================================================================

static hf::UsfsGyrometer gyrometer;
static hf::UsfsQuaternion quaternion; // not really a sensor, but we treat it like one!
// static hf::Vl53l1xRangefinder rangefinder;
// static hf::Pmw3901OpticalFlow flowSensor(38, &SPI1);

// Setup ==============================================================================

void setup(void)
{
    // USFS piggyback
    rft::ArduinoBoard::powerPins(21, 22);
    delay(100);

    // Start I^2C
    Wire.begin();

    // Add sensors
    h.addSensor(&quaternion);
    h.addSensor(&gyrometer);
    // h.addSensor(&rangefinder);
    // h.addSensor(&flowSensor);

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Start Hackflight firmware
    h.begin();
}

// Loop ===============================================================================

void loop(void)
{
    h.update();
}
