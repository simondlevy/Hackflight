/*
   Hackflight sketch for M5Stack AtomFly

   Currently uses MPU6050 instead of actual MPU6886, and a mock receiver

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/MPU
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/ERROPiX/ESP32_AnalogWrite

    Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "HF_full.hpp"
#include "hf_mixers/quad/xmw.hpp"
#include "hf_pidcontrollers/rate.hpp"
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"

#include "hf_receivers/mock.hpp"
#include "hf_sensors/usfs.hpp"

#include <RFT_full.hpp>
#include <rft_boards/realboards/arduino_serial/tinypico.hpp>
#include <rft_motors/arduino/brushed.hpp>

// Receiver ============================================================================

static hf::MockReceiver receiver;

// Board ================================================================================

static rft::TinyPico board;

// Motors  ==============================================================================

// These are reasonable pins for the ESP32
static rft::ArduinoBrushedMotor motor1 = rft::ArduinoBrushedMotor(25);
static rft::ArduinoBrushedMotor motor2 = rft::ArduinoBrushedMotor(26);
static rft::ArduinoBrushedMotor motor3 = rft::ArduinoBrushedMotor(27);
static rft::ArduinoBrushedMotor motor4 = rft::ArduinoBrushedMotor(15);

// Mixer ================================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// Hackflight object ====================================================================

static hf::Hackflight h = hf::Hackflight(&board, &receiver, &mixer);

// PID controllers ======================================================================

static hf::RatePid ratePid = hf::RatePid(0.04, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.20, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.80);

// Sensors =====================================================================

static hf::USFS imu;

// Serial tasks ================================================================

hf::SerialTask gcsTask;
hf::SerialTask telemetryTask = hf::SerialTask(true);


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

    // Start Hackflight firmware
    h.begin();
}

// Loop ===============================================================================

void loop(void)
{
    h.update();
}
