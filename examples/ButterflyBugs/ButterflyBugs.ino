/*
   Hackflight sketch for Tlera STM32L4 with USFS IMU, DSMX receiver, and
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

#include <rft_boards/realboards/arduino_serial/arduino/stm32l4.hpp>
#include <rft_motors/arduino/brushless.hpp>


// Receiver ====================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);

// Board =======================================================================

static rft::STM32L4 board;

// Motors  =====================================================================

static rft::ArduinoBrushlessMotor motor1 =
    rft::ArduinoBrushlessMotor(5);  // XXX 3
static rft::ArduinoBrushlessMotor motor2 =
    rft::ArduinoBrushlessMotor(8);  // XXX 4
static rft::ArduinoBrushlessMotor motor3 =
    rft::ArduinoBrushlessMotor(9);  // XXX 5
static rft::ArduinoBrushlessMotor motor4 =
    rft::ArduinoBrushlessMotor(11); // XXX 8

// Mixer =======================================================================

static hf::MixerQuadXMW mixer =
    hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// PID controllers =============================================================

static hf::RatePid ratePid = hf::RatePid(0.05, 0.00, 0.00);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.40);

// Sensors =====================================================================

static hf::USFS imu;

// Serial tasks ================================================================

static hf::SerialTask gcsTask;

// Hackflight object ===========================================================

static hf::Hackflight h(&board, &receiver, &mixer);

// Setup =======================================================================

void setup(void)
{
    // Debugging
    Serial2.begin(115200);

    // Start I^2C
    Wire.begin(TWI_PINS_6_7);

    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);
    h.addClosedLoopController(&yawPid);

    // Add serial tasks
    h.addSerialTask(&gcsTask);

    // Start Hackflight firmware
    h.begin();
}

// Loop ======================================================================

void loop(void)
{
    h.update();
}
