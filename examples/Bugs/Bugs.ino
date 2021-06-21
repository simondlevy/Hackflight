/*
   Hackflight sketch for TinyPICO with USFS IMU, DSMX receiver, and standard motors

   Copyright (C) 2021 Simon D. Levy
 
   MIT License
 */

#include "hackflight.hpp"
#include "boards/realboards/tinypico.hpp"
#include "receivers/arduino/dsmx/dsmx_esp32_serial1.hpp"
#include "mixers/quadxmw.hpp"
#include "motors/standard.hpp"
#include "imus/usfs.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/yaw.hpp"
#include "pidcontrollers/level.hpp"

static const uint8_t SERIAL1_RX = 32;
static const uint8_t SERIAL1_TX = 33; // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 8.0;

static const uint8_t MOTOR_PINS[4] = {25, 26 ,27, 15};

hf::StandardMotor motors = hf::StandardMotor(MOTOR_PINS, 4);

static hf::DSMX_ESP32_Serial1 receiver = hf::DSMX_ESP32_Serial1(CHANNEL_MAP, DEMAND_SCALE, SERIAL1_RX, SERIAL1_TX);  

static hf::MixerQuadXMW mixer(&motors);

static hf::USFS imu;

static hf::TinyPico board;

static hf::Hackflight h = hf::Hackflight(&board, &imu, &receiver, &mixer);

static hf::RatePid ratePid = hf::RatePid(0.04, 0.002, 0.01);
static hf::YawPid yawPid = hf::YawPid(0.10, 0.01);
static hf::LevelPid levelPid = hf::LevelPid(0.20);

void setup(void)
{
    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Adjust trim
    receiver.setTrimPitch(0.05);

    // Start Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
