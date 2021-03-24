/*
   Hackflight sketch for TinyPICO with USFS IMU, DSMX receiver, and standard motors

   Additional libraries needed:

       https://github.com/simondlevy/RoboFirmwareToolkit
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/DSMRX

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <RoboFirmwareToolkit.hpp>
#include <rft_motors/standard.hpp>

#include "hackflight.hpp"
#include "boards/tinypico_belly.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

#include "receivers/arduino/dsmx/dsmx_esp32_serial1.hpp"

// Receiver --------------------------------------------------------------

static const uint8_t SERIAL1_RX = 32;
static const uint8_t SERIAL1_TX = 33;  // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 8.0f;

hf::DSMX_Receiver receiver = hf::DSMX_Receiver(CHANNEL_MAP, DEMAND_SCALE);  

static void receiverTask(void * params)
{
    while (true) {

        if (Serial1.available()) {
            receiver.handleSerialEvent(Serial1.read(), micros());
        }

        delay(1);
    }
}

// Motors ----------------------------------------------------------------

static const uint8_t MOTOR_PINS[4] = {25, 26 ,27, 15};

rft::StandardMotor motors = rft::StandardMotor(MOTOR_PINS, 4);

// -----------------------------------------------------------------------

hf::TinyPicoBelly board;

static hf::UsfsGyro gyro;
static hf::UsfsQuat quat;
static hf::RatePid ratePid = hf::RatePid( 0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 
static hf::LevelPid levelPid = hf::LevelPid(0.20f);
static hf::MixerQuadXCF mixer(&motors);

static hf::Hackflight h(&board, &receiver, &mixer);

void setup(void)
{
    // Start receiver on Serial1
    Serial1.begin(115000, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    // Add gyro, quaternion sensors
    h.addSensor(&gyro);
    h.addSensor(&quat);

    // Add PID controllers
    h.addClosedLoopController(&levelPid);
    h.addClosedLoopController(&ratePid);

    // Start the receiver timed task
    TaskHandle_t task;
    xTaskCreatePinnedToCore(receiverTask, "ReceiverTask", 10000, NULL, 1, &task, 1);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
