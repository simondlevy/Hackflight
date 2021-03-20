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
#include <rft_motors/mock.hpp>

#include "hackflight.hpp"
#include "boards/tinypico_belly.hpp"
#include "mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "sensors/usfs.hpp"

#include "receivers/mock.hpp"

hf::TinyPicoBelly board;

hf::MockReceiver receiver;

rft::MockMotor motors;

static hf::MixerQuadXCF mixer(&motors);

static hf::Hackflight h(&board, &receiver, &mixer);

static hf::UsfsGyro gyro;
static hf::UsfsQuat quat;

/*
// Timer task for DSMX serial receiver
static void receiverTask(void * params)
{
    while (true) {

        if (Serial1.available()) {
            rc.handleSerialEvent(Serial1.read(), micros());
        }

        delay(1);
    }
}
*/

void setup(void)
{
    // Start receiver on Serial1
    //Serial1.begin(115000, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    // Add gyro, quaternion sensors
    h.addSensor(&gyro);
    h.addSensor(&quat);

    // Add PID controllers
    //h.addPidController(&levelPid);
    //h.addPidController(&ratePid);

    // Start the receiver timed task
    //TaskHandle_t task;
    //xTaskCreatePinnedToCore(receiverTask, "ReceiverTask", 10000, NULL, 1, &task, 1);

    // Initialize Hackflight firmware
    h.begin();
}

void loop(void)
{
    h.update();
}
