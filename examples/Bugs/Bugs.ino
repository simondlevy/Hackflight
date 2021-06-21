/*
   Hackflight sketch for TinyPICO with USFS IMU, DSMX receiver, and standard motors

   Copyright (c) 2021 Simon D. Levy
 
 */

#include "hackflight.hpp"
#include "boards/realboards/tinypico.hpp"
#include "receivers/arduino/dsmx.hpp"
#include "mixers/quadxmw.hpp"
#include "motors/standard.hpp"
#include "imus/usfs.hpp"

#include "imus/mock.hpp"
#include "receivers/mock.hpp"
#include "motors/mock.hpp"

static const uint8_t SERIAL1_RX = 4;
static const uint8_t SERIAL1_TX = 14; // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 8.0f;

static const uint8_t MOTOR_PINS[4] = {25, 26 ,27, 15};

//hf::StandardMotor motors = hf::StandardMotor(MOTOR_PINS, 4);
static hf::MockMotor motors;

static hf::MockReceiver receiver;

static hf::MixerQuadXMW mixer(&motors);

static hf::USFS imu;

static hf::TinyPico board;

static hf::Hackflight h = hf::Hackflight(&board, &imu, &receiver, &mixer);

// Timer task for DSMX serial receiver
static void receiverTask(void * params)
{
    while (true) {

        if (Serial1.available()) {
            //rc.handleSerialEvent(Serial1.read(), micros());
        }

        delay(1);
    }
}


void setup(void)
{
    // Start receiver on Serial1
    // Serial1.begin(115000, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    h.begin();

    // Start the receiver timed task
    // TaskHandle_t task;
    // xTaskCreatePinnedToCore(receiverTask, "Task", 10000, NULL, 1, &task, 0);
}

void loop(void)
{
    h.update();
}
