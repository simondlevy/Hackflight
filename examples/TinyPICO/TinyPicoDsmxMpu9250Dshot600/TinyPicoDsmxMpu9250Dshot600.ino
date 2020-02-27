/*
   Hackflight sketch for TinyPICO with Ultimate Sensor Fusion Solution IMU and DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/MPU
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

       https://github.com/plerup/espsoftwareserial


   Copyright (c) 2019 Simon D. Levy

   This file is part of Hackflight.
   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "hackflight.hpp"
#include "boards/realboards/tinypico.hpp"
//#include "receivers/arduino/dsmx.hpp"
#include "receivers/mock.hpp"
#include "actuators/mixers/quadxcf.hpp"
#include "pidcontrollers/rate.hpp"
#include "pidcontrollers/level.hpp"
#include "motors/mock.hpp"
#include "imus/softquats/mpu9250.hpp"

static const uint8_t SERIAL1_RX = 32;
static const uint8_t SERIAL1_TX = 33; // unused

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};

static constexpr float DEMAND_SCALE = 8.0f;

hf::Hackflight h;

hf::MPU9250SoftwareQuaternionIMU imu;

hf::MockReceiver rc;

hf::MixerQuadXCF mixer;

hf::RatePid ratePid = hf::RatePid(0.05f, 0.00f, 0.00f, 0.10f, 0.01f); 

hf::LevelPid levelPid = hf::LevelPid(0.20f);

hf::MockMotor motor1;
hf::MockMotor motor2;
hf::MockMotor motor3;
hf::MockMotor motor4;

hf::Motor * motors[4] = { &motor1, &motor2, &motor3, &motor4 };

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
    // Use D18,19 for USFS power, ground
    hf::ArduinoBoard::powerPins(18, 19);

    // Start receiver on Serial1
    Serial1.begin(115000, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    // Initialize Hackflight firmware
    h.init(new hf::TinyPico(), &imu, &rc, &mixer, motors);

    // Add Rate and Level PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);

    // Start the receiver timed task
    TaskHandle_t task;
    xTaskCreatePinnedToCore(receiverTask, "Task", 10000, NULL, 1, &task, 0);
}

void loop(void)
{
    h.update();
}
