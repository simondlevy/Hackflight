/*
   Copyright (c) 2024 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <FreeRTOS_TEENSY4.h>
#include <task.h>

#include <vl53l1_arduino.h>

#include <hfheader.h>
#include <visualizer.hpp>
#include <mixers/quadrotor.hpp>
#include <tasks/free_rtos/newcore.hpp>

static Visualizer visualizer;

void serialEvent(void)
{
    while (Serial.available()) {

        if (visualizer.parse(Serial.read())) {

            while (visualizer.available()) {

                Serial.write(visualizer.read());
            }
        }
    }
}

static void getOpenLoopDemands(
        demands_t & demands, uint32_t & timestamp, bool & inHoverMode)
{
}

void setup() 
{
    (void)getOpenLoopDemands;
    (void)mixQuadrotor;

    Serial.begin(115200);

    Wire.begin();

    pinMode(LED_BUILTIN, OUTPUT);

    static VL53L1_Arduino vl53l1;    

    static CoreTask coreTask;

    coreTask.begin(0, 0, SS, &vl53l1, getOpenLoopDemands, mixQuadrotor, true);

    //vl53l1.begin();

    vTaskStartScheduler();
}

void loop(void) 
{
}
