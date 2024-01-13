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

#include <hfheader.h>
#include <visualizer.hpp>

#include <teensy_tasks/free_rtos/core.hpp>
#include <teensy_tasks/free_rtos/estimator.hpp>

static Visualizer visualizer;

static CoreTask coreTask;

static EstimatorTask estimatorTask;

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

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    FreeRTOSTask::create(&coreTask, CoreTask::fun, "CORE", 2);

    FreeRTOSTask::create(&estimatorTask, EstimatorTask::fun, "ESTIMATOR", 2);

    vTaskStartScheduler();
}

void loop(void) 
{
}
