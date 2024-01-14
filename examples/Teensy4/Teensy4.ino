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

#include <Wire.h>

#include <hfheader.h>
#include <safety.hpp>

#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/receiver.hpp>
#include <tasks/visualizer.hpp>

void setup() 
{

    static EstimatorTask estimatorTask;

    static ImuTask imuTask;

    static ReceiverTask receiverTask;

    static VisualizerTask visualizerTask;

    static Safety safety;

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(100);

    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    receiverTask.begin();

    estimatorTask.begin(&safety);

    imuTask.begin(&estimatorTask, 0, 0);

    visualizerTask.begin(&estimatorTask, &receiverTask);

    vTaskStartScheduler();
}

void loop(void) 
{
}
