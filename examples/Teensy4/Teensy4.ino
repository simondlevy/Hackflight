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

static void task1(void*) 
{

    while (true) {

        static uint32_t prev;
        auto msec = millis();

        if (msec - prev > 500) {

            static bool flag;

            digitalWriteFast(LED_BUILTIN, flag);
        
            flag = !flag;

            prev = msec;
        }

        vTaskDelay(1);
    }
}

static void task2(void*) 
{

    while (true) {

        static uint32_t prev;
        auto msec = millis();

        if (msec - prev > 1000) {

            static bool flag;

            Serial.println(flag ? "tick" : "tock");

            flag = !flag;

            prev = msec;
        }


        vTaskDelay(1);
    }
}


void setup() 
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);

    xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop(void) 
{
}
