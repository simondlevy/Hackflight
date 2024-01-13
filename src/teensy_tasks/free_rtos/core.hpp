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

#pragma once

#include <console.h>
#include <crossplatform.h>
#include <rateSupervisor.hpp>
#include <teensy_tasks/free_rtos.hpp>

class CoreTask : public FreeRTOSTask {

    public:

        static void fun(void * obj)
        {
            ((CoreTask *)obj)->run();
        }

        void run(void) {

            static RateSupervisor rateSupervisor;

            vTaskSetApplicationTaskTag(0, (TaskHookFunction_t)TASK_CORE_ID_NBR);

            systemWaitStart();

            consolePrintf("CORE: Wait for sensor calibration...\n");

            while (true) {

                blink();

                vTaskDelay(1);
            }
        }

    private:

        static void blink(void)
        {
            static uint32_t prev;
            auto msec = millis();

            if (msec - prev > 500) {

                static bool ledOn;

                digitalWriteFast(LED_BUILTIN, ledOn);

                ledOn = !ledOn;

                prev = msec;
            }
        }

};
