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

#include <teensy_tasks/free_rtos.hpp>

class EstimatorTask : public FreeRTOSTask {

    public:

        static void fun(void * obj)
        {
            ((EstimatorTask *)obj)->run();
        }

        void run(void) {

            while (true) {

                Serial.println("TICK");
                vTaskDelay(pdMS_TO_TICKS(1'000));

                Serial.println("TOCK");
                vTaskDelay(pdMS_TO_TICKS(1'000));

                vTaskDelay(1);
            }
        }
};
