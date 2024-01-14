/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <math.h>

#include <task.hpp>
#include <tasks/estimator.hpp>

#include <crossplatform.h>
#include <lpf.hpp>
#include <num.hpp>
#include <physicalConstants.h>
#include <datatypes.h>

class ImuTask : public FreeRTOSTask {

    public: // Are are called from CoreTask

        void begin(
                EstimatorTask * estimatorTask, 
                const float calibRoll,
                const float calibPitch)
        {
            if (didInit) {
                return;
            }

            FreeRTOSTask::begin(runImuTask, "imu", this, 4);

            didInit = true;
        }

        void dataAvailableCallback(void)
        {
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            interruptTimestamp = micros();
            xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

            if (xHigherPriorityTaskWoken) {
                portYIELD();
            }
        }


    private:


        static void runImuTask(void *obj)
        {
            ((ImuTask *)obj)->run();
        }

        volatile uint64_t interruptTimestamp;

        xSemaphoreHandle sensorsDataReady;
 
        void run(void)
        {
            while (true) {

                //if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
                //}

                vTaskDelay(1);

            }

        }
        // Hardware-dependent
        bool gyroSelfTest();
        void deviceInit(void); 
        void readGyro(Axis3i16* dataOut);
        void readAccel(Axis3i16* dataOut);
};
