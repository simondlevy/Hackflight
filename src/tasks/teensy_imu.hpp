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

            FreeRTOSTask::begin(runImuTask, "imu", this, 3);

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

        bool gyroBiasFound;
        sensorData_t data;
        Axis3i16 gyroRaw;
        Axis3i16 accelRaw;
        Axis3f gyroBias;
        float accScale;
        bool accScaleFound;
        uint32_t accScaleSumCount;
        volatile uint64_t interruptTimestamp;

        xSemaphoreHandle sensorsDataReady;
        StaticSemaphore_t sensorsDataReadyBuffer;
        xSemaphoreHandle dataReady;
        StaticSemaphore_t dataReadyBuffer;

 
        void run(void)
        {
            while (true) {

                Axis3f gyroScaledIMU;
                Axis3f accScaledIMU;
                Axis3f accScaled;

                if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {

                    data.interruptTimestamp = interruptTimestamp;

                    // Get data from chosen sensors 
                    readGyro(&gyroRaw);
                    readAccel(&accelRaw);

                }

                xSemaphoreGive(dataReady);

                vTaskDelay(1);

            }

        }
        // Hardware-dependent
        bool gyroSelfTest();
        void deviceInit(void); 
        void readGyro(Axis3i16* dataOut);
        void readAccel(Axis3i16* dataOut);
};
