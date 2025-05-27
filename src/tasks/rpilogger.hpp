/**
 * Copyright (C) 2025 Simon D. Levy
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

#include <stdint.h>

#include <debug.h>
#include <msp/messages.h>
#include <msp/serializer.hpp>
#include <system.h>
#include <task.hpp>
#include <tasks/estimator.hpp>

class RpiLoggerTask {

    public:

        void begin(EstimatorTask * estimatorTask)
        {
            if (_task.didInit()){
                return;
            }

            _estimatorTask = estimatorTask;

            _task.init(runRpiLoggerTask, "rpilogger", this, 3);
        }

    private:
        
        static constexpr float FREQ_HZ = 100;

        static void runRpiLoggerTask(void * obj)
        {
            ((RpiLoggerTask *)obj)->run();
        }

        FreeRtosTask _task;

        EstimatorTask * _estimatorTask;

        void run(void)
        {
            systemWaitStart();

            TickType_t lastWakeTime = xTaskGetTickCount();

            MspSerializer serializer = {};

            while (true) {

                vehicleState_t state = {};

                _estimatorTask->getVehicleState(&state);

                serializer.serializeFloats(MSP_STATE, (float *)&state, 12);

                for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                    void uartWriteByte(const uint8_t);
                    uartWriteByte(serializer.payload[k]);
                }

                vTaskDelayUntil(&lastWakeTime, M2T(1000/FREQ_HZ));
            }
        }
};
