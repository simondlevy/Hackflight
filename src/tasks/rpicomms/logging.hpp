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

#include <msp/messages.h>
#include <msp/serializer.hpp>
#include <system.h>
#include <task.hpp>
#include <tasks/estimator.hpp>

class LoggerTask {

    public:

        void begin(EstimatorTask * estimatorTask)
        {
            if (_task.didInit()){
                return;
            }

            _estimatorTask = estimatorTask;

            _task.init(runLoggerTask, "logger", this, 3);
        }

    private:
        
        static constexpr float FREQ_HZ = 100;

        static void runLoggerTask(void * obj)
        {
            ((LoggerTask *)obj)->run();
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

                const float statevals[10] = {
                    state.dx,
                    state.dy,
                    state.z,
                    state.dz,
                    state.phi,
                    state.dphi,
                    state.theta,
                    state.dtheta,
                    state.psi,
                    state.dpsi
                };

                serializer.serializeFloats(MSP_STATE, statevals, 10);

                for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                    systemUartWriteByte(serializer.payload[k]);
                }

                vTaskDelayUntil(&lastWakeTime, M2T(1000/FREQ_HZ));
            }
        }
};
