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

#include <__control__.hpp>

#include <msp/messages.h>
#include <msp/serializer.hpp>
#include <task.hpp>
#include <tasks/estimator.hpp>
#include <uart_api.h>

class LoggingTask {

    public:

        void begin(EstimatorTask * estimatorTask,
                ClosedLoopControl * closedLoopControl)
        {
            if (_task.didInit()){
                return;
            }

            _estimatorTask = estimatorTask;

            _closedLoopControl = closedLoopControl;

            _task.init(runLoggingTask, "logger", this, 3);
        }

    private:
        
        static constexpr float FREQ_HZ = 100;

        static void runLoggingTask(void * obj)
        {
            ((LoggingTask *)obj)->run();
        }

        FreeRtosTask _task;

        EstimatorTask * _estimatorTask;

        ClosedLoopControl * _closedLoopControl;

        void run(void)
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                sendVehicleState();

                sendClosedLoopControlMessage();

                vTaskDelayUntil(&lastWakeTime, 1000/FREQ_HZ);
            }
        }

        void sendVehicleState()
        {
            vehicleState_t state = {};

            MspSerializer serializer = {};

            _estimatorTask->getVehicleState(&state);

            const float statevals[10] = { state.dx, state.dy, state.z,
                state.dz, state.phi, state.dphi, state.theta,
                state.dtheta, state.psi, state.dpsi 
            };

            serializer.serializeFloats(MSP_STATE, statevals, 10);

            sendPayload(serializer);
        }

        void sendClosedLoopControlMessage()
        {
            MspSerializer serializer = {};

            _closedLoopControl->serializeMessage(serializer);

            sendPayload(serializer);
        }
 
        void sendPayload(const MspSerializer & serializer) {
            for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                uart_write_byte(serializer.payload[k]);
            }
        }
};
