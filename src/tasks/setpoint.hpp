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

#include <__messages__.h>
#include <comms.hpp>
#include <msp/parser.hpp>
#include <tasks/debug.hpp>

class SetpointTask {

    private:

        static constexpr float FREQ_HZ = 1000;

    public:

        void begin(DebugTask * debugTask=nullptr)
        {
            _debugTask = debugTask;

            _task.init(runSetpointTask, "setpoint", this, 3);
        }

        void getSetpoint(setpoint_t & setpoint)
        {
            memcpy(&setpoint, &_setpoint, sizeof(setpoint_t));
        }

    private:

        static void runSetpointTask(void * obj)
        {
            ((SetpointTask *)obj)->run();
        }

        FreeRtosTask _task;

        DebugTask * _debugTask;

        setpoint_t _setpoint;

        void run(void)
        {
            MspParser parser = {};

            setpoint_t setpoint = {};

            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, 1000/FREQ_HZ);

                uint8_t byte = 0;

                while (Comms::read_byte(&byte)) {

                    switch (parser.parse(byte)) {

                        case MSP_SET_ARMING:
                            _setpoint.armed = !_setpoint.armed;
                            break;

                        case MSP_SET_IDLE:
                            _setpoint.hovering = false;
                            break;

                        case MSP_SET_SETPOINT:
                            _setpoint.hovering = true;
                            _setpoint.demands.pitch = parser.getFloat(0);
                            _setpoint.demands.roll = parser.getFloat(1);
                            _setpoint.demands.yaw = parser.getFloat(2);
                            _setpoint.demands.thrust = parser.getFloat(3);
                            break;

                        default:
                            break;
                    }
                }

            }
        }
};
