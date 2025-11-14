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

class CommandTask {

    private:

        static constexpr float TASK_FREQ = 1000;

    public:

        void begin(DebugTask * debugTask=nullptr)
        {
            _debugTask = debugTask;

            _task.init(runCommandTask, "command", this, 3);
        }

        void getCommand(command_t & command)
        {
            memcpy(&command, &_command, sizeof(command_t));
        }

    private:

        static void runCommandTask(void * obj)
        {
            ((CommandTask *)obj)->run();
        }

        FreeRtosTask _task;

        DebugTask * _debugTask;

        command_t _command;

        void run(void)
        {
            MspParser parser = {};

            command_t command = {};

            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, 1000/TASK_FREQ);

                uint8_t byte = 0;

                while (Comms::read_byte(&byte)) {

                    switch (parser.parse(byte)) {

                        case MSP_SET_ARMING:
                            _command.armed = !_command.armed;
                            break;

                        case MSP_SET_IDLE:
                            _command.hovering = false;
                            break;

                        case MSP_SET_SETPOINT:
                            _command.hovering = true;
                            _command.demands.pitch = parser.getFloat(0);
                            _command.demands.roll = parser.getFloat(1);
                            _command.demands.yaw = parser.getFloat(2);
                            _command.demands.thrust = parser.getFloat(3);
                            break;

                        default:
                            break;
                    }
                }

            }
        }
};
