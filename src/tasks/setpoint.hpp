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

#include <comms.hpp>
#include <msp/__messages__.h>
#include <msp/parser.hpp>
#include <tasks/debug.hpp>

class SetpointTask {

    private:

        static constexpr float FREQ_HZ = 1000;

    public:

        void begin(DebugTask * debugTask=nullptr)
        {
            _debugTask = debugTask;

            setpointQueue = xQueueCreateStatic(
                    QUEUE_LENGTH,
                    SETPOINT_ITEM_SIZE,
                    setpointQueueStorage,
                    &setpointQueueBuffer);

            xQueueSend(setpointQueue, &nullSetpoint, 0);

            priorityQueue = xQueueCreateStatic(
                    QUEUE_LENGTH,
                    PRIORITY_ITEM_SIZE,
                    priorityQueueStorage,
                    &priorityQueueBuffer);

            xQueueSend(priorityQueue, &priorityDisable, 0);

            _task.init(runSetpointTask, "setpoint", this, 3);
        }

        void getSetpoint(setpoint_t & setpoint)
        {
            xQueuePeek(setpointQueue, &setpoint, 0);
        }

    private:

        static const uint8_t PRIORITY_LOW  = 1;
        static const uint8_t PRIORITY_HIGH = 2;

        static const uint16_t MIN_THRUST = 1000;
        static const uint16_t MAX_THRUST = 60000;

        static void runSetpointTask(void * obj)
        {
            ((SetpointTask *)obj)->run();
        }

        const setpoint_t nullSetpoint = {};

        const int priorityDisable = 0;

        static const size_t QUEUE_LENGTH = 1;

        static const auto SETPOINT_ITEM_SIZE = sizeof(setpoint_t);
        uint8_t setpointQueueStorage[QUEUE_LENGTH * SETPOINT_ITEM_SIZE];
        StaticQueue_t setpointQueueBuffer;
        QueueHandle_t setpointQueue;

        static const auto PRIORITY_ITEM_SIZE = sizeof(int);
        uint8_t priorityQueueStorage[QUEUE_LENGTH * PRIORITY_ITEM_SIZE];
        StaticQueue_t priorityQueueBuffer;
        QueueHandle_t priorityQueue;

        FreeRtosTask _task;

        DebugTask * _debugTask;

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
                            setpoint.armed = !setpoint.armed;
                            break;

                        case MSP_SET_IDLE:
                            setpoint.hovering = false;
                            setSetpoint(&setpoint, PRIORITY_HIGH);
                            break;

                        case MSP_SET_HOVER:
                            setpoint.hovering = true;
                            setpoint.demands.pitch = parser.getFloat(0);
                            setpoint.demands.roll = parser.getFloat(1);
                            setpoint.demands.yaw = parser.getFloat(2);
                            setpoint.demands.thrust = parser.getFloat(3);
                            setSetpoint(&setpoint, PRIORITY_HIGH);
                            break;

                        default:
                            break;
                    }
                }

            }
        }

        int getActivePriority(void)
        {
            int priority = 0;

            const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);

            return peekResult == pdTRUE ? priority : 0;
        }

        void relaxPriority(void)
        {
            int priority = PRIORITY_LOW;
            xQueueOverwrite(priorityQueue, &priority);
        }
        void setSetpoint(setpoint_t *setpoint, int priority)
        {
            int currentPriority = 0;

            const BaseType_t peekResult =
                xQueuePeek(priorityQueue, &currentPriority, 0);

            if (peekResult != pdTRUE) {
                return;
            }

            if (priority >= currentPriority) {
                setpoint->timestamp = xTaskGetTickCount();
                // This is a potential race but without effect on functionality
                xQueueOverwrite(setpointQueue, setpoint);
                xQueueOverwrite(priorityQueue, &priority);
            }        
        }
};
