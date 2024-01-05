/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <free_rtos.h>
#include <task.h>
#include <queue.h>

#include "crtp/crtp_commander.h"

#include <linalg.h>
#include <num.hpp>
#include <datatypes.h>
#include <config.h>

class Commander {

    public:

        static const uint8_t PRIORITY_LOWEST    = 1;
        static const uint8_t PRIORITY_HIGHLEVEL = 1;
        static const uint8_t PRIORITY_CRTP      = 2;
        static const uint8_t PRIORITY_EXTRX     = 3;

        void init(void)
        {    
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

            crtpCommanderInit();
            //crtpCommanderHighLevelInit();
            lastUpdate = xTaskGetTickCount();

            didInit = true;

        }

        bool test(void)
        {
            return didInit;
        }

        void setSetpoint(setpoint_t *setpoint, int priority)
        {
            int currentPriority = 0;

            const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);

            if (peekResult != pdTRUE) {
                return;
            }

            if (priority >= currentPriority) {
                setpoint->timestamp = xTaskGetTickCount();
                // This is a potential race but without effect on functionality
                xQueueOverwrite(setpointQueue, setpoint);
                xQueueOverwrite(priorityQueue, &priority);
                if (priority > PRIORITY_HIGHLEVEL) {
                    // Stop the high-level planner so it will forget its current state
                    //crtpCommanderHighLevelStop();
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
            int priority = PRIORITY_LOWEST;
            xQueueOverwrite(priorityQueue, &priority);
        }

        void getDemands(demands_t & demands, uint32_t & timestamp, bool & inHoverMode)
        {
            setpoint_t setpoint = {};

            xQueuePeek(setpointQueue, &setpoint, 0);

            lastUpdate = setpoint.timestamp;

            timestamp = setpoint.timestamp;

            inHoverMode = setpoint.mode.z != modeDisable;

            // Demands start with setpoints (sticks).  For unformity with
            // other kinds of open-loop control (e.g. R/C receivers) we
            // normalize demands to [-1,+1]
            demands.thrust = setpoint.thrust / UINT16_MAX;
            demands.roll   = setpoint.attitude.roll / 30;
            demands.pitch  = setpoint.attitude.pitch / 30;
            demands.yaw    = setpoint.attitudeRate.yaw / 200;

            if (inHoverMode) {

                demands.thrust = Num::rescale(
                        setpoint.position.z, 0.2, 2.0, -1, +1);

                // In hover mode, velocity demands are already in [-1,+1]
                demands.roll = setpoint.velocity.y;
                demands.pitch = setpoint.velocity.x;
            }
        }

    private:

        bool didInit;

        const setpoint_t nullSetpoint = {};

        const int priorityDisable = 0;

        uint32_t lastUpdate;

        static const size_t QUEUE_LENGTH = 1;

        static const auto SETPOINT_ITEM_SIZE = sizeof(setpoint_t);
        uint8_t setpointQueueStorage[QUEUE_LENGTH * SETPOINT_ITEM_SIZE];
        StaticQueue_t setpointQueueBuffer;
        xQueueHandle setpointQueue;

        static const auto PRIORITY_ITEM_SIZE = sizeof(int);
        uint8_t priorityQueueStorage[QUEUE_LENGTH * PRIORITY_ITEM_SIZE];
        StaticQueue_t priorityQueueBuffer;
        xQueueHandle priorityQueue;

};
