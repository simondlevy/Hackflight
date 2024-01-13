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

#include <FreeRTOS_TEENSY4.h>
#include <task.h>

class FreeRTOSTask {

    public:

        typedef void (*taskfun_t)(void * obj);

        static void create(
                FreeRTOSTask * task,
                taskfun_t fun,
                const char * name, 
                const uint8_t priority)
        {
            xTaskCreateStatic(
                    fun, 
                    name, 
                    STACKSIZE, 
                    task, 
                    priority, 
                    task->stackBuffer,
                    &task->taskBuffer);
         }

    private:

        static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

        StackType_t  stackBuffer[STACKSIZE]; 

        StaticTask_t taskBuffer;
};
