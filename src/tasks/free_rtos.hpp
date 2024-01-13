/**
 * A superclass for FreeRTOS tasks
 *
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

#include <stdint.h>

#include <free_rtos.h>
#include <task.h>

class FreeRTOSTask {

    private:

        typedef void (*taskfun_t)(void * obj);

        static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

        StackType_t  _taskStackBuffer[STACKSIZE]; 

        StaticTask_t _taskTaskBuffer;

    protected:

        void begin(
                const taskfun_t fun,
                const char * name,
                void * obj,
                const uint8_t priority
                )
        {
            xTaskCreateStatic(
                    fun, 
                    name, 
                    STACKSIZE, 
                    obj, 
                    priority, 
                    _taskStackBuffer,
                    &_taskTaskBuffer);
        }

    public:

        // May be shared with logger/params
        bool didInit;

};
