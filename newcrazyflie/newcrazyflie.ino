/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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


#include "free_rtos_task.hpp"

#include <newhackflight.hpp>
#include <mixers/crazyflie.hpp>

static const uint8_t LOOP2_TASK_PRIORITY = 3;

static Hackflight hackflight;

static FreeRtosTask loop2Task;

static void runLoop2Task(void *obj)
{
//    ((hackflight *)obj)->loop2();
}


void setup() 
{
    hackflight.init();

    loop2Task.init(runLoop2Task, "loop2", &hackflight, LOOP2_TASK_PRIORITY);

    vTaskStartScheduler();
}

void loop() 
{
}
