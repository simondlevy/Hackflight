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


#define _MAIN

#include "free_rtos_task.hpp"

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

static const uint8_t LOOP1_TASK_PRIORITY = 5;
static FreeRtosTask loop1Task;
static void runLoop1Task(void *)
{
    while (true) {
        FreeRtosTask::wait(1); // yield to loop2 task
        hackflight.loop1(Mixer::rotorCount, Mixer::mix);
    }
}

static const float LOOP2_TASK_FREQ = 70;
static const uint8_t LOOP2_TASK_PRIORITY = 3;

static FreeRtosTask loop2Task;
static void runLoop2Task(void *)
{
    while (true) {
        FreeRtosTask::wait(LOOP2_TASK_FREQ);
        hackflight.loop2();
    }
}

void setup() 
{
    static HardwareSerial uart = HardwareSerial(PA3, PA2);

    static SPIClass spi;

    spi.setSCLK(PA5);
    spi.setMISO(PA6);
    spi.setMOSI(PA7);

    spi.begin();

    hackflight.init(PC0, true, &uart, &Wire, &spi, PB4);

    loop1Task.init(runLoop1Task, "loop1", NULL, LOOP1_TASK_PRIORITY);

    loop2Task.init(runLoop2Task, "loop2", NULL, LOOP2_TASK_PRIORITY);

    vTaskStartScheduler();
}

void loop() 
{
}
