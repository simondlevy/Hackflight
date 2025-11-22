/**
 *
 * Copyright 2025 Simon D. Levy
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

#include <hackflight.hpp>
#include <free_rtos_task.hpp>
#include <mixers/crazyflie.hpp>

static Hackflight hackflight;

static const uint8_t TASK1_PRIORITY = 5;
static FreeRtosTask task1;
static void runTask1(void *)
{
    while (true) {
        vTaskDelay(1); // yield
        hackflight.task1(Mixer::rotorCount, Mixer::mix);
    }
}

static const float TASK2_FREQ = 70;
static const uint8_t TASK2_PRIORITY = 3;
static FreeRtosTask task2;
static void runTask2(void *)
{
    while (true) {
        FreeRtosTask::wait(TASK2_FREQ);
        hackflight.task2();
    }
}

void setup() 
{
    static HardwareSerial uart = HardwareSerial(PA3, PA2);

    static SPIClass spi = SPIClass(PA7, PA6, PA5);

    spi.begin();

    hackflight.init(PC0, true, &uart, &Wire, &spi, PB4);

    task1.init(runTask1, "task1", NULL, TASK1_PRIORITY);

    task2.init(runTask2, "task2", NULL, TASK2_PRIORITY);

    vTaskStartScheduler();
}

void loop() 
{
}
