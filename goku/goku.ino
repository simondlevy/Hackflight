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

#include "bootloader.hpp"

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
        hackflight.loop1(Mixer::rotorCount, Mixer::mix);
    }
}

static const uint8_t REBOOT_TASK_PRIORITY = 2;
static const float REBOOT_TASK_FREQ = 10;
static FreeRtosTask rebootTask;
static void runRebootTask(void *)
{
    while (true) {
        FreeRtosTask::wait(REBOOT_TASK_FREQ);
        if (Serial.available() && Serial.read() == 'R') {
            Bootloader::jump();
        }
    }
}


void setup()
{
    static HardwareSerial uart = HardwareSerial(PC7, PC6);

    hackflight.init1(PC14, true, &uart);

    task1.init(runTask1, "task1", NULL, TASK1_PRIORITY);

    rebootTask.init(runRebootTask, "reboot", NULL, REBOOT_TASK_PRIORITY);

    vTaskStartScheduler();
}

void loop()
{  

}
