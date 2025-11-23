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

#include <hackflight.hpp>
#include <free_rtos_task.hpp>
#include <mixers/crazyflie.hpp>
#include <parser.hpp>

#include "bootloader.hpp"

static const uint8_t TASK1_PRIORITY = 5;
static void runTask1(void *)
{
    Hackflight hackflight = {};

    HardwareSerial uart = HardwareSerial(PC7, PC6);

    hackflight.init(PC14, true, &uart);

    while (true) {
        vTaskDelay(1); // yield
        hackflight.task1(Mixer::rotorCount, Mixer::mix);
    }
}

static const uint8_t TASK2_PRIORITY = 3;
static void runTask2(void *)
{
    HardwareSerial uart = HardwareSerial(PA1, PA0);

    MspParser parser = {};

    uint32_t count = 0;

    while (true) {

        while (uart.available()) {
           if (parser.parse(uart.read()) == MSP_SENSORS) {
               count++;
           }
        }

        static uint32_t msec_prev;
        if (millis() - msec_prev > 1000) {
            Serial.println(count);
            count = 0;
            msec_prev = millis();
        }

        vTaskDelay(1); // yield
    }
}

static const uint8_t REBOOT_TASK_PRIORITY = 2;
static const float REBOOT_TASK_FREQUENCY = 10;
static void runRebootTask(void *)
{
    while (true) {
        FreeRtosTask::wait(REBOOT_TASK_FREQUENCY);
        if (Serial.available() && Serial.read() == 'R') {
            Bootloader::jump();
        }
    }
}

void setup()
{
    static FreeRtosTask task1;
    task1.init(runTask1, "task1", TASK1_PRIORITY);

    static FreeRtosTask task2;
    task2.init(runTask2, "task2", TASK2_PRIORITY);

    static FreeRtosTask rebootTask;
    rebootTask.init(runRebootTask, "reboot", REBOOT_TASK_PRIORITY);

    vTaskStartScheduler();
}

void loop()
{  
}
