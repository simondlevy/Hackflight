/**
 *
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

#include <arduino_freertos.h>
#include <task.hpp>

static const uint8_t LED_PIN = 5;

static FreeRtosTask _task;

static void led_task(void*) 
{
    pinMode(LED_PIN, arduino::OUTPUT);

    while (true) {
        Serial.println("TICK");
        digitalWriteFast(LED_PIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        Serial.println("TOCK");
        digitalWriteFast(LED_PIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    _task.init( led_task, "led_task", nullptr, 2);

    vTaskStartScheduler();
}

void loop() 
{
}
