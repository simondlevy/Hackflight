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

#include "arduino_freertos.h"
#include "avr/pgmspace.h"

static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; 

static const uint8_t LED_PIN = 5;

static StackType_t  _taskStackBuffer1[STACKSIZE]; 
static StaticTask_t _taskTaskBuffer1;
static void task1(void*) {
    pinMode(LED_PIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(LED_PIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(LED_PIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static StackType_t  _taskStackBuffer2[STACKSIZE]; 
static StaticTask_t _taskTaskBuffer2;
static void task2(void*) {
    Serial.begin(0);
    while (true) {
        Serial.println("TICK");
        vTaskDelay(pdMS_TO_TICKS(1'000));

        Serial.println("TOCK");
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }
}

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);

    //delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    xTaskCreateStatic(
        task1, 
        "task1", 
        STACKSIZE, 
        nullptr, 
        2, 
        _taskStackBuffer1,
        &_taskTaskBuffer1);

    xTaskCreateStatic(
        task2, 
        "task2", 
        STACKSIZE, 
        nullptr, 
        2, 
        _taskStackBuffer2,
        &_taskTaskBuffer2);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}
