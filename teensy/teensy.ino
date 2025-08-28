/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    main.cpp
 * @brief   FreeRTOS example for Teensy boards
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

#include "arduino_freertos.h"


static void task1(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    vTaskDelete(nullptr);
}

static void task2(void*) {
    Serial.begin(0);
    while (true) {
        Serial.println("TICK");
        vTaskDelay(pdMS_TO_TICKS(1'000));

        Serial.println("TOCK");
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }

    vTaskDelete(nullptr);
}

void setup() {
    Serial.begin(0);
    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
    xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);

    Serial.println(PSTR("setup(): starting scheduler..."));
    Serial.flush();

    vTaskStartScheduler();
}

void loop() {}
