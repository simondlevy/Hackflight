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

#include <safety.hpp>
#include <task.hpp>

namespace arduino {
#include "led2.hpp"
}

static const uint8_t LED_PIN = 5;

static Safety _safety;

static arduino::LedTask _ledTask;

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    //_ledTask.init(task1, "task1", nullptr, 2);
    _ledTask.begin(&_safety, LED_PIN);

    vTaskStartScheduler();
}

void loop() {}
