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

#include <system.h>

static const uint8_t LED_PIN = 5;
static const uint8_t FLOWDECK_CS_PIN = 10;

FLASHMEM __attribute__((noinline)) void setup() 
{
    Serial.begin(0);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    systemInit(LED_PIN, FLOWDECK_CS_PIN);
}

void loop() {}

void debug(const char * msg)
{
  printf("%s\n", msg);
}
