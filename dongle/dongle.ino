/*
 * Hackflight ESPNOW radio dongle sketch
 *
 * Copyright (C) 2026 Simon D. Levy
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

#include <espnow-transponder.h>

static const uint8_t XIAO_ADDRESS[] = {0x58,0xE6,0xC5,0x14,0x0A,0x34};


void setup()
{
    Serial.begin(115200);

    EspNowTransponder::begin(XIAO_ADDRESS, &Serial);
}

void loop()
{
    EspNowTransponder::step();
}
