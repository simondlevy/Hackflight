/*
 * Hackflight ESPNOW onboard radio sketch
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

static const uint8_t DONGLE_ADDRESS[] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

static HardwareSerial serial(1);

void setup(void)
{
    serial.begin(115200, SERIAL_8N1, 2, 1); 
    EspNowTransponder::begin(DONGLE_ADDRESS, &serial);
}


void loop(void)
{
    EspNowTransponder::step();
}
