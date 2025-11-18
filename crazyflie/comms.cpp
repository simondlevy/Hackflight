/**
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

#include <Arduino.h>

#include <hackflight.hpp>

static const uint8_t RX_PIN = PA3;
static const uint8_t TX_PIN = PA2;

static HardwareSerial serial = HardwareSerial(RX_PIN, TX_PIN);

void Hackflight::comms_init()
{
    serial.begin(115200);
}

bool Hackflight::comms_read_byte(uint8_t * byte)
{
    if (serial.available()) {
        *byte = serial.read();
        return true;
    }

    return false;
}
            
void Hackflight::comms_write_byte(const uint8_t byte)
{
    serial.write(byte);
}
