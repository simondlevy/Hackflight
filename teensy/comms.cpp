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

void Hackflight::comms_init()
{
    Serial1.begin(115200);
}

bool Hackflight::comms_read_byte(uint8_t * byte)
{
    if (Serial1.available()) {
        *byte = Serial1.read();
        return true;
    }

    return false;
}
            
void Hackflight::comms_write_byte(const uint8_t byte)
{
    Serial1.write(byte);
}
