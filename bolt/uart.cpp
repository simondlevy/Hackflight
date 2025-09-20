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

#pragma once

#include <Arduino.h>

#include <tasks/logging.hpp>
#include <tasks/setpoint.hpp>

static HardwareSerial serial = HardwareSerial(PA3, PA2);

bool SetpointTask::uart_read_byte(uint8_t * byte)
{
    if (serial.available()) {
        *byte = serial.read();
        return true;
    }

    return false;
}
            
void LoggingTask::uart_write_byte(const uint8_t byte)
{
    (void)byte;
    //serial.write(byte);
}
