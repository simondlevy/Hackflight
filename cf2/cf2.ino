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

#include <hackflight.h>
#include <motors.h>
#include <system.h>
#include <tasks/imu.hpp>
#include <tasks/zranger.hpp>

static const uint8_t LED_PIN = PC0;

HardwareSerial Serial2(USART2);

void setup() 
{
    Serial.begin(115200);

    Serial2.begin(115200);
}

void loop() 
{
}
