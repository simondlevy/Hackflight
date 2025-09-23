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

#include <Arduino.h>

#include <motors.hpp>

static const uint8_t M1_PIN = PA1;

static uint8_t pulse_widths[4];

void Motors::device_init()
{
}

void Motors::device_setRatio(uint32_t id, uint16_t ratio)
{
    pulse_widths[id] = 255 * (ratio / 65536.f);
}

void Motors::device_run()
{
    analogWrite(M1_PIN, pulse_widths[0]);
}


