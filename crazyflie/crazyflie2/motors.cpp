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

#include <hackflight.hpp>

static const uint8_t M1_PIN = PA1;
static const uint8_t M2_PIN = PB11;
static const uint8_t M3_PIN = PA15;
static const uint8_t M4_PIN = PB9;

static uint8_t pulse_widths[4];

void Hackflight::motors_init()
{
}

void Hackflight::motors_setSpeed(uint32_t id, float speed)
{
    pulse_widths[id] = (uint8_t)(255 * speed);
}

void Hackflight::motors_run()
{
    analogWrite(M1_PIN, pulse_widths[0]);
    analogWrite(M2_PIN, pulse_widths[1]);
    analogWrite(M3_PIN, pulse_widths[2]);
    analogWrite(M4_PIN, pulse_widths[3]);
}


