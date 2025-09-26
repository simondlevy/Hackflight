/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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

#include <Wire.h>

#include <hackflight.h>
#include <mixers/crazyflie.hpp>
#include <motors.hpp>
#include <safety.hpp>

#include <tasks/imu.hpp>
#include <tasks/led.hpp>

static Motors motors;

static Safety safety = Safety(&motors);

static ImuTask imuTask;
static LedTask ledTask;

void setup() 
{
    ledTask.begin(&safety, &imuTask);
}

void loop()
{
}
