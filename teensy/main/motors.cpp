/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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
#include <oneshot125.hpp>

#include <tasks/motors.hpp>

static const std::vector<uint8_t> PINS = {2, 23, 14, 9};

static auto motors = OneShot125(PINS);

void MotorsTask::device_init()
{
    motors.arm();
}

void MotorsTask::device_setRatio(const uint8_t id, const uint16_t ratio)
{
    motors.set(0, 125);
    motors.set(1, 125);
    motors.set(2, 125);
    motors.set(3, 125);

    motors.run();
}

void MotorsTask::device_stop()
{
    motors.set(0, 125);
    motors.set(1, 125);
    motors.set(2, 125);
    motors.set(3, 125);

    motors.run();
}

