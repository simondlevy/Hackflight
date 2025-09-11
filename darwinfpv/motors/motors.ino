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

static const std::vector<uint8_t> MOTOR_PINS = {PA1, PB11, PA15, PB10};

static const std::vector<uint8_t> POWER_SWITCH_PINS = {PA0, PB12, PC8, PC15};

static auto motors = OneShot125(MOTOR_PINS);

static void enableMotor(const uint8_t id)
{
    pinMode(POWER_SWITCH_PINS[id], OUTPUT);
    digitalWrite(POWER_SWITCH_PINS[id], HIGH);
}

void MotorsTask::device_init()
{
    enableMotor(0);
    enableMotor(1);
    enableMotor(2);
    enableMotor(3);

    motors.arm(); }

void MotorsTask::device_setRatio(const uint8_t id, const uint16_t ratio)
{
    const uint8_t pulse_usec = map(ratio, 0, 65535, 125, 250);

    motors.set(id, pulse_usec);

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

