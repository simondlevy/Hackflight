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

#include <tasks/imu.hpp>

#include <Wire.h>

// internal I^2C
static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t ACCEL_INTERRUPT_PIN = PC13;
static const uint8_t GYRO_INTERRUPT_PIN = PC14;

static const uint8_t ACCEL_ADDRESS = 0x18;
static const uint8_t GYRO_ADDRESS = 0x69;

void ImuTask::deviceInit(void)
{
}

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
    (void)dataOut;
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    (void)dataOut;
}
