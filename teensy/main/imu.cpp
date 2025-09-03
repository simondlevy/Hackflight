
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

#include <Wire.h>
#include <BMI088.h>

#include <tasks/imu.hpp>

static const uint8_t ACCEL_ADDRESS = 0x19;
static const uint8_t GYRO_ADDRESS = 0x69;

static const uint8_t GYRO_INTERRUPT_PIN = 6;

static Bmi088Accel accel(Wire, ACCEL_ADDRESS);

static Bmi088Gyro gyro(Wire, GYRO_ADDRESS);

bool ImuTask::device_init()
{
    Wire.begin();
    Wire.setClock(400000);

    if (accel.begin() < 0) {
        return false;
    }

    if (gyro.begin() < 0) {
        return false;
    }

    accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);

    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
    gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
    gyro.mapDrdyInt3(true);

    return true;
}
                    
void ImuTask::device_readRaw(
        int16_t & gx, int16_t & gy, int16_t & gz, 
        int16_t & ax, int16_t & ay, int16_t & az)
{
    gyro.readSensor();
    gx = gyro.getGyroX_raw();
    gy = gyro.getGyroY_raw();
    gz = gyro.getGyroZ_raw();

    accel.readSensor();
    ax = accel.getAccelX_raw();
    ay = accel.getAccelY_raw();
    az = accel.getAccelZ_raw();
}

uint8_t ImuTask::device_getInterruptPin()
{
    return GYRO_INTERRUPT_PIN;
}
