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


#include <Wire.h>   

#include <BMI088.h>

#include <tasks/imu.hpp>

static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;

static const uint8_t ACCEL_ADDR = 0x18;
static const uint8_t GYRO_ADDR = 0x69;

static TwoWire wire = TwoWire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDR);

static Bmi088Gyro gyro(wire, GYRO_ADDR);

static bool failed(const int status)
{
    return status < 0;
}

bool ImuTask::device_init()
{
    if (failed(gyro.begin())) return false;

    if (failed(accel.begin())) return false;

    if (failed(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ))) return false;

    if (failed(gyro.setRange(Bmi088Gyro::RANGE_2000DPS))) return false;

    if (failed(gyro.pinModeInt3(
                        Bmi088Gyro::PIN_MODE_PUSH_PULL,
                        Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)))
        return false;
            
    if (failed(gyro.mapDrdyInt3(true))) return false;

    if (failed(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ))) return false;

    if (failed(accel.setRange(Bmi088Accel::RANGE_24G))) return false;

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

float ImuTask::device_gyroRaw2Dps(const int16_t raw)
{
    return (float)raw * 2 * 2000 / 65536.f;
}

float ImuTask::device_accelRaw2Gs(const int16_t raw)
{
    return (float)raw * 2 * 24 / 65536.f;
}
