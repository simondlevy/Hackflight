/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY without even the implied warranty of MERCHANTABILITY or FITNESS
   FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
   details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include "usfs.h"

static volatile UsfsImu::gyroDev_t m_gyroDev;

void UsfsImu::interruptHandler(void)
{
    m_gyroDev.gotNewData = true;
   *m_gyroDev.interruptCountPtr += 1;
   *m_gyroDev.syncTimePtr = micros();
}

bool UsfsImu::devGyroIsReady(void)
{
    bool result = false;

    if (m_gyroDev.gotNewData) { 

        m_gyroDev.gotNewData = false;  

        uint8_t eventStatus = usfsCheckStatus(); 

        if (usfsEventStatusIsError(eventStatus)) { 
            usfsReportError(eventStatus);
        }

        if (usfsEventStatusIsGyrometer(eventStatus)) { 
            usfsReadGyrometerRaw(m_gyroAdc);
            result = true;
        }

        if (usfsEventStatusIsQuaternion(eventStatus)) { 
            usfsReadQuaternion(m_qw, m_qx, m_qy, m_qz);
        }
    } 

    return result;
}

void UsfsImu::devInit(
                uint32_t * gyroSyncTimePtr, uint32_t * gyroInterruptCountPtr)
{
    m_gyroDev.syncTimePtr = gyroSyncTimePtr;
    m_gyroDev.interruptCountPtr = gyroInterruptCountPtr;

    Wire.setClock(400000); 
    delay(100);

    usfsLoadFirmware(); 

    usfsBegin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE_TENTH,
            GYRO_RATE_TENTH,
            BARO_RATE,
            INTERRUPT_ENABLE);

    pinMode(m_interruptPin, INPUT);
    attachInterrupt(m_interruptPin, interruptHandler, RISING);  

    // Clear interrupts
    usfsCheckStatus();
}

int16_t UsfsImu::devReadRawGyro(uint8_t k)
{
    return m_gyroAdc[k];
}

auto UsfsImu::getEulerAngles(const uint32_t time) -> Axes
{
    (void)time;

    const auto qw = m_qw;
    const auto qx = m_qx;
    const auto qy = m_qy;
    const auto qz = m_qz;

    const auto phi = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
    const auto theta = asin(2.0f*(qx*qz-qw*qy));
    const auto psi = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);

    // Convert heading from [-pi,+pi] to [0,2*pi]
    return Axes(phi, theta, psi + (psi < 0 ? 2*M_PI : 0)); 
}

UsfsImu::UsfsImu(uint8_t interruptPin) : Imu(GYRO_SCALE_DPS)
{
    m_interruptPin = interruptPin;
}
