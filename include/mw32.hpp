/*
   mw32.hpp : IMU routines for Multiwii32-derived boards (Naze, Beef, AlienflightF3)

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "imu.hpp"


namespace hf {

class MW32 : public Board {

    public:

        virtual void imuInit(ImuConfig & imuConfig) override; 
        virtual void imuRestartCalibration(void) override; 
        virtual bool imuAccelCalibrated(void) override; 
        virtual bool imuGyroCalibrated(void) override; 
        virtual void imuUpdate(uint32_t currentTime, bool armed) override; 
        virtual void imuGetEulerAngles(int16_t eulerAngles[3]) override; 
        virtual void imuGetRawGyro(int16_t gyroRaw[3]) override; 

    private:

        IMU imu;

}; // class MW32

void MW32::imuInit(ImuConfig &imuConfig)
{
    imu.init(imuConfig, this);
}

void MW32::imuRestartCalibration(void) 
{
    imu.restartCalibration();
}

bool MW32::imuAccelCalibrated(void) 
{
    return imu.accelCalibrated();
}

bool MW32::imuGyroCalibrated(void) 
{
    return imu.gyroCalibrated();
}
void MW32::imuUpdate(uint32_t currentTime, bool armed) 
{
    imu.update(currentTime, armed);
}

void MW32::imuGetEulerAngles(int16_t eulerAngles[3]) 
{
    imu.getEulerAngles(eulerAngles);
}

void MW32::imuGetRawGyro(int16_t gyroRaw[3]) 
{
    imu.getRawGyro(gyroRaw);
}

} // namespace hf
