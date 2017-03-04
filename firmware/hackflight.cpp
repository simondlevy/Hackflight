/*
   hackflight.cpp : Hackflight class implementation and hooks to setup(), loop()

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/mw.c

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

#include <math.h>

#include "hackflight.hpp"

#include <string.h>

#if defined(STM32)
extern "C" { 
#endif

void Hackflight::initialize(uint16_t acc1G, float gyroScale, uint32_t looptimeUsec, uint32_t gyroCalibrationMsec)
{
    this->imuLooptimeUsec = looptimeUsec;
    
    // compute cycles for calibration based on board's time constant
    this->calibratingGyroCycles = (uint16_t)(1000. * gyroCalibrationMsec / this->imuLooptimeUsec);
    this->calibratingAccCycles  = (uint16_t)(1000. * CONFIG_CALIBRATING_ACC_MSEC  / this->imuLooptimeUsec);

    // initializing timing tasks
    this->imuTask.init(this->imuLooptimeUsec);
    this->rcTask.init(CONFIG_RC_LOOPTIME_MSEC * 1000);
    this->accelCalibrationTask.init(CONFIG_CALIBRATE_ACCTIME_MSEC * 1000);
    this->altitudeEstimationTask.init(CONFIG_ALTITUDE_UPDATE_MSEC * 1000);

    // initialize our external objects with objects they need
    this->rc.init();
    this->stab.init();
    this->imu.init(acc1G, gyroScale, this->calibratingGyroCycles, this->calibratingAccCycles);
    this->mixer.init(&this->rc, &this->stab); 
    this->msp.init(&this->imu, &this->mixer, &this->rc);

    // do any extra initializations (baro, sonar, etc.)
    this->board.extrasInit(&msp);

    // always do gyro calibration at startup
    this->calibratingG = this->calibratingGyroCycles;

    // assume shallow angle (no accelerometer calibration needed)
    this->haveSmallAngle = true;

    // ensure not armed
    this->armed = false;
    
} // intialize

void Hackflight::setAccelReading(int16_t linear_accel[3]) 
{
    this->accelADC[0] = linear_accel[0];
    this->accelADC[1] = linear_accel[1];
    this->accelADC[2] = linear_accel[2];
}

void Hackflight::setGyrolReading(int16_t angular_velocity[3])
{
    this->gyroADC[0] = angular_velocity[0];
    this->gyroADC[1] = angular_velocity[1];
    this->gyroADC[2] = angular_velocity[2];
}

void Hackflight::setRC(float * channels, uint8_t count)
{
    for (uint8_t k=0; k<count; ++k) {
        this->rc.command[k] = (int16_t)channels[k];
    }
}

void Hackflight::getControls(float * controls, uint8_t count)
{

   // update PIDs and compute motor values
   this->update();

    // grab motor values
    for (uint8_t k=0; k<count; ++k) {
        controls[k] = this->mixer.motors[k];
    }
}

void Hackflight::arm(void)
{
    this->armed = true;
}

void Hackflight::disarm(void)
{
    this->armed = false;
}

void Hackflight::update(void)
{
    // update stability PID controller 
    this->stab.update(this->rc.command, this->gyroADC, this->imu.angle);

    // update mixer
    this->mixer.update(this->armed);
}

#if defined(STM32)
} // extern "C"
#endif
