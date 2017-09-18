/* 
   altitude.hpp: Altitude hold via barometer/accelerometer fusion

   Adapted from

    https://github.com/multiwii/baseflight/blob/master/src/imu.c

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
   along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "filter.hpp"
#include "config.hpp"
#include "barometer.hpp"
#include "accelerometer.hpp"
#include "model.hpp"

namespace hf {
 
class Altitude {

    public:

        void init(const AltitudeConfig & _config, Board * _board, Model * _model);
        void start(uint16_t throttleDemand);
        void stop(void);
        void computePid(bool armed);
        void updateAccelerometer(float eulerAnglesRadians[3], bool armed);
        void modifyThrottleDemand(int16_t & throttleDemand);

    private:

        AltitudeConfig config;

        Board * board;
        Model * model;

        // Barometer
        Barometer baro;
        int32_t   baroAlt;               // cm

        // Accelerometer
        Accelerometer accel;

        // Fused
        int32_t  altHold;
        int16_t  errorAltitudeI;
        bool     holdingAltitude;
        uint16_t initialThrottleHold;  // PWM usec
        int32_t  pid;
        float    velocity;             // cm/sec

        // Microsecond dt for velocity computations
        uint32_t updateTime(void);
};

/********************************************* CPP ********************************************************/

void Altitude::init(const AltitudeConfig & _config, Board * _board, Model * _model)
{
    memcpy(&config, &_config, sizeof(AltitudeConfig));

    board = _board;
    model = _model;

    baro.init(config.baro, _board);

    accel.init(config.accel, _board);

    initialThrottleHold = 0;
    pid = 0;
    holdingAltitude = false;
    errorAltitudeI = 0;
    velocity = 0;
}

void Altitude::start(uint16_t throttleDemand)
{
    holdingAltitude = true;
    initialThrottleHold = throttleDemand;
    altHold = baroAlt;
    pid = 0;
    errorAltitudeI = 0;
}

void Altitude::stop(void)
{
    holdingAltitude = false;
}

void Altitude::modifyThrottleDemand(int16_t & throttleDemand)
{
    if (holdingAltitude) {

        throttleDemand = Filter::constrainMinMax(initialThrottleHold + pid, config.throttleMin, config.throttleMax);
    }
}

void Altitude::computePid(bool armed)
{  
    // Refresh the timer
    uint32_t dTimeMicros = updateTime();
 
    // Update the baro with the current pressure reading
    baro.update();

    // Calibrate baro AGL while not armed
    if (!armed) {
        baro.calibrate();
        return;
    }

    // Get estimated altitude from barometer
    baroAlt = baro.getAltitude();

    // Get estimated vertical velocity from accelerometer
    velocity += accel.getVelocity(dTimeMicros);

    // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accelerometer velocity without loosing the phase, 
    // i.e without delay.
    int32_t baroVel = baro.getVelocity(dTimeMicros);
    velocity = Filter::complementary(velocity, baroVel, config.cfVel);

    // P
    int32_t error = Filter::constrainAbs(altHold - baroAlt, config.pErrorMax);
    error = Filter::deadband(error, config.pDeadband); 
    pid = Filter::constrainAbs((int16_t)(model->altP * error), config.pidMax);

    // I
    errorAltitudeI += (int16_t)(model->altI * error);
    errorAltitudeI = Filter::constrainAbs(errorAltitudeI, config.iErrorMax);
    pid += errorAltitudeI * (dTimeMicros/1e6);

    // D
    int32_t vario = Filter::deadband(velocity, config.dDeadband);
    pid -= Filter::constrainAbs((int16_t)(model->altD * vario), config.pidMax);

} // computePid

void Altitude::updateAccelerometer(float eulerAnglesRadians[3], bool armed)
{
    // Throttle modification is synched to aquisition of new IMU data
    accel.update(eulerAnglesRadians, armed);
}

uint32_t Altitude::updateTime(void)
{
    static uint32_t previousT;
    uint32_t currentT = board->getMicros();
    uint32_t dTimeMicros = currentT - previousT;
    previousT = currentT;
    return dTimeMicros;
}

} // namespace hf
