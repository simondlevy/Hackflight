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
#include "debug.hpp"

namespace hf {
 
class Altitude {

    public:

        void init(const AltitudeConfig & _config, Board * _board, Model * _model);
        void start(float throttleDemand);
        void stop(void);
        void computePid(bool armed);
        void update(float eulerAnglesRadians[3], bool armed, float & throttleDemand);

    private:

        AltitudeConfig config;

        Board * board;
        Model * model;

        // Barometer
        Barometer baro;
        float baroAlt;               // meters

        // Accelerometer
        Accelerometer accel;

        // Fused
        float altHold;              // desired hold altitude, meters
        float errorAltitudeI;
        bool  holdingAltitude;
        float initialThrottleHold;  // [0,1]  
        float pid;
        float velocity;             // meters/sec

        // Microsecond dt for velocity computations
        uint32_t updateTime(void);
        uint32_t previousT;
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
    previousT = 0;
}

void Altitude::start(float throttleDemand)
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

void Altitude::update(float eulerAnglesRadians[3], bool armed, float & throttleDemand)
{
    // Throttle modification is synched to aquisition of new IMU data
    accel.update(eulerAnglesRadians, armed);

    if (holdingAltitude) {
        throttleDemand = Filter::constrainMinMax(initialThrottleHold + pid, config.throttleMargin, 1-config.throttleMargin);
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
    velocity = accel.getVerticalVelocity(dTimeMicros);

    // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accelerometer velocity without loosing the phase, 
    // i.e without delay.
    float baroVel = baro.getVelocity(dTimeMicros);
    velocity = Filter::complementary(velocity, (float)baroVel, config.cfVel);

    // P
    float error = Filter::constrainAbs(altHold-baroAlt, config.pErrorMax);
    error = Filter::deadband(error, config.pDeadband); 
    pid = Filter::constrainAbs(model->altP * error, config.pidMax);

    // I
    errorAltitudeI += (model->altI * error);
    errorAltitudeI = Filter::constrainAbs(errorAltitudeI, config.iErrorMax);
    pid += (errorAltitudeI * (dTimeMicros/1e6));

    // D
    float vario = Filter::deadband(velocity, config.dDeadband);
    pid -= Filter::constrainAbs(model->altD * vario, config.pidMax);

} // computePid

uint32_t Altitude::updateTime(void)
{
    uint32_t currentT = (uint32_t)board->getMicros();
    uint32_t dTimeMicros = currentT - previousT;
    previousT = currentT;
    return dTimeMicros;
}

} // namespace hf
