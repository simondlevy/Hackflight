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
#include "barometer.hpp"
#include "accelerometer.hpp"
#include "model.hpp"
#include "debug.hpp"

namespace hf {

    class Altitude {

        private: // constants

            // Bounds
            const float pDeadband = 0.01f;
            const float dDeadband = 0.1f;
            const float pidMax    = 4.0f;
            const float pErrorMax = 1.0f;
            const float iErrorMax = 8.0f;

            // Complementry filter for accel/baro
            const float    cfAlt                  = 0.965f;
            const float    cfVel                  = 0.985f;

            // Keeps PID adjustment inside range
            const float throttleMargin            = 0.15f;

        public:

            void init(Board * _board, Model * _model);
            void handleAuxSwitch(uint8_t auxState, float throttleDemand);
            void computePid(bool armed);
            void update(float eulerAnglesRadians[3], bool armed, float & throttleDemand);

        private:

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

    void Altitude::init(Board * _board, Model * _model)
    {
        board = _board;
        model = _model;

        baro.init(_board);

        accel.init(_board);

        initialThrottleHold = 0;
        pid = 0;
        holdingAltitude = false;
        errorAltitudeI = 0;
        velocity = 0;
        previousT = 0;
    }

    void Altitude::handleAuxSwitch(uint8_t auxState, float throttleDemand)
    {
        // If board doesn't have baro, don't bother
        if (!board->extrasHaveBaro()) return;

        // Start
        if (auxState > 0) {
            holdingAltitude = true;
            initialThrottleHold = throttleDemand;
            altHold = baroAlt;
            pid = 0;
            errorAltitudeI = 0;
        }

        // Stop
        else {
            holdingAltitude = false;
        }
    }

    void Altitude::update(float eulerAnglesRadians[3], bool armed, float & throttleDemand)
    {
        // Throttle modification is synched to aquisition of new IMU data
        accel.update(eulerAnglesRadians, armed);

        if (holdingAltitude) {
            throttleDemand = Filter::constrainMinMax(initialThrottleHold + pid, throttleMargin, 1-throttleMargin);
        }
    }

    void Altitude::computePid(bool armed)
    {  
        // If board doesn't have baro, don't bother
        if (!board->extrasHaveBaro()) return;

        // Refresh the timer
        uint32_t dTimeMicros = updateTime();

        // Update the baro with the current pressure reading
        baro.update();

        // Calibrate baro AGL at rest
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
        velocity = Filter::complementary(velocity, (float)baroVel, cfVel);

        // P
        float error = altHold-baroAlt;
        error = Filter::constrainAbs(error, pErrorMax);
        error = Filter::deadband(error, pDeadband); 
        pid = Filter::constrainAbs(model->altP * error, pidMax);

        // I
        errorAltitudeI += (model->altI * error);
        errorAltitudeI = Filter::constrainAbs(errorAltitudeI, iErrorMax);
        pid += (errorAltitudeI * (dTimeMicros/1e6));

        // D
        float vario = Filter::deadband(velocity, dDeadband);
        pid -= Filter::constrainAbs(model->altD * vario, pidMax);

    } // computePid

    uint32_t Altitude::updateTime(void)
    {
        uint32_t currentT = (uint32_t)board->getMicros();
        uint32_t dTimeMicros = currentT - previousT;
        previousT = currentT;
        return dTimeMicros;
    }

} // namespace hf
