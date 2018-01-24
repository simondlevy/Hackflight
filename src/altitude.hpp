/* 
    altitude.hpp: Altitude estimation and PID via barometer/accelerometer fusion

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
#include "datatypes.hpp"

namespace hf {

    class Altitude {

        private: 

            // Bounds
            const float pDeadband = 0.01f;
            const float dDeadband = 0.1f;
            const float pidMax    = 4.0f;
            const float pErrorMax = 4.0f;
            const float iErrorMax = 8.0f;

            // Complementry filter for accel/baro
            const float    cfAlt  = 0.965f;
            const float    cfVel  = 0.985f;

            // Keeps PID adjustment inside range
            const float throttleMargin = 0.15f;

            Board * board;
            Model * model;

            // Barometer
            Barometer baro;

            // Accelerometer
            Accelerometer accel;

            float altHold;
            float errorAltitudeI;
            bool  holdingAltitude;
            float initialThrottleHold;  // [0,1]  
            float pid;

        public:

            void init(Board * _board, Model * _model)
            {
                board = _board;
                model = _model;

                baro.init(_board);

                accel.init(_board);

                initialThrottleHold = 0;
                pid = 0;
                holdingAltitude = false;
                errorAltitudeI = 0;
            }

            void handleAuxSwitch(vehicle_state_t * vehicleState, demands_t & demands)
            {
                // Start
                if (demands.aux > 0) {
                    holdingAltitude = true;
                    initialThrottleHold = demands.throttle;
                    altHold = vehicleState->pose.position[2].value;
                    pid = 0;
                    errorAltitudeI = 0;
                }

                // Stop
                else {
                    holdingAltitude = false;
                }
            }

            void updateDemands(vehicle_state_t * vehicleState, demands_t & demands)
            {
                if (holdingAltitude) {

                    // Extract altitude, vertical velocity from vehicle state
                    stateval_t posZ = vehicleState->pose.position[2];
                    float altitude = posZ.value;
                    float velocity = posZ.deriv;

                    // Refresh the timer
                    static uint32_t previousTime;
                    uint32_t currentTime = board->getMicroseconds();
                    uint32_t dTimeMicros = currentTime - previousTime;
                    previousTime = currentTime;

                    // P
                    float error = altHold-altitude;
                    if (holdingAltitude) Debug::printf("%f - %f = %f", altHold, altitude, error);
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

                    demands.throttle = Filter::constrainMinMax(initialThrottleHold + pid, throttleMargin, 1-throttleMargin);
                }
            }

            void fuseWithImu(vehicle_state_t * state)
            {
                // Throttle modification is synched to aquisition of new IMU data
                accel.update(state->pose.orientation, state->armed);
            }

            void estimate(vehicle_state_t * state)
            {  
                // Refresh the timer
                static uint32_t previousTime;
                uint32_t currentTime = board->getMicroseconds();
                uint32_t dTimeMicros = currentTime - previousTime;
                previousTime = currentTime;

                // Update the baro with the current pressure reading
                baro.update();

                // Calibrate baro AGL at rest
                if (!state->armed) {
                    baro.calibrate();
                    return;
                }

                // Get estimated altitude from barometer
                state->pose.position[2].value = baro.getAltitude();

                // Apply complementary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
                // By using CF it's possible to correct the drift of integrated accelerometer velocity without loosing the phase, 
                // i.e without delay.
                float accelVel = accel.getVerticalVelocity(dTimeMicros);
                float baroVel = baro.getVelocity(dTimeMicros);
                state->pose.position[2].deriv = Filter::complementary(accelVel, (float)baroVel, cfVel);

            }

    }; // class Altitude


} // namespace hf
