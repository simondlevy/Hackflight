/* 
    altitude_hold.hpp: PID controller for altitude hold

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
#include "model.hpp"
#include "debug.hpp"
#include "datatypes.hpp"
#include "pid_controller.hpp"

namespace hf {

    class AltitudeHold : public PIDController {

        private: 

            // Bounds
            const float pDeadband = 0.01f;
            const float dDeadband = 0.1f;
            const float pidMax    = 4.0f;
            const float pErrorMax = 4.0f;
            const float iErrorMax = 8.0f;

            // Keeps PID adjustment inside range
            const float throttleMargin = 0.15f;

            // State variables
            float altHold;
            bool  holdingAltitude;
            float initialThrottleHold;  // [0,1]  

        public:

            AltitudeHold(float _pidP, float _pidI, float _pidD) : PIDController(_pidP, _pidI, _pidD) 
            {
            }

            void init(void)
            {
                PIDController::init();
                initialThrottleHold = 0;
            }

            void handleAuxSwitch(vehicle_state_t & vehicleState, demands_t & demands)
            {
                // Start
                if (demands.aux > 0) {
                    holdingAltitude = true;
                    initialThrottleHold = demands.throttle;
                    altHold = vehicleState.pose.position[2].value;
                    pid = 0;
                    errorI = 0;
                }

                // Stop
                else {
                    holdingAltitude = false;
                }
            }

            void updateDemands(vehicle_state_t & vehicleState, demands_t & demands, uint32_t currentTime)
            {
                if (holdingAltitude) {

                    // Extract altitude, vertical velocity from vehicle state
                    stateval_t posZ = vehicleState.pose.position[2];
                    float altitude = posZ.value;
                    float velocity = posZ.deriv;

                    // Refresh the timer
                    static uint32_t previousTime;
                    uint32_t dTimeMicros = currentTime - previousTime;
                    previousTime = currentTime;

                    // P
                    float error = altHold-altitude;
                    error = Filter::constrainAbs(error, pErrorMax);
                    error = Filter::deadband(error, pDeadband); 
                    pid = Filter::constrainAbs(pidP * error, pidMax);


                    // I
                    errorI += (pidI * error);
                    errorI = Filter::constrainAbs(errorI, iErrorMax);
                    pid += (errorI * (dTimeMicros/1e6));

                    // D
                    float vario = Filter::deadband(velocity, dDeadband);
                    pid -= Filter::constrainAbs(pidD * vario, pidMax);

                    demands.throttle = Filter::constrainMinMax(initialThrottleHold + pid, throttleMargin, 1-throttleMargin);
                }
            }

    }; // class AltitudeHold


} // namespace hf
