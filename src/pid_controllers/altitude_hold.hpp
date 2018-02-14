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
            float initialThrottleHold;  // [0,1]  

            // No velocity control for now
            bool velocityControl = false;

            float errorI;

        public:

            AltitudeHold(float _pidP, float _pidI, float _pidD) : PIDController(_pidP, _pidI, _pidD) { }

            void init(void)
            {
                PIDController::init();
                errorI = 0;
                initialThrottleHold = 0;
            }

            void handleAuxSwitch(vehicle_state_t & vehicleState, demands_t & demands)
            {
                // Start
                if (demands.aux > 0) {
                    holding = true;
                    initialThrottleHold = demands.throttle;
                    altHold = vehicleState.position.values[2];
                    errorI = 0;
                }

                // Stop
                else {
                    holding = false;
                }
            }

            void updateDemands(vehicle_state_t & vehicleState, demands_t & demands, uint32_t currentTime)
            {
                // Refresh the timer
                uint32_t dtime = getDeltaTime(currentTime);

                if (holding) {

                    // Altitude P-Controller
                    if (!velocityControl) {
                        //Debug::printf("%d %d\n", (int)altHold, (int)vehicleState.position.values[2]);
                        //int32_t error = Filter::constrain(altHold - fusedAlt, -500, 500);
                        //error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
                        //setVel = constrain((cfg.P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
                    } else {
                        //setVel = setVelocity;
                    }

                    // Velocity PID-Controller
                    // P
                    //error = setVel - vel_tmp;
                    //altitudePid = constrain((cfg.P8[PIDVEL] * error / 32), -300, +300);

                    // I
                    //errorVelocityI += (cfg.I8[PIDVEL] * error);
                    //errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
                    //altitudePid += errorVelocityI / 8196;     // I in the range of +/-200

                    // D
                    //altitudePid -= constrain(cfg.D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);
                }
            }

    }; // class AltitudeHold


} // namespace hf
