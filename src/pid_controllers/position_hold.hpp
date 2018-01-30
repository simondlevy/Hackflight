/* 
    position_hold.hpp: PID controller for position hold (forward/backward/left/right)

    Adapted from

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

    class PositionHold : public PIDController {

        private: 

            // Bounds
            const float pidMax    = 0.05f;
            const float iErrorMax = 8.0f;

            // State variables
            float posHold[2];

            float errorI[2];

        public:

            PositionHold(float _pidP, float _pidI, float _pidD) : PIDController(_pidP, _pidI, _pidD) 
            {
            }

            void init(void)
            {
                PIDController::init();
                for (uint8_t k=0; k<2; ++k) {
                    errorI[k] = 0;
                }
            }

            void handleAuxSwitch(vehicle_state_t & vehicleState, demands_t & demands)
            {
                // Start
                if (demands.aux > 1) {
                    holding = true;
                    for (uint8_t k=0; k<2; ++k) {
                        posHold[k] = vehicleState.pose.position[k].value;
                        errorI[k] = 0;
                    }
                }

                // Stop
                else {
                    holding = false;
                }
            }

            void updateDemands(vehicle_state_t & vehicleState, demands_t & demands, uint32_t currentTime)
            {
                if (holding) {

                    float error[2];
                    float pid[2];

                    stateval_t * position = vehicleState.pose.position;

                    // Refresh the timer
                    static uint32_t previousTime;
                    uint32_t dTimeMicros = currentTime - previousTime;
                    previousTime = currentTime;

                    for (uint8_t k=0; k<2; ++k) {

                        // P
                        error[k] = posHold[k] - position[k].value;
                        pid[k] = Filter::constrainAbs(pidP * error[k], pidMax);

                        // I
                        errorI[k] += (pidI * error[k]);
                        errorI[k] = Filter::constrainAbs(errorI[k], iErrorMax);
                        pid[k] += (errorI[k] * (dTimeMicros/1e6));

                        // D
                        pid[k] -= Filter::constrainAbs(pidD * position[k].deriv, pidMax);

                    }

                    demands.roll  += pid[1];
                    demands.pitch += pid[0];
                }
            }

            private:


    }; // class PositionHold


} // namespace hf
