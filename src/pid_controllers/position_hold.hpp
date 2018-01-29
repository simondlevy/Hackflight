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

            // State variables
            bool  holdingPosition;
            float posHold[2];

        public:

            PositionHold(float _pidP, float _pidI, float _pidD) : PIDController(_pidP, _pidI, _pidD) 
            {
            }

            void init(void)
            {
                PIDController::init();
                holdingPosition = false;
            }

            void handleAuxSwitch(vehicle_state_t & vehicleState, demands_t & demands)
            {
                // Start
                if (demands.aux > 1) {
                    holdingPosition = true;
                    for (uint8_t k=0; k<2; ++k) {
                        posHold[k] = vehicleState.pose.position[k].value;
                    }
                }

                // Stop
                else {
                    holdingPosition = false;
                }
            }

            void updateDemands(vehicle_state_t & vehicleState, demands_t & demands, uint32_t currentTime)
            {
                if (holdingPosition) {

                    float error[2];

                    for (uint8_t k=0; k<2; ++k) {
                        error[k] = vehicleState.pose.position[k].value - posHold[k];
                    }

                    Debug::printf("%f %f\n", error[0], error[1]);

                    // D
                    //demands.roll -= vehicleState.pose.position[1].deriv * pidD;
                }
            }

            private:


    }; // class PositionHold


} // namespace hf
