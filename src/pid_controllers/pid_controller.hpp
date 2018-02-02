/* 
    pid_controller.hpp: abstract PID controller class for additional functionality
    (altitude-hold, loiter, etc.)

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

namespace hf {

    class PIDController {

        friend class Hackflight;

        protected: 

            float pidP;
            float pidI;
            float pidD;

            bool  holding;

            PIDController(float _pidP, float _pidI, float _pidD) : pidP(_pidP), pidI(_pidI), pidD(_pidD)
            {
                next = NULL;
            }

            void init(void) 
            {
                holding = false;
                previousTime = 0;
            }

            uint32_t getDeltaTime(uint32_t currentTime)
            {
                uint32_t dtime = currentTime - previousTime;
                previousTime = currentTime;
                return dtime;

            }

            virtual void handleAuxSwitch(vehicle_state_t & vehicleState, demands_t & demands) = 0;

            virtual void updateDemands(vehicle_state_t & vehicleState, demands_t & demands, uint32_t currentTime) = 0;


            
        private:

            // Simple linked-list support
            class PIDController * next; 

            uint32_t previousTime;

    }; // class PIDController


} // namespace hf
