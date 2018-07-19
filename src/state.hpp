/*
   state.hpp : state estimation

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

#pragma once

#include <cmath>

#include "debug.hpp"
#include "filters.hpp"

namespace hf {

    class State {

        private:


        public:

            float eulerAngles[3];
            float angularVelocities[3];
            bool  armed;
            float altitude;
            float variometer;
            float velocityForward;  
            float velocityRightward; 

            void init(void)
            {
                armed = false;
                altitude = 0;
                variometer = 0;
                velocityForward = 0;
                velocityRightward = 0;
            }

            void updateQuaternion(float q[4])
            {
                eulerAngles[0] = atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
                eulerAngles[1] =  asin(2.0f*(q[1]*q[3]-q[0]*q[2]));
                eulerAngles[2] = atan2(2.0f*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (eulerAngles[2] < 0) {
                    eulerAngles[2] += 2*M_PI;
                }
            }

            void updateGyrometer(float gyroRates[3], float seconds)
            {
                (void)seconds;

                memcpy(angularVelocities, gyroRates, 3*sizeof(float));
            }

    };  // class State

} // namespace
