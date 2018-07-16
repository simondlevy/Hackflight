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
#include "filter.hpp"

namespace hf {

    class State {

        private:

            LowPassFilter _rangefinderLpf = LowPassFilter(20);

        public:

            float eulerAngles[3];
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

                _rangefinderLpf.init();
            }

            void updateGyrometer(float gyroRates[3], uint32_t microseconds)
            {
                (void)gyroRates;
                (void)microseconds;
            }

            void updateAccelerometer(float accelGs[3], uint32_t microseconds)
            {
                (void)accelGs;
                (void)microseconds;
            }

            void updateBarometer(float pressure)
            {
                (void)pressure;
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

            void updateOpticalFlow(float flow[2])
            {
                velocityForward   = flow[0];
                velocityRightward = flow[1];
            }

            void updateRangefinder(float distance, uint32_t microseconds)
            {
                float seconds = microseconds / 1.e6;
                static float _seconds;
                static float _altitude;

                // Compensate for effect of pitch, roll on rangefinder reading
                altitude =  distance * cos(eulerAngles[0]) * cos(eulerAngles[1]);

                // Use first-differenced, low-pass-filtered altitude as variometer
                variometer = _rangefinderLpf.update((altitude-_altitude) / (seconds-_seconds));

                _seconds = seconds;
                _altitude = altitude;
            }

    };  // class State

} // namespace
