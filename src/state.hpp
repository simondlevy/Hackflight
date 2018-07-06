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

namespace hf {

    class State {

        private:

            float _altitudePrev; // XXX simulate variometer for now
            uint32_t _microsecondsPrev;

        public:

            float eulerAngles[3];
            bool armed;
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

                _altitudePrev = 0;
                _microsecondsPrev = 0;
            }

            void updateGyrometer(float gyroRate[3])
            {
                (void)gyroRate;
            }

            void updateAccelerometer(float accelGs[3])
            {
                (void)accelGs;
                //Debug::printf("X: %+2.2f  Y: %+2.2f  Z: %+2.2f\n", accelGs[0], accelGs[1], accelGs[2]);
            }

            void updateBarometer(float pressure)
            {
                // Pascals to meters: see
                //  https://www.researchgate.net/file.PostFileLoader.html?id=5409cac4d5a3f2e81f8b4568&assetKey=AS%3A273593643012096%401442241215893
               // altitude = 44331.5 - 4946.62 * pow(pressure, 0.190263);

                //variometer = altitude - _altitudePrev;
                //_altitudePrev = altitude;
            }

            void updateQuaternion(float q[4])
            {
                eulerAngles[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
                eulerAngles[1] = asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
                eulerAngles[2] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 

                // Convert heading from [-pi,+pi] to [0,2*pi]
                if (eulerAngles[2] < 0) {
                    eulerAngles[2] += 2*M_PI;
                }
            }

            void updateOpticalFlow(float velFore, float velRight)
            {
                velocityForward  = velFore;
                velocityRightward = velRight;
            }

            void updateSonar(float distance, uint32_t microseconds)
            {
                altitude = distance;
                variometer = (altitude - _altitudePrev) / ((microseconds-_microsecondsPrev) / 1.e6);
                Debug::printf("Vario: %+3.3f", variometer);
                _altitudePrev = altitude;
                _microsecondsPrev = microseconds;
            }

    };  // class State

} // namespace
