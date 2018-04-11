/*
   mixer.hpp : Mixer class header

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "filter.hpp"
#include "debug.hpp"
#include "datatypes.hpp"

#include <cstring>

namespace hf {

    class Mixer {

        public:

            // This is set by MSP
            float  motorsDisarmed[4];

            void init(Board * _board)
            {
                //  T   A    E   R
                mixerQuadX[0] = { +1, -1,  +1, +1 };    // right rear
                mixerQuadX[1] = { +1, -1,  -1, -1 };    // right front
                mixerQuadX[2] = { +1, +1,  +1, -1 };    // left rear
                mixerQuadX[3] = { +1, +1,  -1, +1 };    // left front

                board = _board;

                // set disarmed motor values
                for (uint8_t i = 0; i < 4; i++)
                    motorsDisarmed[i] = 0;
            }

            void runArmed(demands_t demands)
            {
                float motors[4];

                for (uint8_t i = 0; i < 4; i++) {

                    motors[i] = 
                        (demands.throttle * mixerQuadX[i].throttle + 
                         demands.roll     * mixerQuadX[i].roll +     
                         demands.pitch    * mixerQuadX[i].pitch +   
                         demands.yaw      * mixerQuadX[i].yaw);      
                }

                float maxMotor = motors[0];

                for (uint8_t i = 1; i < 4; i++)
                    if (motors[i] > maxMotor)
                        maxMotor = motors[i];

                for (uint8_t i = 0; i < 4; i++) {

                    // This is a way to still have good gyro corrections if at least one motor reaches its max
                    if (maxMotor > 1) {
                        motors[i] -= maxMotor - 1;
                    }

                    // Keep motor values in interval [0,1]
                    motors[i] = Filter::constrainMinMax(motors[i], 0, 1);
                }

                for (uint8_t i = 0; i < 4; i++) {
                    board->writeMotor(i, motors[i]);
                }
            }

            // This is how we can spin the motors from the GCS
            void runDisarmed(void)
            {
                for (uint8_t i = 0; i < 4; i++) {
                    board->writeMotor(i, motorsDisarmed[i]);
                }
            }

            void cutMotors(void)
            {
                for (uint8_t i = 0; i < 4; i++) {
                    board->writeMotor(i, 0);
                }
            }


        private:

            Board * board;

            // Custom mixer data per motor
            typedef struct motorMixer_t {
                int8_t throttle; // T
                int8_t roll; 	 // A
                int8_t pitch;	 // E
                int8_t yaw;	     // R
            } motorMixer_t;

            motorMixer_t mixerQuadX[4];
    };

} // namespace
