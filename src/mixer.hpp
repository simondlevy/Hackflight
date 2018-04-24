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

        private:

            Board * board;

            // Custom mixer data per motor
            typedef struct motorMixer_t {
                int8_t throttle; // T
                int8_t roll; 	 // A
                int8_t pitch;	 // E
                int8_t yaw;	     // R
            } motorMixer_t;

            // Arbitrary
            static const uint8_t MAXMOTORS = 20;

            float motorsPrev[MAXMOTORS];

            void writeMotor(uint8_t index, float value)
            {
                // Avoid sending the motor the same value over and over
                if (motorsPrev[index] != value) {
                    board->writeMotor(index,value);
                }

                motorsPrev[index] = value;
            }

        protected:

            motorMixer_t motorDirections[MAXMOTORS];

            Mixer(uint8_t _nmotors)
            {
                nmotors = _nmotors;
            }

        public:

            // These are also use by MSP
            float  motorsDisarmed[MAXMOTORS];
            uint8_t nmotors;

            void init(Board * _board)
            {
                board = _board;

                // set disarmed, previous motor values
                for (uint8_t i = 0; i < nmotors; i++) {
                    motorsDisarmed[i] = 0;
                    motorsPrev[i] = 0;
                }

            }

            void runArmed(demands_t demands)
            {
                float motors[MAXMOTORS];

                for (uint8_t i = 0; i < nmotors; i++) {

                    motors[i] = 
                        (demands.throttle * motorDirections[i].throttle + 
                         demands.roll     * motorDirections[i].roll +     
                         demands.pitch    * motorDirections[i].pitch +   
                         demands.yaw      * motorDirections[i].yaw);      
                }

                float maxMotor = motors[0];

                for (uint8_t i = 1; i < nmotors; i++)
                    if (motors[i] > maxMotor)
                        maxMotor = motors[i];

                for (uint8_t i = 0; i < nmotors; i++) {

                    // This is a way to still have good gyro corrections if at least one motor reaches its max
                    if (maxMotor > 1) {
                        motors[i] -= maxMotor - 1;
                    }

                    // Keep motor values in interval [0,1]
                    motors[i] = Filter::constrainMinMax(motors[i], 0, 1);
                }

                for (uint8_t i = 0; i < nmotors; i++) {
                    writeMotor(i, motors[i]);
                }
            }

            // This is how we can spin the motors from the GCS
            void runDisarmed(void)
            {
                for (uint8_t i = 0; i < nmotors; i++) {
                    writeMotor(i, motorsDisarmed[i]);
                }
            }

            void cutMotors(void)
            {
                for (uint8_t i = 0; i < nmotors; i++) {
                    board->writeMotor(i, 0);
                }
            }


    };

} // namespace
