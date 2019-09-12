/*
   teensy40.hpp : Teensy4.0 Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2019 Simon D. Levy

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

#include <Wire.h>
#include "sentral.hpp"
#include "motors/standard2.hpp"

namespace hf {

    class Teensy40 : public SentralBoard {


        private:

            StandardMotor motors[4] = { 
                StandardMotor(14), 
                StandardMotor(15), 
                StandardMotor(9), 
                StandardMotor(2) 
            };

            void swap(float & a, float & b)
            {
                float tmp = a;
                a = b;
                b = tmp;
            }

         protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].set(value);
            }

            virtual void adjustGyrometer(float & gx, float & gy, float & gz) override
            { 
                gy = -gy;
                gz = -gz;
            }

            virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) override
            { 
                swap(qw, qx);
                swap(qy, qz);

                qx = -qx;
                qy = -qy;
            }

         public:

            Teensy40(void) 
                : SentralBoard(13)
            {
                // User D21 for 3.3V, D22 for ground to power SENTRAL
                powerPin(21, HIGH);
                powerPin(22, LOW);

                // Hang a bit 
                delay(100);

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);

                // Start the USFS
                SentralBoard::begin();

                // Initialize the motors
                for (uint8_t k=0; k<4; ++k) {
                    motors[k].begin();
                }

                // Hang a bit more
                delay(1000);
            }

    }; // class Teensy40

} // namespace hf
