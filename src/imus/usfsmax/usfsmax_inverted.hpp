/*
   Support for USFSMAX IMU mounted upside-down

   Copyright (c) 2020 Simon D. Levy

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

#include "imus/usfsmax.hpp"

namespace hf {

    class USFSMAX_Inverted : public USFSMAX_IMU {

        private:

            void swap(float & a, float & b)
            {
                float tmp = a;
                a = b;
                b = tmp;
            }

        protected:

            virtual void adjustGyrometer(float & gx, float & gy, float & gz) override
            { 
                gz = -gz;

                Serial.print("gx: ");
                Serial.print(gx);
                Serial.print("\tgy: ");
                Serial.print(gy);
                Serial.print("\tgz: ");
                Serial.println(gz);
            }

            virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) override
            { 
                /*
                Serial.print("qw: ");
                Serial.print(qw);
                Serial.print("\tqx: ");
                Serial.print(qx);
                Serial.print("\tqy: ");
                Serial.print(qy);
                Serial.print("\tqz: ");
                Serial.println(qz);
                */
            }

    }; // class USFSMAX_Inverted

} // namespace hf
