/*
   Support for USFSMAX IMU mounted at a 90-degree angle

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

    class USFSMAX_Rotated : public USFSMAX {

        private:

            void swap(float & a, float & b)
            {
                float tmp = a;
                a = b;
                b = tmp;
            }

        protected:

            /*
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
            */

    }; // class USFSMAX_Rotated

} // namespace hf
