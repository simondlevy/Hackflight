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

        protected:

            virtual void adjustGyrometer(float & x, float & y, float & z) override
            { 
            }

            virtual void adjustEulerAngles(float & x, float & y, float & z) override
            { 
                x = M_PI - x;
                z = -z;
            }

     }; // class USFSMAX_Inverted

} // namespace hf
